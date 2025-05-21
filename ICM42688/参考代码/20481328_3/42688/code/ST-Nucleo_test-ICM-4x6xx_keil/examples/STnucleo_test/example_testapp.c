/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

/* 
 * Select communication between STNucleo and ICM20690 by setting 0/1 to one of the following defines
 */

#include <stdint.h>

/* InvenSense */
#include "Invn/Devices/DeviceIcm20690.h"
#include "Invn/Devices/DeviceSmartMotion.h"
#include "Invn/Devices/DeviceAggregator.h"
#include "Invn/Devices/DeviceAk0991x.h"
#include "Invn/Utils/Message.h"
#include "Invn/Utils/ErrorHelper.h"
#include "Invn/Devices/Drivers/IddWrapper/IddWrapperTransportUart.h"
#include "Invn/Devices/Drivers/IddWrapper/IddWrapperProtocol.h"

/* board driver */
#include "uart.h"
#include "rtc_timer.h"
#include "delay.h"
#include "gpio.h"
#include "i2c_master.h"
#include "spi_master.h"
#include "timer.h"

/* std */
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

//#include "ICM-406xx.h"
#include "ICM-4x6xx.h"
#include "example-selftest.h"

/******************************************************************************/
/* Hardware configuration                                                     */
/******************************************************************************/

//#define SELF_TEST_MAIN    //comment out this line for sensor data example; otherwise for sensor self-test

//#define SERIF_TYPE_SPI 0
#define SERIF_TYPE_I2C 1

//#define INTERRUPT_MODE   //comment out this line for polling-mode to get sensor data
//#define DATA_FIFO_MODE
//#define ICM42688_HIRES_MODE		//20-bit hires FIFO mode of icm42688 - comment this line for regular 16bit FIFO mode
   
#ifdef ICM42688_HIRES_MODE
	int fifo_data_20bit = 1;
#else
	int fifo_data_20bit = 0;
#endif
        
#if SERIF_TYPE_SPI
  int bus_type = 0;  //SPI
#elif SERIF_TYPE_I2C
  int bus_type = 1;  //I2C
#else
  int bus_type = -1; //error
#endif

#define UART_LOG_FIFO_SIZE  4096
#define UART_MAIN_FIFO_SIZE  4096
#define UART_RX_FIFO_SIZE    256
#define UART_TX_FIFO_SIZE   4096


/*
 * Select the Nucleo Carrier board version
 */
#define CARRIER_BOARD_REV_A 0
#define CARRIER_BOARD_REV_B 1

//#define ICM_I2C_ADDR     0x68 /* I2C slave address for ICM406xx */
//#define ICM_I2C_ADDR     0x69 /* I2C slave address for ICM406xx */
  
/*
 * Set to 1 to swap main and log UART
 * Some instability (BSOD) and byte loss were observed with main UART (going through ST-Link bridge) under Windows.
 * Using log UART with FTDI allows more reliable communication (with the downside of having to use two USB cables even
 * if we don't care about traces)
 */
#define SWAP_UART 0

/*
 * Overload UART id to swap between the two
 */
#if CARRIER_BOARD_REV_A
  #if !SWAP_UART
	#define MAIN_UART_ID UART_MAIN
	#define LOG_UART_ID  UART_LOG
  #else
	#define MAIN_UART_ID UART_LOG
	#define LOG_UART_ID  UART_MAIN
  #endif
#elif CARRIER_BOARD_REV_B
  #if !SWAP_UART
	#define MAIN_UART_ID UART_MAIN
//	#define LOG_UART_ID  UART_LOG_HDSK  //use FTDI/UART1 port for message logging
	#define LOG_UART_ID  UART_LOG
  #else
	#define MAIN_UART_ID UART_LOG_HDSK
	#define LOG_UART_ID  UART_MAIN
  #endif
#endif

static uint8_t uart_log_buffer[UART_LOG_FIFO_SIZE];
static uint8_t uart_mainRx_buffer[UART_MAIN_FIFO_SIZE];
static uint8_t uart_mainTx_buffer[UART_MAIN_FIFO_SIZE];

/*
 * Flag set from device irq handler
 */


/******************************************************************************/
/* Example configuration                                                      */
/******************************************************************************/



static void msg_printer(int level, const char * str, va_list ap)
{
#ifdef INV_MSG_ENABLE
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * ptr = out_str;
	const char * s[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
//	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
//	if(idx >= (sizeof(out_str)))
//		return;

	while(*ptr != '\0') {
		uart_putc(LOG_UART_ID, *ptr);
		++ptr;
	}

#else
	(void)level, (void)str, (void)ap;
#endif
}

#define ASCII_ESC 27
//printf( "%c[2J", ESC );

#ifndef SELF_TEST_MAIN
#ifdef INTERRUPT_MODE
int dataready = 0;
static void ext_interrupt_data_ready_cb(void *context, int int_num)
{
    dataready = 1;
}
#endif
#endif

/*
 * main() functio
 */
/* Select communication between MCU target board and ICM406xx by setting correct SERIF_TYPE */
//#define SERIF_TYPE ICM406XX_UI_SPI3
//#define SERIF_TYPE ICM406XX_UI_SPI4
#define SERIF_TYPE ICM406XX_UI_I2C

#ifdef SELF_TEST_MAIN
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(0, "[E] %s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}
#endif

#if 1
#pragma import(__use_no_semihosting)             
//±ê×¼¿âÐèÒªµÄÖ§³Öº¯Êý                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//¶¨Òå_sys_exit()ÒÔ±ÜÃâÊ¹ÓÃ°ëÖ÷»úÄ£Ê½    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//ÖØ¶¨Òåfputcº¯Êý 
int fputc(int ch, FILE *f)
{ 	
	while((USART2->SR&0X40)==0);//Ñ­»··¢ËÍ,Ö±µ½·¢ËÍÍê±Ï   
	USART2->DR = (u8) ch;      
	return ch;
}
#endif 

int main(void)
{

	
	/*
	 * Init UART
	 * LOG: USART6 - TX(PA11) / RX(PA12) - 921600 baud ; 8bits ; no parity - TX IRQ enabled
	 * MAIN IddWrapper: USART2 (USB) - 2000000 baud ; 8bits ; no parity - IRQs enabled when using Nucleo Carrier board rev A
	 * MAIN IddWrapper: USART6 - TX(PA9) / RX(PA10) / CTS(PA11) / RTS(PA12) - 2000000 baud ; 8bits ; no parity - IRQs enabled when using Nucleo Carrier board rev B
	 */
	uart_config(LOG_UART_ID, uart_log_buffer, NULL, UART_LOG_FIFO_SIZE, 0, 921600, 1);
	uart_config(MAIN_UART_ID, uart_mainTx_buffer, uart_mainRx_buffer, UART_TX_FIFO_SIZE, UART_RX_FIFO_SIZE, 921600, 1);


	/*
	 * Init RTC timer for timestamping
	 */
	rtc_timer_init(0, 1000 /* wake-up interrupt 1ms */, RTC_TIMER_CLOCK_LSE);

	/*
	 * Setup message facility to see internal traces from IDD and FW
	 */
	INV_MSG_SETUP(INV_MSG_LEVEL_MAX, msg_printer);

	/*
	 * Welcome message
	 */

//		INV_MSG(INV_MSG_LEVEL_INFO, "#        ICM406xx example         #\r\n");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################\r\n");
	INV_MSG(INV_MSG_LEVEL_INFO, "#        ICM406xx example         #\r\n");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################################\r\n");
	 

#if SERIF_TYPE_SPI
	/*
	 * Init SPI communication: SPI1 - SCK(PA5) / MISO(PA6) / MOSI(PA7) / CS(PB6)
	 */
	//const inv_host_serif_t * pserif = idd_io_hal_get_serif_instance_spi();
        spi_master_init(SPI_NUM1, SPI_6MHZ);
	INV_MSG(INV_MSG_LEVEL_INFO, "Openning serial interface through SPI\r\n");
        
#elif SERIF_TYPE_I2C
	/* 
	 * Init I2C communication: I2C1 - SCL(PB8) / SDA(PB9)
	 */
	//const inv_host_serif_t * pserif = idd_io_hal_get_serif_instance_i2c();
        i2c_master_init();
	INV_MSG(INV_MSG_LEVEL_INFO, "Openning serial interface through I2C\r\n");


	//for I2C mode with ST-NUcleo carrier card
        extern void gpio_pins_as_input(void);
        gpio_pins_as_input();

	extern void gpio_init_ad0_low(void); 
	extern void gpio_init_spi_slave_cs_as_high_for_I2C(void);
	gpio_init_ad0_low();//for I2C slave address 0x68
//        gpio_init_ad0_high();//for I2C slave address 0x69
	gpio_init_spi_slave_cs_as_high_for_I2C();

        
#if 1	
		//YOK I3C vs. I2C control
		inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);
		inv_serif_write_1B(0x19, 0x09);
		
		inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x01);
		uint8_t reg_val = inv_serif_read_1B(0x7C);
		reg_val = reg_val & (~0x03);
		inv_serif_write_1B(0x7C, reg_val);
#endif		
        
#else
	#error "No serial interface selected.."
#endif
                
#if !defined(SELF_TEST_MAIN)  //def ->selftest demo; undef ->sensor data demo                
        uint8_t ch;

        
    ICM4x6xx_init(); 
	
    ICM4x6xx_Gyro_on();
    ICM4x6xx_Accel_on();
		
//    ICM4x6xx_accel_odr(10);   //idx=10: 25hz ODR
//    ICM4x6xx_gyro_odr(10);    //idx=10: 25hz ODR
    ICM4x6xx_accel_odr(6);   //idx=7: 200hz ODR
    ICM4x6xx_gyro_odr(6);    //idx=7: 200hz ODR
//    ICM4x6xx_accel_fsr(3);   //idx=3: 2G FSR
	ICM4x6xx_accel_fsr(0);			//idx=0:	16G FSR
    ICM4x6xx_gyro_fsr(0);    //idx=0: 2000dps FSR         
        
#ifdef INTERRUPT_MODE
        delay_ms(50);   
    ICM4x6xx_int_enable(0xC, 0x0);    //dataready and fifo_WM on INT1
    ICM4x6xx_int_mode_config(0x1B);   //INT_pin mode config: pulse/push-pull/active-high  (workaround 9226)        
        gpio_sensor_irq_init(TO_MASK(GPIO_SENSOR_IRQ_D6), ext_interrupt_data_ready_cb, 0);    
#endif
#ifdef DATA_FIFO_MODE        
        //ICM406xx_SetupFIFO();
        ICM4x6xx_SetupFIFO();
#endif          
        inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);         // Set to bank 0

        ch = inv_serif_read_1B(0x13);      // 
        INV_MSG(INV_MSG_LEVEL_INFO, ">>reg 0x13 (bank 0) = 0x%x\r\n", ch&0xff);
        ch = inv_serif_read_1B(0x75);      //
        INV_MSG(INV_MSG_LEVEL_INFO, ">>whoami/reg 0x75 (bank 0) = 0x%x\r\n", ch&0xff);
        ch = inv_serif_read_1B(0x4C);      // 
        INV_MSG(INV_MSG_LEVEL_INFO, ">>reg 0x4C (bank 0) = 0x%x\r\n\r\n", ch&0xff);        
        
        delay_ms(100);  
          
        INV_MSG(0, "raw_temp	Temp	ax    ay    az   gx	gy	gz\r\n");  
        //int timestamp;
        while(1)
        {          
            int timestamp = (int)(rtc_timer_get_time_us()/1000);
					//USART_SendData(USART2,0x11);

#ifdef INTERRUPT_MODE         
#ifdef DATA_FIFO_MODE
            if (dataready == 1) //the flag is set by ISR 
            {
              uint8_t int_status = inv_serif_read_1B(MPUREG_INT_STATUS);                 
              short gyro[3], accel[3];
              uint8_t raw_temp;
              /*ICM406xx_GetGyroAccelData_fromFIFO(gyro, accel);
              INV_MSG(INV_MSG_LEVEL_INFO, "INT_STATUS: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d\r\n",
                      int_status, timestamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] ); */
              
              ICM4x6xx_GetGyroAccelTemperatureData_fromFIFO(gyro, accel, &raw_temp);
              float temp_gain = 1.0f /115.6;
              float temp_offset = 25.0;
              float temp = (raw_temp * 128.0 * temp_gain) + temp_offset; 
              INV_MSG(0, "[FIFO] 0x%X %4.2f %d %d %d %d %d %d\r\n", 
                      raw_temp&0xff, temp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] );  
              dataready = 0; //reset the flag after done
            }
#else
            if (dataready == 1) //the flag is set by ISR 
            {
              uint8_t int_status = inv_serif_read_1B(MPUREG_INT_STATUS);                 
              short gyro[3];
              short accel[3];
              ICM4x6xx_GetGyroData_UI(gyro);
              ICM4x6xx_GetAccelData_UI(accel);
              //INV_MSG(INV_MSG_LEVEL_INFO, "INT_STATUS: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d\r\n",
              //        int_status, timestamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] ); 
              unsigned char buf[6];
              inv_serif_read(0x1d, buf, 2);
              short raw_temp = buf[0]<<8 | buf[1];
              
              float temp_gain = 1.0f /115.6f;
              float temp_offset = 25.0;
              float temp = (raw_temp * temp_gain) + temp_offset; 
              INV_MSG(0, "[REG] 0x%X %4.2f %d %d %d %d %d %d\r\n", 
                      raw_temp&0xffff, temp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] );    
              dataready = 0; //reset the flag after done
            }
#endif
#else 
#ifdef DATA_FIFO_MODE
            uint8_t fifocount_lo, fifocount_hi;
            uint16_t fifocount;
            fifocount_hi = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT1_REG);        // Read the FIFO size
            fifocount_lo = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT2_REG);
            fifocount = (fifocount_hi << 8) | fifocount_lo;      
            INV_MSG(0, "[FIFO count]: %d\r\n", fifocount);
            while (fifocount >= 16)
            {
              short gyro[3], accel[3];
              uint8_t raw_temp;
              ICM4x6xx_GetGyroAccelTemperatureData_fromFIFO(gyro, accel, &raw_temp);
              float temp_gain = 1.0f /115.6;
              float temp_offset = 25.0;
              float temp = (raw_temp * 128.0 * temp_gain) + temp_offset; 
              INV_MSG(0, "[FIFO] 0x%X %4.2f %d %d %d %d %d %d\r\n", 
                      raw_temp&0xff, temp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] );                
              fifocount -= 16;
            }
            delay_ms(5);
#else 
//polling mode for DataReady event
            uint8_t int_status = inv_serif_read_1B(MPUREG_INT_STATUS);    
            uint8_t i;
            if ((int_status & BIT_STATUS_DRDY) !=0) //sensor data ready              
            {
              short gyro[3], accel[3];
						
              ICM4x6xx_GetGyroData_UI(gyro);
              ICM4x6xx_GetAccelData_UI(accel);
							printf("%.3f, %.3f, %.3f,%d, %d,%d\n",(float)accel[0]/2048,(float)accel[1]/2048,(float)accel[2]/2048,gyro[0],gyro[1],gyro[2]);
							
							
              INV_MSG(INV_MSG_LEVEL_INFO, "INT_STATUS: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d\r\n",
                      int_status, timestamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] );  
//              INV_MSG(INV_MSG_LEVEL_INFO, "INT_STATUS: 0x%X | Time(ms): %d | ACC: %f %f %f | Gyro: %d %d %d\r\n",
//                      int_status, timestamp, accel[0]/2048, accel[1]/2048, accel[2]/2048, gyro[0], gyro[1], gyro[2] );							
            }         
//            delay_ms(1);
#endif            
#endif            
        }    
#else //for the self-test
	/*
	 * Welcome message
	 */
	INV_MSG(0, "#####################################\r\n");
	INV_MSG(0, "#   ICM4x6xx Self-Test, bus: (%s)  #\r\n", ((bus_type==0)?"SPI":"I2C"));	
	INV_MSG(0, "#####################################\r\n");
	
	//inv_serif_init();
	
	//ICM4x6xx_init(); 
		
	int rc = 0;
	
	/* Initialize Icm406xx */
	//rc = SetupInvDevice(idd_io_hal_read_reg, idd_io_hal_write_reg, SERIF_TYPE);
	rc = SetupInvDevice(NULL, NULL, SERIF_TYPE);
	check_rc(rc, "error while setting up INV device");
	INV_MSG(0, "\r\n");
	
	/* Perform Self-Test */
	RunSelfTest();
	INV_MSG(0, "\r\n");

	/* Get Low Noise / Low Power bias computed by self-tests scaled by 2^16 */
	GetBias();
	INV_MSG(0, "<<Self-test...done>>\r\n");
	/* Add a delay so the messages get fully printed out */
	delay_ms(1000);
	
	
#endif        
}


