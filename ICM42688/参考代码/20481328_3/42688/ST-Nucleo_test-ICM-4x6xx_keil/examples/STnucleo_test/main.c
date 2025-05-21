/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * This is a bare minimum user application template.
 *
 * For documentation of the board, go \ref group_common_boards "here" for a link
 * to the board-specific documentation.
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# Minimal main function that starts with a call to board_init()
 * -# Basic usage of on-board LED and button
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */

//#include <asf.h>
//#include "conf_board.h"
//#include "FreeRTOS.h"
//#include "rtc.h"
#if 0
#include "inv/inv_uart.h"
#include "inv/inv_i2c.h"
#include "inv/inv_spi.h"
#include "inv/inv_pio.h"
#endif
   
#include "ICM-4x6xx.h"

#include "example-selftest.h"
#include "ErrorHelper.h"

#define SELF_TEST_MAIN    //comment this line for sensor data example; otherwise for sensor self-test

#define SERIF_TYPE_SPI 1
#define SERIF_TYPE_I2C 0

#define CHIP_ON_EVB				1
#define CHIP_ON_ATMEL_BOARD		0

#define INTERRUPT_MODE			//comment this line for polling-mode to get sensor data
#define DATA_FIFO_MODE			//comment this line for FIFO mode to get sensor data
//#define ICM42688_HIRES_MODE		//20-bit hires FIFO mode of icm42688 - comment this line for regular 16bit FIFO mode

#if SERIF_TYPE_SPI
	int bus_type = 0;  //SPI
#elif SERIF_TYPE_I2C
	int bus_type = 1;  //I2C
#else
	int bus_type = -1; //error
#endif

#ifdef ICM42688_HIRES_MODE
	int fifo_data_20bit = 1;
#else
	int fifo_data_20bit = 0;
#endif

#if CHIP_ON_EVB
	int chip_on_evb = 1;  //sensor chip on EVB
#elif CHIP_ON_ATMEL_BOARD
	int chip_on_evb = 0; //sensor chip on Atmel board
#else
	int chip_on_evb = -1; //error 
#endif

#ifdef INTERRUPT_MODE
int dataready = 0;
static void ext_interrupt_data_ready_cb(void)
{
	dataready = 1;
}
#endif
/*
 *  Helper function to check RC value and block program execution
 */
static void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(0, "[E] %s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}

/* Select communication between MCU target board and ICM406xx by setting correct SERIF_TYPE */
//#define SERIF_TYPE ICM406XX_UI_SPI3
#define SERIF_TYPE ICM406XX_UI_SPI4
//#define SERIF_TYPE ICM406XX_UI_I2C

extern uint8_t chipID;

int main(void)
{

	sysclk_init();
	board_init();
	configure_console();
	
#if !defined(SELF_TEST_MAIN)  //def ->selftest demo; undef ->sensor data demo
	/*
	 * Welcome message
	 */
	INV_MSG(0, "#####################################\r\n");
	INV_MSG(0, "#       ICM4x6xx example  (%s)   #\r\n", ((bus_type==0)?"SPI":"I2C"));	
	INV_MSG(0, "#####################################\r\n");
	
	inv_serif_init();
	
    ICM4x6xx_init(); 
	
    ICM4x6xx_Gyro_on();
    ICM4x6xx_Accel_on();
		
    ICM4x6xx_accel_odr(10);   //idx=10: 25hz ODR
    ICM4x6xx_gyro_odr(10);    //idx=10: 25hz ODR
    ICM4x6xx_accel_fsr(3);   //idx=3: 2G FSR
    ICM4x6xx_gyro_fsr(0);    //idx=0: 2000dps FSR 

#ifdef INTERRUPT_MODE
    delay_ms(50);   
    ICM4x6xx_int_enable(0xC, 0x0);    //dataready and fifo_WM on INT1
    ICM4x6xx_int_mode_config(0x1B);   //INT_pin mode config: pulse/push-pull/active-high  (workaround 9226)
    ICM4x6xx_SetupFIFO();
	ext_int_initialize_evb(ext_interrupt_data_ready_cb); //data ready interrupt handler via GPIO
#endif 
	uint8_t ch;
	
    inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);         // Set to bank 0


	//extern uint8_t chipID;
	INV_MSG(0, ">>whoami/reg 0x75 (bank 0) = 0x%x\r\n", chipID&0xff);


    ch = inv_serif_read_1B(0x4C);      // 
	INV_MSG(0, ">>reg 0x4C (bank 0) = 0x%x\r\n\r\n", ch&0xff);        
        
    delay_ms(500);  
          
	while(1)
    {          
#ifdef INTERRUPT_MODE         
#ifdef DATA_FIFO_MODE
        if (dataready == 1) //the flag is set by ISR 
        {
			int timestamp = rtc_get_time_in_ms(RTC);
			uint8_t int_status = inv_serif_read_1B(MPUREG_INT_STATUS);                 
			short gyro[3], accel[3];
			int gyro32[3], accel32[3];
			static int8_t temperature; //static type for fixing gcc bug?
			//ICM4x6xx_GetGyroAccelData_fromFIFO(gyro, accel);
			if (fifo_data_20bit == 1)
				ICM4x6xx_GetGyroAcceTemplData_fromFIFO_Hires(gyro32, accel32, &temperature);
			else
				ICM4x6xx_GetGyroAcceTemplData_fromFIFO(gyro, accel, &temperature);
				
			float temp = (temperature*128.0/127.65)*0.5 + 25.0;
			short temperature1;
			ICM4x6xx_GetTemperatureData_UI(&temperature1);
			float temp1 = (temperature1/127.65) + 25.0;

			if (fifo_data_20bit == 1)
				INV_MSG(0, "[20bitFIFO] INT: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d | raw_Temp: 0x%x Temp: %f  raw_Temp1: 0x%x Temp1: %f\r\n",
					int_status, timestamp, accel32[0], accel32[1], accel32[2], gyro32[0], gyro32[1], gyro32[2], (uint8_t)temperature, temp, temperature1&0xffff, temp1); 												
			else
				INV_MSG(0, "[FIFO] INT: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d | raw_Temp: 0x%x Temp: %f  raw_Temp1: 0x%x Temp1: %f\r\n",
				int_status, timestamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], (uint8_t)temperature, temp, temperature1&0xffff, temp1);
			
			/*short gyro_r[3], accel_r[3];
			ICM4x6xx_GetGyroData_UI(gyro_r);
			ICM4x6xx_GetAccelData_UI(accel_r);
			INV_MSG(0, "[REG] INT: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d\r\n",
					int_status, timestamp, accel_r[0], accel_r[1], accel_r[2], gyro_r[0], gyro_r[1], gyro_r[2] );  */        
					     
			dataready = 0; //reset the flag after done
        }

#else
        if (dataready == 1) //the flag is set by ISR 
        {
			int timestamp = rtc_get_time_in_ms(RTC);
			uint8_t int_status = inv_serif_read_1B(MPUREG_INT_STATUS);                 
			short gyro[3], accel[3];
			ICM4x6xx_GetGyroData_UI(gyro);
			ICM4x6xx_GetAccelData_UI(accel);
			INV_MSG(0, "INT: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d\r\n",
					int_status, timestamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2] ); 
              
			dataready = 0; //reset the flag after done
        }

#endif
#else 
//polling mode for DataReady event
        uint8_t int_status;
		int_status = inv_serif_read_1B(MPUREG_INT_STATUS); 
            
        if ((int_status & BIT_STATUS_DRDY) !=0) //sensor data ready              
        {
			int timestamp = rtc_get_time_in_ms(RTC);				
			short gyro[3], accel[3], temperature;
			float temp;
			
			temp = (temperature/127.65) + 25.0; 
			
			ICM4x6xx_GetGyroData_UI(gyro);
			ICM4x6xx_GetAccelData_UI(accel);
			ICM4x6xx_GetTemperatureData_UI(&temperature);
			INV_MSG(0, "INT: 0x%X | Time(ms): %d | ACC: %d %d %d | Gyro: %d %d %d | raw_Temp: 0x%x Temp: %4.2f\r\n",
					int_status, timestamp, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], temperature&0xffff, temp);                      
        }   
#endif            
    }    

#else //for the self-test
	/*
	 * Welcome message
	 */
	INV_MSG(0, "#####################################\r\n");
	INV_MSG(0, "#   ICM4x6xx Self-Test, bus: (%s)  #\r\n", ((bus_type==0)?"SPI":"I2C"));	
	INV_MSG(0, "#####################################\r\n");
	
	inv_serif_init();
	
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
