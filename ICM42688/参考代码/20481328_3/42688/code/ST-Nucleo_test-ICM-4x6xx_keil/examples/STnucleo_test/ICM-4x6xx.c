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

#include <stdint.h>

#if 1
/* InvenSense */
#include "Invn/Devices/DeviceIcm20690.h"
#include "Invn/Devices/DeviceSmartMotion.h"
//#include "Invn/EmbUtils/Message.h"
#include "Message.h"

#include "ErrorHelper.h"

/* board driver */
#include "uart.h"
#include "delay.h"
#include "gpio.h"
#include "i2c_master.h"
#include "spi_master.h"
#include "timer.h"
#include "rtc_timer.h"
#endif //
#if 0
#include "inv/inv_uart.h"
#include "inv/inv_i2c.h"
#include "inv/inv_spi.h"
#include "delay.h"
#include "ioport.h"
#endif

/* std */
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <string.h>

#include "ICM-4x6xx.h"

#define FOUR_WIRE_SPI         // Uncomment this to run in 4-wire mode.


extern int bus_type;  //defined in main.c
extern int chip_on_evb;
uint8_t chipID;

int i2c_address;
#if 0  //--yd
uint8_t fifo_packet_format = FIFO_16_BYTE;
uint8_t fifo_packet_size = 16;
#else
uint8_t fifo_packet_format = FIFO_20_BYTE;
uint8_t fifo_packet_size = 20;
#endif
#if 0
void inv_serif_init(void)
{
	if (bus_type ==1) //I2C
	{
		if(chip_on_evb == 1) //sensor chip on EVB
			i2c_address = 0x68; 
		else//sensor chip on Atmel board
			i2c_address = 0x69; 
			
		i2c_master_initialize();	
#if 1	
		//YOK I2C vs. I2C control
		inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);
		inv_serif_write_1B(0x19, 0x09);
		
		inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x01);
		uint8_t reg_val = inv_serif_read_1B(0x7C);
		reg_val = reg_val & (~0x03);
		inv_serif_write_1B(0x7C, reg_val);
#endif			
	}else if (bus_type ==0) //SPI
	{
		if(chip_on_evb == 1)//sensor chip on EVB
		{
			arch_ioport_set_pin_mode(SPI_NPCS0_GPIO, SPI_NPCS0_FLAGS);
			arch_ioport_disable_pin(SPI_NPCS0_GPIO);	
			spi_master_init(0); //0-evb ;1-onboard 
		}
		else //sensor chip onboard
		{
			arch_ioport_set_pin_mode(SPI_NPCS1_GPIO, SPI_NPCS1_FLAGS);
			arch_ioport_disable_pin(SPI_NPCS1_GPIO);
			spi_master_init(1);
		}
	}
	
}
#endif //0--yd

uint8_t inv_serif_read_1B(uint16_t reg_addr)
{
  uint8_t tmp_reg_addr, tmp_reg_val;
  tmp_reg_addr = reg_addr % 0xFF;

  if (bus_type == 0)
    spi_master_read_register(SPI_NUM1, tmp_reg_addr, 1, &tmp_reg_val);
    //spi_master_read_register(tmp_reg_addr, 1, &tmp_reg_val);
	//void spi_master_write_register(uint8_t register_addr, uint32_t len, uint8_t * value)
  else if (bus_type == 1)
    i2c_master_read_register(ICM_I2C_ADDR, tmp_reg_addr, 1, &tmp_reg_val);
    //i2c_master_read_register(i2c_address, tmp_reg_addr, 1, &tmp_reg_val);
	
//  delay_us(50);
  return tmp_reg_val;
}
uint8_t inv_serif_read(uint16_t reg_addr, uint8_t *buf, int len)
{
  uint8_t tmp_reg_addr;
  tmp_reg_addr = reg_addr % 0xFF;

  if (bus_type == 0)  
    spi_master_read_register(SPI_NUM1, tmp_reg_addr, len, buf);
    //spi_master_read_register(tmp_reg_addr, len, buf);
  else if (bus_type == 1)
    i2c_master_read_register(ICM_I2C_ADDR, tmp_reg_addr, len, buf);
    //i2c_master_read_register(i2c_address, tmp_reg_addr, len, buf);

//  delay_us(20);
  return 0;
}
void inv_serif_write(uint16_t reg_addr, const uint8_t *buf, int len)
{
	uint8_t tmp_reg_addr;
	
	tmp_reg_addr = reg_addr % 0xFF;

	if (bus_type == 0)
          spi_master_write_register(SPI_NUM1, tmp_reg_addr, len, buf);
	//spi_master_write_register(tmp_reg_addr, len, buf); 
//unsigned long spi_master_read_register(spi_num_t spinum, unsigned char register_addr, unsigned short register_len, unsigned char *register_value);
	else if (bus_type == 1)
          i2c_master_write_register(ICM_I2C_ADDR, tmp_reg_addr, len, buf);
	//i2c_master_write_register(i2c_address, tmp_reg_addr, len, buf);

	delay_us(20);
}
void inv_serif_write_1B(uint16_t reg_addr, uint8_t reg_val)
{
  uint8_t tmp_reg_addr;
  
  tmp_reg_addr = reg_addr % 0xFF;

  if (bus_type == 0)  
      spi_master_write_register(SPI_NUM1, tmp_reg_addr, 1, &reg_val);
      //spi_master_write_register(tmp_reg_addr, 1, &reg_val);
  else if (bus_type == 1)
      i2c_master_write_register(ICM_I2C_ADDR, tmp_reg_addr, 1, &reg_val);
      //i2c_master_write_register(i2c_address, tmp_reg_addr, 1, &reg_val);

 //  delay_us(20);
}
void ICM4x6xx_Accel_on(void)
{
  uint8_t reg_val;
    // Enable Accel
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);
  reg_val = inv_serif_read_1B(MPUREG_PWR_MGMT_0_REG) | (BIT_ACCEL_MODE_LN);      // Accel on in LNM
  inv_serif_write_1B(MPUREG_PWR_MGMT_0_REG, reg_val);  
  delay_us(20);  //workaround 9136
}
void ICM4x6xx_Accel_off(void)
{
  uint8_t reg_val;
    // disable Accel
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);
  reg_val = inv_serif_read_1B(MPUREG_PWR_MGMT_0_REG) & (~BIT_ACCEL_MODE_LN);      // Accel off
  delay_us(50);  
  inv_serif_write_1B(MPUREG_PWR_MGMT_0_REG, reg_val);
  delay_us(200);  //workaround 9136  
}

int gyro_on_ts;
void ICM4x6xx_Gyro_on(void)
{
  uint8_t reg_val;

   // Gyro Workaround 9396
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x04);
  inv_serif_write_1B(0x10, 0x0d);
  inv_serif_write_1B(0x12, 0x01);

  // Enable Gyro
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);
  reg_val = inv_serif_read_1B(MPUREG_PWR_MGMT_0_REG) | (BIT_GYRO_MODE_LN);      // Gyro on in LNM
  delay_us(50);  
  inv_serif_write_1B(MPUREG_PWR_MGMT_0_REG, reg_val);  
  delay_us(200);  //workaround 9136

   // Gyro Workaround 9396
  delay_ms(10);
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x04);
  inv_serif_write_1B(0x10, 0x2a); //set 42 to reg 0x10
  delay_ms(100);
 
}
void ICM4x6xx_Gyro_off(void)
{
  uint8_t reg_val;
  //int ts;
  
 // ts = (int)(rtc_timer_get_time_us()/1000);
  //if ((ts - gyro_on_ts) < 40 ) return; //workaround 9099
  
  // disable Gyro
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);
  reg_val = inv_serif_read_1B(MPUREG_PWR_MGMT_0_REG) & ~(BIT_GYRO_MODE_LN);      // Gyro off
  inv_serif_write_1B(MPUREG_PWR_MGMT_0_REG, reg_val);  
//  delay_us(200);  //workaround 9136
  delay_ms(200);
}

void ICM4x6xx_init(void)
{
  //uint8_t reg_val;;
   
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);           // Set to bank 0
  inv_serif_write_1B(MPUREG_CHIP_CONFIG_REG, BIT_SOFT_RESET_CHIP_CONFIG);   //chip soft reset
  delay_ms(100);
  
    inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);         // Set to bank 0
    chipID = inv_serif_read_1B(0x75);      //

}

//--

void ICM4x6xx_accel_odr(int odr_idx)
{

    if (odr_idx <= 0x0F)
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
      uint8_t reg_val = (inv_serif_read_1B(MPUREG_ACCEL_CONFIG0_REG)& 0xF8) | odr_idx;
      inv_serif_write_1B(MPUREG_ACCEL_CONFIG0_REG, reg_val);    
    }
}

void ICM4x6xx_gyro_odr(int odr_idx)
{
    if (odr_idx <= 0x0F)
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);      
      uint8_t reg_val = (inv_serif_read_1B(MPUREG_GYRO_CONFIG0_REG)& 0xF8) | odr_idx;
      inv_serif_write_1B(MPUREG_GYRO_CONFIG0_REG, reg_val);    
    } 
}

int gyro_fsr_idx = 0;
int accel_fsr_idx = 0;
// Set the accel FS
void ICM4x6xx_accel_fsr(int fsr_idx)
{

    uint8_t     reg_val;
	
    accel_fsr_idx = fsr_idx;
    if (fsr_idx < 0x08)
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
      reg_val = (inv_serif_read_1B(MPUREG_ACCEL_CONFIG0_REG) & 0x1F) | (fsr_idx << 5);
      inv_serif_write_1B(MPUREG_ACCEL_CONFIG0_REG, (uint8_t)reg_val);
    }
}

// Set the gyro FS
void ICM4x6xx_gyro_fsr(int fsr_idx)
{

    uint8_t     reg_val;
    
	gyro_fsr_idx = fsr_idx;
    if (fsr_idx < 0x08)
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
      reg_val = (inv_serif_read_1B(MPUREG_GYRO_CONFIG0_REG) & 0x1F) | (fsr_idx << 5);
      inv_serif_write_1B(MPUREG_GYRO_CONFIG0_REG, (uint8_t)reg_val);
    } 
}


void ICM4x6xx_GetGyroData_UI(short * data)
{
  uint8_t buf[6];
  
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
  inv_serif_read(MPUREG_GYRO_DATA_X0_UI, buf, 6);
#if 0  //little endian
  data[0] = buf[1]<<8 | buf[0];
  data[1] = buf[3]<<8 | buf[2];
  data[2] = buf[5]<<8 | buf[4];   
#else
  data[0] = buf[0]<<8 | buf[1];
  data[1] = buf[2]<<8 | buf[3];
  data[2] = buf[4]<<8 | buf[5]; 

  
#endif
  
}
void ICM4x6xx_GetAccelData_UI(short * data)
{
  uint8_t buf[6];
  
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
  inv_serif_read(MPUREG_ACCEL_DATA_X0_UI, buf, 6);
#if 0  //little endian
  data[0] = buf[1]<<8 | buf[0];
  data[1] = buf[3]<<8 | buf[2];
  data[2] = buf[5]<<8 | buf[4];   
#else
  data[0] = buf[0]<<8 | buf[1];
  data[1] = buf[2]<<8 | buf[3];
  data[2] = buf[4]<<8 | buf[5];    
#endif
}
void ICM4x6xx_GetTemperatureData_UI(short * data)
{
	uint8_t buf[6];
	
	inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
	//inv_serif_read(MPUREG_TEMP_DATA0_UI, buf, 6);
	inv_serif_read(MPUREG_TEMP_DATA0_UI, buf, 2);
	#if 0  //little endian
	data[0] = buf[1]<<8 | buf[0];
	//data[1] = buf[3]<<8 | buf[2];
	#else
	data[0] = buf[0]<<8 | buf[1];
	//data[1] = buf[2]<<8 | buf[3];
	#endif
}
void ICM4x6xx_GetGyroAccelData_UI(short * data)
{
  uint8_t buf[12];

  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
  inv_serif_read(MPUREG_ACCEL_DATA_X0_UI, buf, 12);
#if 0  //little endian
  data[0] = buf[1]<<8 | buf[0];
  data[1] = buf[3]<<8 | buf[2];
  data[2] = buf[5]<<8 | buf[4];  
  data[3] = buf[7]<<8 | buf[6];
  data[4] = buf[9]<<8 | buf[8];
  data[5] = buf[11]<<8 | buf[10];
#else
  data[0] = buf[0]<<8 | buf[1];
  data[1] = buf[2]<<8 | buf[3];
  data[2] = buf[4]<<8 | buf[5];  
  data[3] = buf[6]<<8 | buf[7];
  data[4] = buf[8]<<8 | buf[9];
  data[5] = buf[10]<<8 | buf[11];
#endif  
}

void ICM4x6xx_int_enable(unsigned char int1_en, unsigned char int2_en)
{
  
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0); 
  unsigned char reg_val = inv_serif_read_1B(MPUREG_INT_SOURCE0_REG) | int1_en;      
  inv_serif_write_1B(MPUREG_INT_SOURCE0_REG, reg_val);
  
  reg_val = inv_serif_read_1B(MPUREG_SENSOR_SELFTEST_REG) | int2_en;      
  inv_serif_write_1B(MPUREG_SENSOR_SELFTEST_REG, reg_val);
}


// Configure the interrupt mode
void ICM4x6xx_int_mode_config(unsigned char int_mode)
{
   inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0);
   inv_serif_write_1B(MPUREG_INT_CONFIG_REG, int_mode); 
}

void ICM4x6xx_aux_ois_enable(void)
{
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 1);
      uint8_t reg_val = inv_serif_read_1B(MPUREG_SENSOR_CONFIG2_B1_REG);
      reg_val &= ~BIT_OIS_MODE_MASK;
      reg_val |= BIT_OIS_MODE_8k;
      inv_serif_write_1B(MPUREG_SENSOR_CONFIG2_B1_REG, reg_val);  
}

void ICM4x6xx_aux_spi_mode(int setup_aux, int aux_port)
{
  if (aux_port == 1)
  {
    if (setup_aux)                                      // We're setting up the interface
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 2);
      inv_serif_write_1B(MPUREG_OIS1_CONFIG1_REG, 0x03);
    }
    else                                                // We're tearing it down.
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 2);
      inv_serif_write_1B(MPUREG_OIS1_CONFIG1_REG, 0);     
    }
  }
  else if (aux_port == 2)
  {
    if (setup_aux)                                      // We're setting up the interface
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 2);
      inv_serif_write_1B(MPUREG_OIS2_CONFIG1_REG, 3);
    }
    else                                                // We're tearing it down.
    {
      inv_serif_write_1B(MPUREG_REG_BANK_SEL, 2);
      inv_serif_write_1B(MPUREG_OIS2_CONFIG1_REG, 0);     
    }
  }
}

void ICM4x6xx_SetupFIFO(void)
{  
   uint8_t reg_val;
  
  // Havana FIFO Init
  inv_serif_write_1B(MPUREG_REG_BANK_SEL, 0x00);        // Set to bank 0
  reg_val = inv_serif_read_1B(0x4C);  
  reg_val = reg_val | 0x80;
  inv_serif_write_1B(0x4C, reg_val);
  
//  inv_serif_write_1B(MPUREG_FIFO_CONFIG_REG, 0x60);
  //--yd inv_serif_write_1B(MPUREG_FIFO_CONFIG_REG, 0x40);
  inv_serif_write_1B(MPUREG_FIFO_CONFIG_REG, 0x80); //stop-on-full --yd
  
  inv_serif_write_1B(MPUREG_TMST_CONFIG_REG, 0x39);
  
  reg_val = inv_serif_read_1B(MPUREG_INT_SOURCE0_REG);      
  inv_serif_write_1B(MPUREG_INT_SOURCE0_REG, 0x0);  //workaround 9108  
  //yd inv_serif_write_1B(MPUREG_FIFO_CONFIG2_REG, 0x10);              // Config the fifo WM size
  inv_serif_write_1B(MPUREG_FIFO_CONFIG2_REG, 0x0);              // Config the fifo WM size (test--yd)
  inv_serif_write_1B(MPUREG_INT_SOURCE0_REG, reg_val);  //workaround 9108  
  
  extern int fifo_data_20bit;
  if (fifo_data_20bit == 1)
  //--yd inv_serif_write_1B(MPUREG_FIFO_CONFIG1_REG, 0x03);               // Enable the accel and gyro to the FIFO
	inv_serif_write_1B(MPUREG_FIFO_CONFIG1_REG, 0x13);               // Enable the accel and gyro to the FIFO, 20bit fifo--yd
  else
	inv_serif_write_1B(MPUREG_FIFO_CONFIG1_REG, 0x03);               // Enable the accel and gyro to the FIFO
  
  // Extra FIFO config

   inv_serif_write_1B(MPUREG_INT_CONFIG1_REG, 0x10); //for A1
}  

uint64_t timestamp = 0;
void ICM4x6xx_GetGyroAccelData_fromFIFO(short * gyr_data, short * acc_data)
{
	
  uint8_t fifocount_lo, fifocount_hi;
  uint16_t fifocount;

  uint8_t fifo_data[HAVANA_MAX_PACKET_SIZE];

  unsigned char reg_val = inv_serif_read_1B(MPUREG_INT_SOURCE0_REG);      
  inv_serif_write_1B(MPUREG_INT_SOURCE0_REG, 0x0);  //workaround 8401
  
  fifocount_hi = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT1_REG);        // Read the FIFO size
  fifocount_lo = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT2_REG);
  fifocount = (fifocount_hi << 8) | fifocount_lo;

//  for (int ii = 0; ii < fifocount; ii=ii+fifo_packet_size)                                // Keep reading from the FIFO 
  {                                                             // until we have less than one packet.
    if (fifocount >= fifo_packet_size)                         // If we have a complete packet in the FIFO
    { 
      inv_serif_read(0x30, fifo_data, fifo_packet_size);
      
      timestamp = 0;

#if 0   //FIFO data: little endian
      if (fifo_packet_format == FIFO_20_BYTE)
        timestamp = ((int16_t)fifo_data[16] << 8) | (int16_t)fifo_data[15];
      else if (fifo_packet_format == FIFO_16_BYTE)
        timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[14];
      else
        timestamp = 0;
      
      if (fifo_data[0] & FIFO_ACCEL_EN)
      {
        acc_data[0] = ((int16_t)fifo_data[2] << 8) | (int16_t)fifo_data[1];
        acc_data[1] = ((int16_t)fifo_data[4] << 8) | (int16_t)fifo_data[3];
        acc_data[2] = ((int16_t)fifo_data[6] << 8) | (int16_t)fifo_data[5];
      }
      if (fifo_data[0] & FIFO_GYRO_EN)
      {
        if (fifo_packet_format == FIFO_GYRO_ONLY)
        {
          gyr_data[0] = ((int16_t)fifo_data[2] << 8) | (int16_t)fifo_data[1];
          gyr_data[1] = ((int16_t)fifo_data[4] << 8) | (int16_t)fifo_data[3];
          gyr_data[2] = ((int16_t)fifo_data[6] << 8) | (int16_t)fifo_data[5];
        }
        else
        {
          gyr_data[0] = ((int16_t)fifo_data[8] << 8) | (int16_t)fifo_data[7];
          gyr_data[1] = ((int16_t)fifo_data[10] << 8) | (int16_t)fifo_data[9];
          gyr_data[2] = ((int16_t)fifo_data[12] << 8) | (int16_t)fifo_data[11];
        }
      }
#else  //FIFO data: big endian
      //if (fifo_packet_format == FIFO_20_BYTE)
        //timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[16];
      //else if (fifo_packet_format == FIFO_16_BYTE)
        //timestamp = ((int16_t)fifo_data[14] << 8) | (int16_t)fifo_data[15];
      //else
        //timestamp = 0;
      
      if (fifo_data[0] & FIFO_ACCEL_EN)
      {
        acc_data[0] = ((fifo_data[1] << 8) | fifo_data[2]);
        acc_data[1] = ((fifo_data[3] << 8) | fifo_data[4]);
        acc_data[2] = ((fifo_data[5] << 8) | fifo_data[6]);
      }
      if (fifo_data[0] & FIFO_GYRO_EN)
      {
        if (fifo_packet_format == FIFO_GYRO_ONLY)
        {
          gyr_data[0] = (fifo_data[1] << 8) | (int16_t)fifo_data[2];
          gyr_data[1] = (fifo_data[3] << 8) | (int16_t)fifo_data[4];
          gyr_data[2] = (fifo_data[5] << 8) | (int16_t)fifo_data[6];
        }
        else
        {
          gyr_data[0] = (fifo_data[7] << 8) | (int16_t)fifo_data[8];
          gyr_data[1] = (fifo_data[9] << 8) | (int16_t)fifo_data[10];
          gyr_data[2] = (fifo_data[11] << 8) | (int16_t)fifo_data[12];
        }
      }
      if (fifo_packet_format == FIFO_20_BYTE)
	  { 
			timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[16];
	  }
      else if (fifo_packet_format == FIFO_16_BYTE)
      timestamp = ((int16_t)fifo_data[14] << 8) | (int16_t)fifo_data[15];
      else
      timestamp = 0;	  
#endif
    }
//    else
//      break;
  }
  
  inv_serif_write_1B(MPUREG_INT_SOURCE0_REG, reg_val);  //workaround 8401
  
}

void ICM4x6xx_GetGyroAccelTemperatureData_fromFIFO(short * gyr_data, short * acc_data, uint8_t *temp_data)
{

	uint8_t fifocount_lo, fifocount_hi;
	uint16_t fifocount;
	//  int ii;
	uint8_t fifo_data[HAVANA_MAX_PACKET_SIZE];

	fifocount_hi = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT1_REG);        // Read the FIFO size
	fifocount_lo = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT2_REG);
	fifocount = (fifocount_hi << 8) | fifocount_lo;
		
	//INV_MSG(0, "fifo_count = %d\n", fifocount);
	
	{                                                             // until we have less than one packet.
		int package_cnt = fifocount / fifo_packet_size;
		if(package_cnt >= 1)
			for(int jj=0; jj<package_cnt; jj++)
				inv_serif_read(0x30, fifo_data, fifo_packet_size);
				
		//if (fifocount >= fifo_packet_size)                         // If we have a complete packet in the FIFO
		{
			timestamp = 0;

#if 0   //FIFO data: little endian
			if (fifo_packet_format == FIFO_20_BYTE)
			timestamp = ((int16_t)fifo_data[16] << 8) | (int16_t)fifo_data[15];
			else if (fifo_packet_format == FIFO_16_BYTE)
			timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[14];
			else
			timestamp = 0;
			
			if (fifo_data[0] & FIFO_ACCEL_EN)
			{
				acc_data[0] = ((int16_t)fifo_data[2] << 8) | (int16_t)fifo_data[1];
				acc_data[1] = ((int16_t)fifo_data[4] << 8) | (int16_t)fifo_data[3];
				acc_data[2] = ((int16_t)fifo_data[6] << 8) | (int16_t)fifo_data[5];
			}
			if (fifo_data[0] & FIFO_GYRO_EN)
			{
				if (fifo_packet_format == FIFO_GYRO_ONLY)
				{
					gyr_data[0] = ((int16_t)fifo_data[2] << 8) | (int16_t)fifo_data[1];
					gyr_data[1] = ((int16_t)fifo_data[4] << 8) | (int16_t)fifo_data[3];
					gyr_data[2] = ((int16_t)fifo_data[6] << 8) | (int16_t)fifo_data[5];
				}
				else
				{
					gyr_data[0] = ((int16_t)fifo_data[8] << 8) | (int16_t)fifo_data[7];
					gyr_data[1] = ((int16_t)fifo_data[10] << 8) | (int16_t)fifo_data[9];
					gyr_data[2] = ((int16_t)fifo_data[12] << 8) | (int16_t)fifo_data[11];
				}
			}
#else  //FIFO data: big endian
			if (fifo_packet_format == FIFO_20_BYTE)
			timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[16];
			else if (fifo_packet_format == FIFO_16_BYTE)
			timestamp = ((int16_t)fifo_data[14] << 8) | (int16_t)fifo_data[15];
			else
			timestamp = 0;
			
			if (fifo_data[0] & FIFO_ACCEL_EN)
			{
				acc_data[0] = (int16_t)((fifo_data[1] << 8) | fifo_data[2]);
				acc_data[1] = (int16_t)((fifo_data[3] << 8) | fifo_data[4]);
				acc_data[2] = (int16_t)((fifo_data[5] << 8) | fifo_data[6]);
			}
			if (fifo_data[0] & FIFO_GYRO_EN)
			{
				if (fifo_packet_format == FIFO_GYRO_ONLY)
				{
					gyr_data[0] = ((int16_t)fifo_data[1] << 8) | (int16_t)fifo_data[2];
					gyr_data[1] = ((int16_t)fifo_data[3] << 8) | (int16_t)fifo_data[4];
					gyr_data[2] = ((int16_t)fifo_data[5] << 8) | (int16_t)fifo_data[6];
				}
				else
				{
					gyr_data[0] = ((int16_t)fifo_data[7] << 8) | (int16_t)fifo_data[8];
					gyr_data[1] = ((int16_t)fifo_data[9] << 8) | (int16_t)fifo_data[10];
					gyr_data[2] = ((int16_t)fifo_data[11] << 8) | (int16_t)fifo_data[12];
				}
			}
#endif
#if 1      
      //handle the invalid temperature data in FIFO
      static uint8_t last_temp;
      if (fifo_data[13] != 0x80)
      {
        *temp_data = fifo_data[13]; //
        last_temp = fifo_data[13]; //save the valid value
      }
      else
      {
        *temp_data = last_temp; //0x80 is invalid code for 1-byte temperature, use the last valid one
      }
#else
      *temp_data = fifo_data[13];
#endif 

			//*temp_data = (int8_t)(fifo_data[13]);
			//INV_MSG(0, "raw temp=0x%x\r\n", fifo_data[13]&0xff);
		}

	}
	

	
}


void ICM4x6xx_GetGyroAccelTemperatureData_fromFIFO_Hires(int * gyr_data, int * acc_data, uint8_t *temp_data)
{
	//  inv_sensor_event_t event;
	//   uint64_t timestamp = 0;
	int16_t acc_data_16[3], gyr_data_16[3];
	//  struct inv_icm20690 * s = &self->basesensor_states;
	uint8_t fifocount_lo, fifocount_hi;
	uint16_t fifocount;
	//  int ii;
	uint8_t fifo_data[HAVANA_MAX_PACKET_SIZE];
	
	fifocount_hi = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT1_REG);        // Read the FIFO size
	fifocount_lo = inv_serif_read_1B(MPUREG_FIFO_BYTE_COUNT2_REG);
	fifocount = (fifocount_hi << 8) | fifocount_lo;
	
	//INV_MSG(0, "fifo_count = %d, ", fifocount);

	{                                                             // 
		int package_cnt = fifocount / fifo_packet_size;
		if(package_cnt >= 1) 
			for(int jj=0; jj<package_cnt; jj++)
				inv_serif_read(0x30, fifo_data, fifo_packet_size);
		
		//INV_MSG(0, "fifo_header = 0x%X\r\n", fifo_data[0]);
		{

			timestamp = 0;

#if 0   //FIFO data: little endian
			if (fifo_packet_format == FIFO_20_BYTE)
			timestamp = ((int16_t)fifo_data[16] << 8) | (int16_t)fifo_data[15];
			else if (fifo_packet_format == FIFO_16_BYTE)
			timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[14];
			else
			timestamp = 0;
			
			if (fifo_data[0] & FIFO_ACCEL_EN)
			{
				acc_data[0] = ((int16_t)fifo_data[2] << 8) | (int16_t)fifo_data[1];
				acc_data[1] = ((int16_t)fifo_data[4] << 8) | (int16_t)fifo_data[3];
				acc_data[2] = ((int16_t)fifo_data[6] << 8) | (int16_t)fifo_data[5];
			}
			if (fifo_data[0] & FIFO_GYRO_EN)
			{
				if (fifo_packet_format == FIFO_GYRO_ONLY)
				{
					gyr_data[0] = ((int16_t)fifo_data[2] << 8) | (int16_t)fifo_data[1];
					gyr_data[1] = ((int16_t)fifo_data[4] << 8) | (int16_t)fifo_data[3];
					gyr_data[2] = ((int16_t)fifo_data[6] << 8) | (int16_t)fifo_data[5];
				}
				else
				{
					gyr_data[0] = ((int16_t)fifo_data[8] << 8) | (int16_t)fifo_data[7];
					gyr_data[1] = ((int16_t)fifo_data[10] << 8) | (int16_t)fifo_data[9];
					gyr_data[2] = ((int16_t)fifo_data[12] << 8) | (int16_t)fifo_data[11];
				}
			}
#else  //FIFO data: big endian

			unsigned char accel_hires[3], gyro_hires[3];
			
			if (fifo_data[0] & FIFO_ACCEL_EN)
			{
				acc_data_16[0] = (int16_t)((fifo_data[1] << 8) | fifo_data[2]);
				acc_data_16[1] = (int16_t)((fifo_data[3] << 8) | fifo_data[4]);
				acc_data_16[2] = (int16_t)((fifo_data[5] << 8) | fifo_data[6]);
			}
			if (fifo_data[0] & FIFO_GYRO_EN)
			{
				if (fifo_packet_format == FIFO_GYRO_ONLY)
				{
					gyr_data_16[0] = ((int16_t)fifo_data[1] << 8) | (int16_t)fifo_data[2];
					gyr_data_16[1] = ((int16_t)fifo_data[3] << 8) | (int16_t)fifo_data[4];
					gyr_data_16[2] = ((int16_t)fifo_data[5] << 8) | (int16_t)fifo_data[6];
				}
				else
				{
					gyr_data_16[0] = ((int16_t)fifo_data[7] << 8) | (int16_t)fifo_data[8];
					gyr_data_16[1] = ((int16_t)fifo_data[9] << 8) | (int16_t)fifo_data[10];
					gyr_data_16[2] = ((int16_t)fifo_data[11] << 8) | (int16_t)fifo_data[12];
				}
			}
			//hires data for 20-bit
			for(int ii=0; ii<3;ii++)
			{
				accel_hires[ii] = (fifo_data[17+ii] &0xFF00) >> 4;
				gyro_hires[ii]  = (fifo_data[17+ii] &0x00FF);
			}
#endif

			if (fifo_packet_format == FIFO_20_BYTE)
			{
				for(int jj=0; jj<3; jj++) //hires--yd
				{
					if(gyro_fsr_idx <= 4)
						gyr_data[jj] = (gyr_data_16[jj] <<4 | gyro_hires[jj])/(1<<(4-gyro_fsr_idx));
					else
						gyr_data[jj] = (gyr_data_16[jj] <<4 | gyro_hires[jj])*(1<<(gyro_fsr_idx-4));
						
					acc_data[jj] = (acc_data_16[jj] <<4 | accel_hires[jj])/(1<<(4-accel_fsr_idx));

				}				
				timestamp = ((int16_t)fifo_data[15] << 8) | (int16_t)fifo_data[16];
			}
			else if (fifo_packet_format == FIFO_16_BYTE)
				timestamp = ((int16_t)fifo_data[14] << 8) | (int16_t)fifo_data[15];
			else
				timestamp = 0;
			
#if 1      
			//handle the invalid temperature data in FIFO
			static uint8_t last_temp;
			if (fifo_data[13] != 0x80)
			{
				*temp_data = fifo_data[13]; //
				last_temp = fifo_data[13]; //save the valid value
			}
			else
			{
				*temp_data = last_temp; //0x80 is invalid code for 1-byte temperature, use the last valid one
			}
#else
			*temp_data = fifo_data[13];
#endif 

			//*temp_data = (int8_t)(fifo_data[13]);
			//INV_MSG(0, "raw temp=0x%x\r\n", fifo_data[13]&0xff);
		}

	}
	
}


