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

#ifndef ICM4x6xx_H_
#define ICM4x6xx_H_

void ICM4x6xx_init(void);
void ICM4x6xx_Accel_on(void);
void ICM4x6xx_Accel_off(void);  
void ICM4x6xx_Gyro_on(void);
void ICM4x6xx_Gyro_off(void);
void ICM4x6xx_accel_odr(int odr_idx);
void ICM4x6xx_gyro_odr(int odr_idx);
void ICM4x6xx_accel_fsr(int fsr_idx);
void ICM4x6xx_gyro_fsr(int fsr_idx);
void ICM4x6xx_GetGyroData_UI(short * data);
void ICM4x6xx_GetAccelData_UI(short * data);
void ICM4x6xx_GetGyroAccelData_UI(short * data);
void ICM4x6xx_GetTemperatureData_UI(short * data);
void ICM4x6xx_int_mode(int set_to_latched);
void ICM4x6xx_int_polarity(int set_to_low);
void ICM4x6xx_aux_spi_mode(int setup_aux, int aux_port);
void ICM4x6xx_aux_ois_enable(void);
void ICM4x6xx_SetupFIFO(void);

void inv_serif_init(void);
uint8_t inv_serif_read_1B(uint16_t reg_addr);
//--yd uint8_t inv_serif_read(uint16_t , uint8_t *, int );
uint8_t inv_serif_read(uint16_t reg_addr, uint8_t *buf, int len);
void inv_serif_write(uint16_t reg_addr, const uint8_t *buf, int len);
void inv_serif_write_1B(uint16_t reg_addr, uint8_t reg_val);
void ICM4x6xx_int_enable(unsigned char int1_en, unsigned char int2_en);
void ICM4x6xx_int_mode_config(unsigned char int_mode);
void ICM4x6xx_GetGyroAccelData_fromFIFO(short * gyr_data, short * acc_data);
void ICM4x6xx_GetGyroAccelTemperatureData_fromFIFO(short * gyr_data, short * acc_data, uint8_t *temp_data);
void ICM4x6xx_GetGyroAccelTemperatureData_fromFIFO_Hires(int * gyr_data, int * acc_data, uint8_t *temp_data);

////////////
/** BANK0 */
#define MPUREG_CHIP_CONFIG_REG 0x11
	#define BIT_SPI_MODE 0x10
	#define BIT_SOFT_RESET_CHIP_CONFIG 0x01
	
#define MPUREG_DRIVE_CONFIG_REG 0x13
	#define BIT_PADS_SLEW_TRIM_D2A 0x07
	
#define MPUREG_INT_CONFIG_REG 0x14
	#define BIT_INT2_MODE 0x20
	#define BIT_INT2_DRIVE_CIRCUIT 0x10
	#define BIT_INT2_POLARITY 0x08
	#define BIT_INT1_MODE 0x04
	#define BIT_INT1_DRIVE_CIRCUIT 0x02
	#define BIT_INT1_POLARITY 0x01
	
#define MPUREG_FIFO_CONFIG_REG 0x16
	#define BIT_FIFO_MODE_CTRL_MASK ((0x03)<<6)
	#define BIT_FIFO_MODE_CTRL_BYPASS ((0x00)<<6)
	#define BIT_FIFO_MODE_CTRL_STREAM ((0x01)<<6)
	#define BIT_FIFO_MODE_CTRL_SNAPSHOT ((0x02)<<6)

#define MPUREG_TEMP_DATA0_UI 0x1D

#define MPUREG_ACCEL_DATA_X0_UI 0x1F

#define MPUREG_GYRO_DATA_X0_UI 0x25

#define MPUREG_TMST_FSYNC1 0x2B
	
#define MPUREG_INT_STATUS 0x2D
	#define BIT_STATUS_UI_FSYNC 0x40
	#define BIT_STATUS_PLL_RDY 0x20
	#define BIT_STATUS_RESET_DONE 0x10
	#define BIT_STATUS_DRDY 0x08
	#define BIT_STATUS_FIFO_THS 0x04
	#define BIT_STATUS_FIFO_FULL 0x02
	#define BIT_STATUS_AGC_RDY 0x01

#define MPUREG_FIFO_BYTE_COUNT1_REG 0x2E
#define MPUREG_FIFO_BYTE_COUNT2_REG 0x2F
#define MPUREG_FIFO_DATA_REG 0x30
	
#define MPUREG_SIGNAL_PATH_RESET_REG 0x4B
	#define BIT_ABORT_AND_RESET 0x08
	#define BIT_TMST_STROBE 0x04
	#define BIT_FIFO_FLUSH 0x02
	#define BIT_TEMP_RST 0x01

#define MPUREG_INTF_CONFIG0_REG 0x4C	
	#define BIT_FIFO_SREG_INVALID_IND_DIS 0x80
	#define BIT_FIFO_COUNT_REC 0x40
	#define BIT_FIFO_COUNT_ENDIAN 0x20
	#define BIT_SENSOR_DATA_ENDIAN 0x10
	#define BIT_SPI_MODE_OIS2 0x08
	#define BIT_SPI_MODE_OIS1 0x04

#define MPUREG_INTF_CONFIG1_REG 0x4D	
	#define BITS_GYRO_AFSR_MODE_MASK (0xC0)
	#define BITS_ACCEL_AFSR_MODE_MASK (0x30)
	#define BITS_ACCEL_LP_CLK_SEL 0x08
	#define BITS_RTC_MODE 0x04
	#define BITS_CLKSEL_MASK (0x03)
	
#define MPUREG_PWR_MGMT_0_REG 0x4E
	#define BIT_TEMP_DIS 0x20
	#define BIT_IDLE 0x10
	#define BIT_GYRO_MODE_MASK ((0x03)<<2)
		#define BIT_GYRO_MODE_OFF ((0x00)<<2)
		#define BIT_GYRO_MODE_STANDBY ((0x01)<<2)
		#define BIT_GYRO_MODE_LP ((0x02)<<2)
		#define BIT_GYRO_MODE_LN ((0x03)<<2)
	#define BIT_ACCEL_MODE_MASK ((0x03)<<0)
		#define BIT_ACCEL_MODE_OFF 0x00
		#define BIT_ACCEL_MODE_LP 0x02
		#define BIT_ACCEL_MODE_LN 0x03
	
#define SET_LPM 0
#define SET_LNM 1

#define MPUREG_GYRO_CONFIG0_REG 0x4F
	#define BIT_GYRO_UI_FS_SEL_SHIFT 5
	#define BIT_GYRO_UI_FS_SEL_MASK ((0x07)<<BIT_GYRO_UI_FS_SEL_SHIFT)
	#define BIT_GYRO_ODR_NONFLAME_MASK ((0x0F)<<0)
	
#define MPUREG_ACCEL_CONFIG0_REG 0x50
	#define BIT_ACCEL_UI_FS_SEL_SHIFT 5
	#define BIT_ACCEL_UI_FS_SEL_MASK ((0x07)<<BIT_ACCEL_UI_FS_SEL_SHIFT)
	#define BIT_ACCEL_ODR_NONFLAME_MASK ((0x0F)<<0)
	
#define MPUREG_GYRO_CONFIG1_REG 0x51
	#define BIT_TEMP_FILT_BW_SHIFT 5
	#define BIT_TEMP_FILT_BW_MASK ((0x07)<<BIT_TEMP_FILT_BW_SHIFT)
	#define BIT_GYRO_AVG_FILT_RATE 0x10
	#define BIT_GYRO_UI_FILT_ORD_IND_SHIFT 2
	#define BIT_GYRO_UI_FILT_ORD_IND_MASK ((0x03)<<BIT_GYRO_UI_FILT_ORD_IND_SHIFT)
	#define BIT_GYRO_DEC2_M2_ORD_MASK (0x03)
	
#define MPUREG_ACCEL_GYRO_CONFIG0_REG 0x52
	#define BIT_ACCEL_UI_FILT_BW_IND_SHIFT 4
	#define BIT_ACCEL_UI_FILT_BW_IND_MASK ((0x0F)<<BIT_ACCEL_UI_FILT_BW_IND_SHIFT)
	#define BIT_GYRO_UI_FILT_BW_IND_MASK (0x0F)
	
#define MPUREG_ACCEL_CONFIG1_REG 0x53
	#define BIT_ACCEL_AVG_FILT_RATE 0x01
	#define BIT_ACCEL_UI_FILT_ORD_IND_SHIFT 3
	#define BIT_ACCEL_UI_FILT_ORD_IND_MASK ((0x03)<<BIT_ACCEL_UI_FILT_ORD_IND_SHIFT)
	#define BIT_ACCEL_DEC2_M2_ORD_MASK ((0x03)<<1)
	
#define MPUREG_ACCEL_WOM_X_THR_REG 0x54
#define MPUREG_ACCEL_WOM_Y_THR_REG 0x55
#define MPUREG_ACCEL_WOM_Z_THR_REG 0x56

#define MPUREG_SMD_CONFIG_REG 0x57
	#define BIT_WOM_INT_MODE_AND  ((0x01)<<3)
	#define BIT_WOM_MODE_CMP_PREV ((0x01)<<2)
	#define BIT_SMD_MODE_SMD_LONG  0x03
	#define BIT_SMD_MODE_SMD_SHORT 0x02
	#define BIT_SMD_MODE_WOM       0x01
	#define BIT_SMD_MODE_DISABLE   0x00

#define MPUREG_INT_RAW_REG 0x58

#define MPUREG_INT_STATUS2_REG 0x59
    #define BIT_SMD_INT        0x08
    #define BIT_WOM_Z_INT      0x04
    #define BIT_WOM_Y_INT      0x02
    #define BIT_WOM_X_INT      0x01

#define MPUREG_TMST_CONFIG_REG 0x5A
	#define BIT_FIFO_RAM_ISO_ENA 0x40
	#define BIT_EN_DREG_FIFO_D2A 0x20
	#define BIT_TMST_TO_REGS_EN 0x10
	#define BIT_TMST_RESOL 0x08
	#define BIT_TMST_DELTA_EN 0x04
	#define BIT_TMST_FSYNC_EN 0x02
	#define BIT_TMST_EN 0x01

#define MPUREG_FIFO_CONFIG1_REG 0x5F
	#define BIT_FIFO_RESUME_PARTIAL_RD 0x40
	#define BIT_FIFO_WM_GT_TH 0x20
	#define BIT_FIFO_HIRES_EN 0x10
	#define BIT_FIFO_TMST_FSYNC_EN 0x08
	#define BIT_FIFO_TEMP_EN 0x04
	#define BIT_FIFO_GYRO_EN 0x02
	#define BIT_FIFO_ACCEL_EN 0x01
	
#define MPUREG_FIFO_CONFIG2_REG 0x60				

#define MPUREG_FSYNC_CONFIG_REG 0x62				
	#define BIT_FSYNC_UI_SEL_MASK ((0x07)<<4)
		#define BIT_FSYNC_UI_SEL_TAG_TEMP ((0x01)<<4)
	#define BIT_FSYNC_UI_FLAG_CLEAR_SEL 0x02

#define MPUREG_INT_CONFIG0_REG 0x63
#define MPUREG_INT_CONFIG1_REG 0x64
	#define BIT_INT_ASY_RST_DISABLE 0x10

#define MPUREG_INT_SOURCE0_REG 0x65	
	#define BIT_INT_UI_FSYNC_INT1_EN 0x40
	#define BIT_INT_PLL_RDY_INT1_EN 0x20
	#define BIT_INT_RESET_DONE_INT1_EN 0x10
	#define BIT_INT_UI_DRDY_INT1_EN 0x08
	#define BIT_INT_FIFO_THS_INT1_EN 0x04
	#define BIT_INT_FIFO_FULL_INT1_EN 0x02
	#define BIT_INT_UI_AGC_RDY_INT1_EN 0x01

#define MPUREG_INT_SOURCE1_REG 0x66
	#define BIT_INT_SMD_INT1_EN   0x08
	#define BIT_INT_WOM_Z_INT1_EN 0x04
	#define BIT_INT_WOM_Y_INT1_EN 0x02
	#define BIT_INT_WOM_X_INT1_EN 0x01

#define MPUREG_INT_SOURCE2_REG 0x67
#define MPUREG_INT_SOURCE3_REG 0x68
	#define BIT_INT_UI_FSYNC_INT2_EN 0x40
	#define BIT_INT_PLL_RDY_INT2_EN 0x20
	#define BIT_INT_RESET_DONE_INT2_EN 0x10
	#define BIT_INT_UI_DRDY_INT2_EN 0x08
	#define BIT_INT_FIFO_THS_INT2_EN 0x04
	#define BIT_INT_FIFO_FULL_INT2_EN 0x02
	#define BIT_INT_UI_AGC_RDY_INT2_EN 0x01
#define MPUREG_INT_SOURCE4_REG 0x69
#define MPUREG_INT_SOURCE5_REG 0x6A

#define MPUREG_SENSOR_SELFTEST_REG 0x6B
	#define BIT_ACCEL_ST_RESULT 0x08
	#define BIT_GYRO_ST_RESULT 0x04
	#define BIT_ACCEL_ST_STATUS 0x02
	#define BIT_GYRO_ST_STATUS 0x01

#define MPUREG_FIFO_LOST_PKT0_REG 0x6C

#define MPUREG_AFSR_CONFIG0_REG 0x6E
#define MPUREG_AFSR_CONFIG1_REG 0x6F

#define MPUREG_SELF_TEST_CONFIG_REG 0x70
	#define BIT_ST_REGULATOR_EN 0x40
	#define BIT_ACCEL_Z_ST_EN 0x20
	#define BIT_ACCEL_Y_ST_EN 0x10
	#define BIT_ACCEL_X_ST_EN 0x08
	#define BIT_GYRO_Z_ST_EN 0x04
	#define BIT_GYRO_Y_ST_EN 0x02
	#define BIT_GYRO_X_ST_EN 0x01

#define MPUREG_SCAN0_REG 0x71

#define MPUREG_MEM_BANK_SEL 0x72
#define MPUREG_MEM_START_ADDR 0x73
#define MPUREG_FIFO_R_W 0x74

#define MPUREG_WHO_AM_I 0x75

#define MPUREG_REG_BANK_SEL 0x76

#define MPUREG_GOS_USER_0_REG 0x77

/** BANK1 */
#define MPUREG_SENSOR_CONFIG1_B1_REG 0x04
	#define BIT_PAD_SCENARIO_MASK ((0x0F)<<4)
		#define BIT_PAD_SCENARIO_4 ((0x04)<<4)
		#define BIT_PAD_SCENARIO_10 ((0x0A)<<4)
#define MPUREG_SENSOR_CONFIG2_B1_REG 0x05
	#define BIT_OIS_MODE_MASK ((0x03)<<4)
		#define BIT_OIS_MODE_OFF ((0x00)<<4)
		#define BIT_OIS_MODE_8k ((0x01)<<4)
		#define BIT_OIS_MODE_32k ((0x02)<<4)
		#define BIT_OIS_MODE_64k ((0x03)<<4)
	#define BIT_GYRO_4000DPS_FS_MASK ((0x01)<<1)
	#define BIT_ACCEL_32G_FS_MASK ((0x01)<<0)
#define MPUREG_GYRO_CONFIG_STATIC0_B1_REG 0x09
#define MPUREG_GYRO_CONFIG_STATIC1_B1_REG 0x0A
#define MPUREG_INTF_CONFIG4_B1_REG 0x7A

/** BANK2 */
#define MPUREG_ACCEL_CONFIG_STATIC0_B2_REG 0x39
#define MPUREG_ACCEL_CONFIG_STATIC1_B2_REG 0x3A
#define MPUREG_OIS1_CONFIG1_REG 0x44
	#define BIT_OIS1_MASK   (0x07<<2)
		#define BIT_OIS1_DEC_1  (0x00<<2)
		#define BIT_OIS1_DEC_2  (0x01<<2)
		#define BIT_OIS1_DEC_4  (0x02<<2)
		#define BIT_OIS1_DEC_8  (0x03<<2)
		#define BIT_OIS1_DEC_16 (0x04<<2)
		#define BIT_OIS1_DEC_32 (0x05<<2)
	#define BIT_GYRO_OIS1_EN 0x02
	#define BIT_FSYNC_OIS_SEL_TAG_FSYNC_GYRO_XOUT ((0x02)<<5)
#define MPUREG_OIS1_CONFIG2_REG 0x45
	#define BIT_GYRO_OIS_FS_SEL_MASK ((0x03)<<3)
		#define BIT_GYRO_OIS_FS_SEL_2000DPS ((0x00)<<3)
		#define BIT_GYRO_OIS_FS_SEL_1000DPS ((0x01)<<3)
		#define BIT_GYRO_OIS_FS_SEL_500DPS  ((0x02)<<3)
		#define BIT_GYRO_OIS_FS_SEL_250DPS  ((0x03)<<3)
		#define BIT_GYRO_OIS_FS_SEL_125DPS  ((0x04)<<3)
		#define BIT_GYRO_OIS_FS_SEL_62_5DPS ((0x05)<<3)
		#define BIT_GYRO_OIS_FS_SEL_31_25DPS ((0x06)<<3)
		#define BIT_GYRO_OIS_FS_SEL_15_6DPS ((0x07)<<3)
#define MPUREG_GYRO_DATA_X0_OIS1_B2_REG 0x4F
/* Only accessible from AUX1 */
#define MPUREG_INT_STATUS_OIS1_B2_REG 0x57
	#define BIT_STATUS_OIS_DRDY 0x02
/* End of Only accessible from AUX1 */
#define MPUREG_OIS2_CONFIG1_REG 0x59
	#define BIT_GYRO_OIS2_EN 0x02
#define MPUREG_GYRO_DATA_X0_OIS2_B2_REG 0x64

/** BANK3 */
#define MPUREG_AMP_GX_TRIM1_B3_REG 0x31
#define MPUREG_AMP_GX_TRIM2_B3_REG 0x32
#define MPUREG_AMP_GY_TRIM1_B3_REG 0x36
#define MPUREG_AMP_GY_TRIM2_B3_REG 0x37
#define MPUREG_AMP_GZ_TRIM1_B3_REG 0x3B
#define MPUREG_AMP_GZ_TRIM2_B3_REG 0x3C
#define MPUREG_ACCEL_XY_TRIM4_B3_REG 0x47
#define MPUREG_ACCEL_X_TRIM3_B3_REG 0x4B
#define MPUREG_ACCEL_Y_TRIM3_B3_REG 0x4F
#define MPUREG_ACCEL_Z_TRIM1_B3_REG 0x51
#define MPUREG_ACCEL_Z_TRIM5_B3_REG 0x55

/** BANK4 */
#define MUPREG_DRV_GYR_CFG0_REG 0x10
	#define GYRO_DRV_TEST_FSMFORCE_D2A_LINEAR_START_MODE         0x0D
	#define GYRO_DRV_TEST_FSMFORCE_D2A_STEADY_STATE_AGC_REG_MODE 0x2A
#define MUPREG_DRV_GYR_CFG1_REG 0x11
#define MUPREG_DRV_GYR_CFG2_REG 0x12
	#define GYRO_DRV_SPARE2_D2A_EN 0x1

/** FIFO CONTENT DEFINITION */
#define HEADER_SIZE        1
#define ACCEL_DATA_SIZE    6
#define GYRO_DATA_SIZE     6
#define TEMP_DATA_SIZE     1
#define TS_FSYNC_SIZE      2

enum ICM4x6xx_fio_format {
  FIFO_20_BYTE,
  FIFO_ACCEL_ONLY,
  FIFO_GYRO_ONLY,
  FIFO_16_BYTE,
};
#define FIFO_ACCEL_EN           0x40
#define FIFO_GYRO_EN            0x20
#define FIFO_TS_MASK            0x0C
#define FIFO_FSYNC_BITS         0x0C
#define HAVANA_MAX_PACKET_SIZE      20

#endif /* ICM4x6xx_H_ */
