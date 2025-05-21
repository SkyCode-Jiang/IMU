/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define u8 uint8_t 
#define u16 uint16_t
#define u32 uint32_t


#define sensor_42688
#if defined sensor_42688

#define ADDRESS       0XD0
#define WHOAMI        0X75
#define PWR_MGMT0     0x4E
#define GYRO_CONFIG0  0x4F
#define ACCEL_CONFIG0 0x50
#define SENSOR_CONFIG0 0x03
#define ACCEL_DATA_X1 0x1F
#define ACCEL_DATA_X0 0x20
#define ACCEL_DATA_Y1 0x21
#define ACCEL_DATA_Y0 0x22
#define ACCEL_DATA_Z1 0x23
#define ACCEL_DATA_Z0 0x24
#define GYRO_DATA_X1  0x25
#define GYRO_DATA_X0  0x26
#define GYRO_DATA_Y1  0x27
#define GYRO_DATA_Y0  0x28
#define GYRO_DATA_Z1  0x29
#define GYRO_DATA_Z0  0x2A
#define REG_BANK_SEL  0x76
#endif


#if defined st

#define ACC_GYRO_ADDRESS     0xD6

/* sensor output data */
#define LSM6DSL_OUTX_L_G  			0X22
#define LSM6DSL_OUTX_H_G  			0X23
#define LSM6DSL_OUTY_L_G  			0X24
#define LSM6DSL_OUTY_H_G  			0X25
#define LSM6DSL_OUTZ_L_G  			0X26
#define LSM6DSL_OUTZ_H_G  			0X27

#define LSM6DSL_OUTX_L_XL  			0X28
#define LSM6DSL_OUTX_H_XL  			0X29
#define LSM6DSL_OUTY_L_XL  			0X2A
#define LSM6DSL_OUTY_H_XL  			0X2B
#define LSM6DSL_OUTZ_L_XL  			0X2C
#define LSM6DSL_OUTZ_H_XL  			0X2D
/* sensor control reg */
#define LSM6DSL_CTRL1_XL  		  0X10

#define LSM6DSL_CTRL2_G  			  0X11
#define LSM6DSL_CTRL3_C  			  0X12
#define LSM6DSL_CTRL4_C  			  0X13
#define LSM6DSL_CTRL5_C  			  0X14
#define LSM6DSL_CTRL6_G  			  0X15
#define LSM6DSL_CTRL7_G  			  0X16
#define LSM6DSL_CTRL8_XL  			0X17
#define LSM6DSL_CTRL9_XL  			0X18
#define LSM6DSL_CTRL10_C  			0X19
#define LSM6DSL_INT1_CTRL  			0X0D
#define LSM6DSL_INT2_CTRL  			0X0E
#define LSM6DSL_WHO_AM_I        0x0F    //get id


#define LSM6DSL_SENSORHUB1_REG  		0X2E
#define LSM6DSL_SENSORHUB2_REG  		0X2F
#define LSM6DSL_SENSORHUB3_REG  		0X30
#define LSM6DSL_SENSORHUB4_REG  		0X31
#define LSM6DSL_SENSORHUB5_REG  		0X32
#define LSM6DSL_SENSORHUB6_REG  		0X33
#define LSM6DSL_SENSORHUB7_REG  		0X34
#define LSM6DSL_SENSORHUB8_REG  		0X35
#define LSM6DSL_SENSORHUB9_REG  		0X36
#define LSM6DSL_SENSORHUB10_REG  		0X37
#define LSM6DSL_SENSORHUB11_REG  		0X38
#define LSM6DSL_SENSORHUB12_REG  		0X39
#define LSM6DSL_FIFO_STATUS1  				0X3A
#define LSM6DSL_FIFO_STATUS2  				0X3B
#define LSM6DSL_FIFO_STATUS3  				0X3C
#define LSM6DSL_FIFO_STATUS4  				0X3D
#define LSM6DSL_FIFO_DATA_OUT_L  		0X3E
#define LSM6DSL_FIFO_DATA_OUT_H  		0X3F
#define LSM6DSL_TIMESTAMP0_REG  			0X40
#define LSM6DSL_TIMESTAMP1_REG  			0X41
#define LSM6DSL_TIMESTAMP2_REG  			0X42
#define LSM6DSL_STEP_TIMESTAMP_L  		0X49
#define LSM6DSL_STEP_TIMESTAMP_H  		0X4A
#define LSM6DSL_STEP_COUNTER_L  			0X4B
#define LSM6DSL_STEP_COUNTER_H  			0X4C
#define LSM6DSL_SENSORHUB13_REG  		0X4D
#define LSM6DSL_SENSORHUB14_REG  		0X4E
#define LSM6DSL_SENSORHUB15_REG  		0X4F
#define LSM6DSL_SENSORHUB16_REG  		0X50
#define LSM6DSL_SENSORHUB17_REG  		0X51
#define LSM6DSL_SENSORHUB18_REG  		0X52
#define LSM6DSL_FUNC_SRC1  					0X53
#define LSM6DSL_FUNC_SRC2  					0X54
#define LSM6DSL_TAP_CFG  						0X58
#define LSM6DSL_TAP_THS_6D  					0X59
#define LSM6DSL_INT_DUR2  						0X5A
#define LSM6DSL_WAKE_UP_THS  				0X5B
#define LSM6DSL_WAKE_UP_DUR  				0X5C
#define LSM6DSL_FREE_FALL  					0X5D
#define LSM6DSL_MD1_CFG  						0X5E
#define LSM6DSL_MD2_CFG  						0X5F

/************** Access Device RAM  *******************/
#define LSM6DSL_MASTER_CMD_CODE  						0X60
#define LSM6DSL_SENS_SYNC_SPI_ERROR_CODE  		0X61

/************** Embedded functions register mapping  *******************/
#define LSM6DSL_OUT_MAG_RAW_X_L              0x66
#define LSM6DSL_OUT_MAG_RAW_X_H              0x67
#define LSM6DSL_OUT_MAG_RAW_Y_L              0x68
#define LSM6DSL_OUT_MAG_RAW_Y_H              0x69
#define LSM6DSL_OUT_MAG_RAW_Z_L              0x6A
#define LSM6DSL_OUT_MAG_RAW_Z_H              0x6B
#define LSM6DSL_X_OFS_USR                  	 0x73
#define LSM6DSL_Y_OFS_USR                    0x74
#define LSM6DSL_Z_OFS_USR                    0x75

#define LSM6DSL_STATUS_REG                   0x1E 

#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
