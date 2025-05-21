/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
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
//#include <asf.h>
#include "example-selftest.h"
#include "Icm406xxSelfTest.h"

/* InvenSense drivers and utils */
//#include "Invn/EmbUtils/Message.h"
#include "Message.h"

/* board driver */
#include "common.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the icm406xx object */
static struct inv_icm406xx icm_driver;


/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */
 
int SetupInvDevice(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),
				   int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len),
				   ICM406XX_SERIAL_IF_TYPE_t serif_type)
{
	int rc = 0;
	struct inv_icm406xx_serif icm406xx_serif;
	uint8_t who_am_i;

	INV_MSG(0, "#######################\r\n");
	INV_MSG(0, "#   SelfTest example  #\r\n");
	INV_MSG(0, "#######################\r\n");
	
	icm406xx_serif.context   = 0;       /* no need */
	icm406xx_serif.read_reg  = (*read_reg);
	icm406xx_serif.write_reg = (*write_reg);
	icm406xx_serif.max_read  = 1024*32; /* maximum number of bytes allowed per serial read */
	icm406xx_serif.max_write = 1024*32; /* maximum number of bytes allowed per serial write */
	icm406xx_serif.serif_type = serif_type;
	
	
	/* Initialize device */
	INV_MSG(0, "Initialize ICM4x6xx\r\n");
	
	rc = inv_icm406xx_init(&icm_driver, &icm406xx_serif, NULL);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm406xx.");
		return rc;
	}
	
	
	/* Check WHOAMI */
	INV_MSG(0, "Check Icm406xx whoami value\r\n");
	
	rc = inv_icm406xx_get_who_am_i(&icm_driver, &who_am_i);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(0, "!!! ERROR : failed to read Icm406xx whoami value.\r\n");
		return rc;
	}

	if((who_am_i != ICM40605_WHOAMI) && (who_am_i != ICM40604_WHOAMI) && (who_am_i != ICM40602_WHOAMI) && 
			(who_am_i != ICM42602_WHOAMI) && (who_am_i != ICM42605_WHOAMI)&& (who_am_i != ICM40607_WHOAMI)) 
	{
		INV_MSG(0, "!!! ERROR :  bad WHOAMI value. Got 0x%02x\r\n", who_am_i); //
		return INV_ERROR;
	}

	return rc;
}

void RunSelfTest(void)
{
	int rc = 0;

	rc = inv_icm406xx_run_selftest(&icm_driver);
	/* Check for GYR success (1 << 0) and ACC success (1 << 1) */
	if ( (rc & 0x1)) {
		INV_MSG(0, "Gyro Selftest PASS\r\n");
	} else {
		INV_MSG(0, "Gyro Selftest FAIL\r\n");
	}
	if ( (rc & 0x2)) {
		INV_MSG(0, "Accel Selftest PASS\r\n");
	} else {
		INV_MSG(0, "Accel Selftest FAIL\r\n");
	}
}

void GetBias(void)
{
	int raw_bias[12];

	/* Get Low Noise / Low Power bias computed by self-tests scaled by 2^16 */
	INV_MSG(0, "Getting LP/LN bias\r\n");
	inv_icm406xx_get_st_bias(&icm_driver, raw_bias);
	INV_MSG(0, "GYR LN bias (FS=2000dps) (dps): x=%f, y=%f, z=%f\r\n",
			(float)(raw_bias[0] / (float)(1 << 16)), (float)(raw_bias[1] / (float)(1 << 16)), (float)(raw_bias[2] / (float)(1 << 16)));
	INV_MSG(0, "ACC LN bias (FS=4g) (g): x=%f, y=%f, z=%f\r\n",
			(float)(raw_bias[0 + 6] / (float)(1 << 16)), (float)(raw_bias[1 + 6] / (float)(1 << 16)), (float)(raw_bias[2 + 6] / (float)(1 << 16)));
}
