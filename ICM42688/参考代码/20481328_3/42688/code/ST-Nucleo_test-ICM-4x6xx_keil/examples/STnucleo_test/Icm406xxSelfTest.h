/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

/** @defgroup DriverIcm406xxSelfTest Icm406xx selftest
 *  @brief Low-level function to run selftest on a Icm406xx device
 *  @ingroup  DriverIcm406xx
 *  @{
 */

/** @file Icm406xxSelfTest.h
 * Low-level function to run selftest on a Icm406xx device
 */

#ifndef _INV_ICM406XX_SELFTEST_H_
#define _INV_ICM406XX_SELFTEST_H_

//--yd #include "Invn/InvExport.h"
#include "InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_icm406xx;

/**
*  @brief      Perform hardware self-test for Accel and Gyro
*  @param[in]  None
*  @return     ACCEL_SUCCESS<<1 | GYRO_SUCCESS so 3
*/
int inv_icm406xx_run_selftest(struct inv_icm406xx * s);

/**
*  @brief      Retrieve bias collected by self-test.
*  @param[out] st_bias bias scaled by 2^16, accel is gee and gyro is dps.
*                      The buffer will be stuffed in order as below.
*                      Gyro normal mode X,Y,Z
*                      Gyro LP mode X,Y,Z
*                      Accel normal mode X,Y,Z
*                      Accel LP mode X,Y,Z
*/
void INV_EXPORT inv_icm406xx_get_st_bias(struct inv_icm406xx * s, int * st_bias);

/**
*  @brief      Apply bias.
*  @param[in] st_bias bias scaled by 2^16, accel is gee and gyro is dps.
*                      The buffer should be be stuffed in order as below.
*                      Gyro normal mode X,Y,Z
*                      Gyro LP mode X,Y,Z
*                      Accel normal mode X,Y,Z
*                      Accel LP mode X,Y,Z
*/
void inv_icm406xx_set_st_bias(struct inv_icm406xx * s, int * st_bias);

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM406XX_SELFTEST_H_ */

/** @} */
