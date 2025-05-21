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

/** @defgroup DriverIcm20690Setup Icm20690 driver setup
 *  @brief Low-level function to setup an Icm20690 device
 *  @ingroup  DriverIcm20690
 *  @{
 */

#ifndef _INV_ICM20690_SETUP_H_
#define _INV_ICM20690_SETUP_H_

#include "Invn/InvExport.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "Invn/InvBool.h"

/* forward declaration */
struct inv_icm20690;

/** @brief return WHOAMI value
 *  @param[out] whoami WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int INV_EXPORT inv_icm20690_get_whoami(struct inv_icm20690 * s, uint8_t * whoami);

/** @brief Initialize the device
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_initialize(struct inv_icm20690 * s);

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_soft_reset(struct inv_icm20690 * s);

/** @brief Set the mpu sample rate
 *  @return Value written to MPUREG_SMPLRT_DIV register.
 */
int INV_EXPORT inv_icm20690_set_divider(struct inv_icm20690 * s, uint8_t idiv);

int INV_EXPORT inv_icm20690_set_i2c_mst_dly(struct inv_icm20690 * s, uint16_t minDly, uint16_t minDly_compass);

/** @brief Sets the power state of the Ivory chip loop
 *  @param[in] func   CHIP_AWAKE, CHIP_LP_ENABLE
 *  @param[in] on_off The functions are enabled if previously disabled and
 *                	  disabled if previously enabled based on the value of On/Off.
 *  @return 0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_chip_power_state(struct inv_icm20690 * s, uint8_t func, uint8_t on_off);

/** @brief Current wake status of the Mems chip
 * @return the wake status
 */
uint8_t INV_EXPORT inv_icm20690_get_chip_power_state(struct inv_icm20690 * s);

/** @brief Get internal sample rate currently configured
 * @return Internal sample rate currently configured in MEMS registers in Hz
 */
uint16_t INV_EXPORT inv_icm20690_get_chip_base_sample_rate(struct inv_icm20690 * s);

/** @brief Get internal register value for given FSR in mg for Accelerometer
 *  Allowed value are: 2000 (+/-2g), 4000 (+/-4g), 8000 (+/-8g), 16000 (+/-16g), 
 *  @return internal register value for FSR confiugration
 */
int INV_EXPORT inv_icm20690_accel_fsr_2_reg(int32_t fsr);

/** @brief Sets fullscale range of accel in hardware.
 * @param[in] level  See mpu_accel_fs.
 * @return 			0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_accel_fullscale(struct inv_icm20690 * s, int level);

/** @brief Sets fullscale range of OIS accel in hardware.
 *  @param[in] level  See mpu_accel_ois_fs.
 *  @return 			0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_accel_ois_fullscale(struct inv_icm20690 * s, int level);

/** @brief Sets bandwidth range of accel in hardware.
 * @param[in] level  See mpu_accel_bw.
 * @return 			0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_accel_bandwidth(struct inv_icm20690 * s, int level);

/** @brief Returns fullscale range of accelerometer in hardware
 *  @return the fullscale range
 */
uint8_t INV_EXPORT inv_icm20690_get_accel_fullscale(struct inv_icm20690 * s);

/** @brief Returns fullscale range of OIS accelerometer in hardware
 *  @return the fullscale range
 */
uint8_t INV_EXPORT inv_icm20690_get_accel_ois_fullscale(struct inv_icm20690 * s);

/** @brief Returns bandwidth range of accelerometer in hardware
 *  @return the bandwidth range
 */
uint16_t INV_EXPORT inv_icm20690_get_accel_bandwidth(struct inv_icm20690 * s);

/** @brief Get internal register value for given FSR in mg for Gyroscope
 *  Allowed value are: 250 (+/-250dsp), 500 (+/-500dps), 1000 (+/-dps), 2000 (+/-2000dps), 
 *  @return internal register value for FSR confiugration
 */
int INV_EXPORT inv_icm20690_gyro_fsr_2_reg(int32_t fsr);

/** @brief Sets fullscale range of gyro in hardware.
 *  @param[in]  level  See mpu_gyro_fs.
 *  @return 				0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_gyro_fullscale(struct inv_icm20690 * s, int level);

/** @brief Sets fullscale range of OIS gyro in hardware.
 *  @param[in]  level  See mpu_gyro_fs.
 *  @return 				0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_gyro_ois_fullscale(struct inv_icm20690 * s, int level);

/** @brief Sets bandwidth range of gyro in hardware.
 *  @param[in]  level  See mpu_gyro_bw.
 *  @return 				0 on success, negative value on error.
 */
int INV_EXPORT inv_icm20690_set_gyro_bandwidth(struct inv_icm20690 * s, int level);

/** @brief Returns fullscale range of gyroscope in hardware
 *  @return the fullscale range
 */
uint8_t INV_EXPORT inv_icm20690_get_gyro_fullscale(struct inv_icm20690 * s);

/** @brief Returns fullscale range of OIS gyroscope in hardware
 *  @return the fullscale range
 */
uint8_t INV_EXPORT inv_icm20690_get_gyro_ois_fullscale(struct inv_icm20690 * s);

/** @brief Returns bandwidth range of gyroscope in hardware
 *  @return the bandwidth range
 */
uint16_t INV_EXPORT inv_icm20690_get_gyro_bandwidth(struct inv_icm20690 * s);

/** @brief Get the available features according the base sensor device
 *  @param[in] in  	0: No advanced features supported
 *                  1: OIS sensor and FSYNC behavior supported (for icm20690 only)
 */
int INV_EXPORT inv_icm20690_is_advanced_features_supported(void);

/** @brief Set FSYNC bit location
 *  @param[in] in  	0: Disable FSYNC pin data to be sampled
 *                  1: Enable FSYNC pin data to be sampled at TEMP_OUT_L[0]
 */
int INV_EXPORT inv_icm20690_set_fsync_bit_location(struct inv_icm20690 * s, int bit_location);


#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM20690_SETUP_H_ */

/** @} */
