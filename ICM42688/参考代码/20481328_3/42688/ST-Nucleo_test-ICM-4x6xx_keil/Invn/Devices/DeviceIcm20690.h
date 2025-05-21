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

/** @defgroup DeviceIcm20690 DeviceIcm20690
 *	@brief    Concrete implementation of the 'Device' interface for Icm20690 devices
 *
 *            See @ref ExampleDeviceIcm20690.c example.
 *
 *  @ingroup  Device
 *	@{
 */

#ifndef _INV_DEVICE_ICM20690_H_
#define _INV_DEVICE_ICM20690_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/Device.h"
#include "Invn/Devices/SerifHal.h"

#include "Invn/Devices/Drivers/Icm20690/Icm20690.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief States for Icm20690 device
 */
typedef struct inv_device_icm20690
{
	inv_device_t              base;
	struct inv_icm20690 basesensor_states;
	inv_bool_t                reset_done;
	int32_t mounting_matrix[9];
	int32_t mounting_matrix_secondary_compass[9];
	uint8_t wom_threshold;
	uint64_t last_timestamp;
	inv_host_serif_t          legacy_serif;     /* used for backward API compatibility */
	inv_host_serif_t          legacy_serif_ois; /* used for backward API compatibility */
} inv_device_icm20690_t;

/** @brief constructor-like function for basesensor device
 *
 *  Will initialize inv_device_icm20690_t object states to default value for basesensor.
 *
 *  @param[in] self         handle to device
 *  @param[in] serif        reference to Serial Interface object
 *  @param[in] listener     reference to Sensor Event Listener object
 */
void INV_EXPORT inv_device_icm20690_init2(inv_device_icm20690_t * self,
		const inv_serif_hal_t * serif, const inv_sensor_listener_t * listener);

/** @brief constructor-like function for basesensor device
 *
 *  @deprecated Use innv_device_icm20690_init2() instead.
 *
 *  Will initialize inv_device_icm20690_t object states to default value for basesensor.
 *
 *  @param[in] self         handle to device
 *  @param[in] serif        reference to legacy Host Serial Interface object
 *  @param[in] listener     reference to Sensor Event Listener object
 */
void INV_EXPORT inv_device_icm20690_init(inv_device_icm20690_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener);

/** @brief Set reference to Serial Interface object for OIS interface
 *
 *  When set, a call to the poll() method will also retrieve and report OIS data (if enabled)
 *
 *  Should be called after inv_device_icm20690_init2() but before device setup.
 *
 *  @param[in] self         handle to device
 *  @param[in] serif_ois    reference to Serial Interface object for OIS interface
 */
void INV_EXPORT inv_device_icm20690_init_serif_ois2(inv_device_icm20690_t * self,
		const inv_serif_hal_t * serif_ois);

/** @brief Set reference to Serial Interface object for OIS interface
 *
 *  @deprecated Use innv_device_icm20690_init_serif_ois2() instead.
 *
 *  @param[in] self         handle to device
 *  @param[in] serif        reference to legacy Host Serial Interface object for OIS interface
 */
void INV_EXPORT inv_device_icm20690_init_serif_ois(inv_device_icm20690_t * self,
		const inv_host_serif_t * serif_ois);

/** @brief Register a compass as AUX senosr
 *
 *  Should be called after inv_device_icm20690_init() but before device setup.
 *
 *  @param[in] aux_compass_id   auxiliary compass id (as of @ref enum inv_icm20690_compass_id)
 *  @param[in] aux_compass_add  I2C slave address for compass
 */
void INV_EXPORT inv_device_icm20690_init_aux_compass(inv_device_icm20690_t * self,
		int aux_compass_id, uint8_t aux_compass_addr);

/** @brief Allowed config setting value for Icm20690 device.
 */
enum  inv_device_icm20690_config {
	INV_DEVICE_ICM20690_CONFIG_MOUNTING_MATRIX = INV_SENSOR_CONFIG_MOUNTING_MATRIX,
	INV_DEVICE_ICM20690_CONFIG_FSR             = INV_SENSOR_CONFIG_FSR,
	INV_DEVICE_ICM20690_CONFIG_RESET           = INV_SENSOR_CONFIG_RESET,
	INV_DEVICE_ICM20690_CONFIG_BIAS_ST         = INV_SENSOR_CONFIG_CUSTOM,
	INV_DEVICE_ICM20690_CONFIG_WOM_THRESHOLD,
};

/** @brief Bias collected during self-test (config INV_DEVICE_ICM20690_CONFIG_BIAS_ST)
 *         Value is  scaled by 2^16, accel is gee and gyro is dps
 */
typedef struct inv_device_icm20690_config_bias_st {
	int32_t gyr_bias_lp[3]; /**< Gyro LP mode X,Y,Z */
	int32_t gyr_bias_nl[3]; /**< Gyro normal mode X,Y,Z */
	int32_t acc_bias_lp[3]; /**< Accel LP mode X,Y,Z */
	int32_t acc_bias_nl[3]; /**< Accel normal mode X,Y,Z */
}  inv_device_icm20690_config_bias_st_t;

/** @brief WOM threshold value (expressed in LSB)
 */
typedef int32_t inv_device_icm20690_config_wom_threshold_t;

/** @brief Helper function to get handle to base object
 */
static inline inv_device_t * inv_device_icm20690_get_base(inv_device_icm20690_t * self)
{
	if(self)
		return &self->base;

	return 0;
}

/*
 * Functions below are described in Device.h
 */

int INV_EXPORT inv_device_icm20690_whoami(void * context, uint8_t * whoami);

int INV_EXPORT inv_device_icm20690_reset(void * context);

int INV_EXPORT inv_device_icm20690_setup(void * context);

int INV_EXPORT inv_device_icm20690_cleanup(void * context);

int INV_EXPORT inv_device_icm20690_ping_sensor(void * context, int sensor);

int INV_EXPORT inv_device_icm20690_enable_sensor(void * context, int sensor, inv_bool_t en);

int INV_EXPORT inv_device_icm20690_set_sensor_period_us(void * context,
		int sensor, uint32_t period);

// int INV_EXPORT inv_device_icm20690_set_sensor_timeout(void * context,
// 		int sensor, uint32_t timeout);

// int INV_EXPORT inv_device_icm20690_flush_sensor(void * context, int sensor);

// int INV_EXPORT inv_device_icm20690_set_sensor_bias(void * context, int sensor,
// 		const float bias[3]);

// int INV_EXPORT inv_device_icm20690_get_sensor_bias(void * context, int sensor,
// 		float bias[3]);

int INV_EXPORT inv_device_icm20690_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9]);

// int INV_EXPORT inv_device_icm20690_get_sensor_data(void * context, int sensor,
// 		inv_sensor_event_t * event);

int INV_EXPORT inv_device_icm20690_poll(void * context);

int INV_EXPORT inv_device_icm20690_self_test(void * context, int sensor);

int INV_EXPORT inv_device_icm20690_get_sensor_config(void * context, int sensor, int setting, void * arg, unsigned size);

int INV_EXPORT inv_device_icm20690_set_sensor_config(void * context, int sensor, int setting, const void * arg, unsigned size);

int INV_EXPORT inv_device_icm20690_write_mems_register(void * context, int sensor, uint16_t reg_addr,
		const void * data, unsigned size);

int INV_EXPORT inv_device_icm20690_read_mems_register(void * context, int sensor, uint16_t reg_addr,
		void * data, unsigned size);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_ICM20690_H_ */


/** @} */
