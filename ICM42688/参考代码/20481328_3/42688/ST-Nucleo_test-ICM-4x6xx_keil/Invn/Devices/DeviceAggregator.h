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

/** @defgroup DeviceAggregator DeviceAggregator
 *	@brief    Device implementation that merge several devices into one
 *
 *            This is useful for controlling multiple devices that provides differents sensors
 *            with only one handle.
 *
 *  @ingroup  Device
 *	@{
 */

#ifndef _INV_DEVICE_AGGREGATOR_H_
#define _INV_DEVICE_AGGREGATOR_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/Device.h"
#include "Invn/VSensor/VSensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief States for Aggregator device implementation
 */
typedef struct inv_device_aggregator
{
	inv_device_t          base;                 /**< base inv_device_t states */
	struct {
		inv_device_t * dev;                     /**< ref to device */
		uint64_t sensor_available_mask;         /**< mask to keep track of HW available sensor from underlying device */
	} hw_devices[4];                            /**< references to inv_device_t to aggregate */
	unsigned              hw_devices_cnt;       /**< number of sources */
	inv_sensor_listener_t private_listener;     /**< internal listener to catch events from HW devices */
} inv_device_aggregator_t;

/** @brief Initialize states for Aggregator device implementation
 *
 *  Assumes underlying devices are already initialized.
 *  Will take ownership of the underlying devices and re-use their listeners.
 *  No HW access are done in this function.
 *
 *  @param[in] self       handle to current device
 *  @param[in] hw_devicew array of pointer to inv_device_t to be aggregated
 *  @param[in] cnt        number of devices (4 max)
 *  @param[in] listener   new listener that will receives sensor events from all devices
 */
void inv_device_aggregator_init(inv_device_aggregator_t * self, 
		inv_device_t ** hw_device, unsigned cnt, const inv_sensor_listener_t * listener);

/** @brief Return generic handle to base inv_device_t
 */
static inline inv_device_t * inv_device_aggregator_get_base(inv_device_aggregator_t * self)
{
	return (self) ? &self->base : 0;
}

int INV_EXPORT inv_device_aggregator_reset(void * context);

/** @brief Initialize masks of available devices
 *
 *  Will call ping() for all sensors and all devices.
 *  Underlying devices should be setup and all FW/images loaded before calling this method.
 */
int INV_EXPORT inv_device_aggregator_setup(void * context);

int INV_EXPORT inv_device_aggregator_cleanup(void * context);

int INV_EXPORT inv_device_aggregator_ping_sensor(void * context, int sensor);

int INV_EXPORT inv_device_aggregator_enable_sensor(void * context, int sensor, inv_bool_t en);

int INV_EXPORT inv_device_aggregator_set_sensor_period_us(void * context, int sensor, uint32_t period);

int INV_EXPORT inv_device_aggregator_flush_sensor(void * context, int sensor);

int INV_EXPORT inv_device_aggregator_set_sensor_bias(void * context, int sensor, const float bias[3]);

int INV_EXPORT inv_device_aggregator_get_sensor_bias(void * context, int sensor, float bias[3]);

int INV_EXPORT inv_device_aggregator_set_sensor_mounting_matrix(void * context, int sensor, const float matrix[9]);

int INV_EXPORT inv_device_aggregator_get_sensor_data(void * context, int sensor, inv_sensor_event_t * event);

int INV_EXPORT inv_device_aggregator_set_sensor_config(void * context, int sensor, int setting, const void * value, unsigned size);

int INV_EXPORT inv_device_aggregator_get_sensor_config(void * context, int sensor, int setting, void * value, unsigned size);

int INV_EXPORT inv_device_aggregator_poll(void * context);

int INV_EXPORT inv_device_aggregator_self_test(void * context, int sensor);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_AGGREGATOR_H_ */
