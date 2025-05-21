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

/** @defgroup DeviceAk0991x DeviceAk0991x
 *	@brief    Device implementation for Ak0991x devices
 *
 *  @ingroup  Device
 *	@{
 */
 
#ifndef _INV_DEVICE_AK0991X_H_
#define _INV_DEVICE_AK0991X_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/Device.h"
#include "Invn/Devices/SerifHal.h"

#include "Invn/Devices/Drivers/Ak0991x/Ak0991x.h"

#ifdef __cplusplus
extern "C" {
#endif

#define INV_DEVCIE_AK0991X_ODR_MIN_DELAY         10000//us

/** @brief States for Ak0991x device
 */
typedef struct inv_device_ak0991x
{
	inv_device_t   base;
	struct inv_ak0991x sensor_states;
	uint32_t odr_requested;
	uint64_t last_timestamp;                    /* used to prevent data polling at higher frequency than requested */
	int32_t mounting_matrix_compass[9];
	inv_host_serif_t          legacy_serif;     /* used for backward API compatibility */
	uint8_t setup_done; 
} inv_device_ak0991x_t;

/** @brief constructor-like function for Ak0991x device
 *
 *  Will initialize inv_device_ak0991x_t object states to default value.
 *
 *  @param[in] self         handle to device
 *  @param[in] serif        reference to Serial Interface HAL object
 *  @param[in] listener     reference to Sensor Event Listener object
 */
void INV_EXPORT inv_device_ak0991x_init(inv_device_ak0991x_t * self, 
        const inv_host_serif_t * serif, const inv_sensor_listener_t * listener);

/** @brief Helper function to get handle to base object
 */
static inline inv_device_t * inv_device_ak0991x_get_base(inv_device_ak0991x_t * self)
{
	if(self)
		return &self->base;

	return 0;
}

/*
 * Functions below are described in Device.h
 */

int INV_EXPORT inv_device_ak0991x_whoami(void * context, uint8_t * whoami);

int INV_EXPORT inv_device_ak0991x_reset(void * context);

int INV_EXPORT inv_device_ak0991x_setup(void * context);

int INV_EXPORT inv_device_ak0991x_cleanup(void * context);

int INV_EXPORT inv_device_ak0991x_ping_sensor(void * context, int sensor);

int INV_EXPORT inv_device_ak0991x_enable_sensor(void * context, int sensor, inv_bool_t en);

int INV_EXPORT inv_device_ak0991x_set_sensor_period_us(void * context,
		int sensor, uint32_t period);

int INV_EXPORT inv_device_ak0991x_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9]);

int INV_EXPORT inv_device_ak0991x_poll(void * context);

int INV_EXPORT inv_device_ak0991x_self_test(void * context, int sensor);

int INV_EXPORT inv_device_ak0991x_get_sensor_config(void * context, int sensor, int setting, void * arg, unsigned size);

int INV_EXPORT inv_device_ak0991x_set_sensor_config(void * context, int sensor, int setting, const void * arg, unsigned size);

#endif /*_INV_DEVICE_AK0991X_H_ */
