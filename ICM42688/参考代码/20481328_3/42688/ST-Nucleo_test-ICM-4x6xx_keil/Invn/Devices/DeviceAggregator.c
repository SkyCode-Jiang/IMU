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

#include "DeviceAggregator.h"

#include <string.h>

/* Virtual functions table ****************************************************/

static const inv_device_vt_t device_vt = {
	0,
	inv_device_aggregator_reset,
	inv_device_aggregator_setup,
	inv_device_aggregator_cleanup,
	0,
	inv_device_aggregator_poll,
	inv_device_aggregator_self_test,
	0,
	inv_device_aggregator_ping_sensor,
	0,
	inv_device_aggregator_enable_sensor,
	inv_device_aggregator_set_sensor_period_us,
	0,
	inv_device_aggregator_flush_sensor,
	inv_device_aggregator_set_sensor_bias,
	inv_device_aggregator_get_sensor_bias,
	inv_device_aggregator_set_sensor_mounting_matrix,
	inv_device_aggregator_get_sensor_data,
	inv_device_aggregator_set_sensor_config,
	inv_device_aggregator_get_sensor_config,
	0,
	0
};

/* Look at bit masks and return first HW device that have this sensor
 */
static unsigned get_device_idx(inv_device_aggregator_t * self, int sensor)
{
	unsigned idevice;

	sensor = INV_SENSOR_ID_TO_TYPE(sensor);

	for(idevice = 0; idevice < self->hw_devices_cnt; ++idevice) {

		if((1ULL << sensor) & self->hw_devices[idevice].sensor_available_mask) {
			return idevice;
		}
	}

	return sizeof(self->hw_devices) / sizeof(self->hw_devices[0]);
}

/* Internal IDD listener sensor event
 * Will catch sensor events comming from any underlying devices and to IDD clent
 */
static void hw_device_event_cb(const inv_sensor_event_t * event, void * context)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	/* Forward event directly to IDD client */
	inv_sensor_listener_notify(self->base.listener, event);
}

/* IDD function implementation ************************************************/

void inv_device_aggregator_init(inv_device_aggregator_t * self,
		inv_device_t ** hw_devices, unsigned cnt, const inv_sensor_listener_t * listener)
{
	unsigned i;

	memset(self, 0, sizeof(*self));
	self->base.instance = self;
	self->base.vt       = &device_vt;
	self->base.listener = listener;

	inv_sensor_listener_init(&self->private_listener, hw_device_event_cb, self);

	/* Override HW device listener */
	for(i = 0; i < cnt; i++) {
		if(hw_devices[i]) {
			self->hw_devices[self->hw_devices_cnt++].dev = hw_devices[i];
			hw_devices[i]->listener = &self->private_listener;
		}
	}
}

int inv_device_aggregator_poll(void * context)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;
	int rc = 0;
	unsigned i;

	/* Poll all HW devices */
	for(i = 0; i < self->hw_devices_cnt; ++i) {
		rc += inv_device_poll(self->hw_devices[i].dev);
	}

	return rc;
}

int inv_device_aggregator_self_test(void * context, int sensor)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;
	unsigned idx = get_device_idx(self, sensor);


	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_self_test(self->hw_devices[idx].dev, sensor);
}

int inv_device_aggregator_reset(void * context)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;
	unsigned idevice;
	int rc = 0;

	for(idevice = 0; idevice < self->hw_devices_cnt; ++idevice) {
		rc += inv_device_reset(self->hw_devices[idevice].dev);
	}

	return rc;
}

int inv_device_aggregator_setup(void * context)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;
	unsigned idevice;

	for(idevice = 0; idevice < self->hw_devices_cnt; ++idevice) {
		int sensor;

		self->hw_devices[idevice].sensor_available_mask = 0;

		/* Ping all hw sensors and set those that are available */
		for(sensor = INV_SENSOR_TYPE_RESERVED + 1; sensor < INV_SENSOR_TYPE_MAX; ++sensor) {

			const int rc = inv_device_ping_sensor(self->hw_devices[idevice].dev, sensor);

			if(rc == 0) {
				self->hw_devices[idevice].sensor_available_mask |= (1ULL << sensor);
			}
		}
	}

	return 0;
}

int inv_device_aggregator_cleanup(void * context)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	(void)self;

	/* Nothing to do for now...*/
	
	return 0;
}

int inv_device_aggregator_ping_sensor(void * context, int sensor)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	if(get_device_idx(self, sensor) >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR;

	return 0;
}

int inv_device_aggregator_enable_sensor(void * context, int sensor, inv_bool_t en)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_enable_sensor(self->hw_devices[idx].dev, sensor, en);
}


int inv_device_aggregator_set_sensor_period_us(void * context,
		int sensor, uint32_t period)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_set_sensor_period_us(self->hw_devices[idx].dev, sensor, period);
}

int inv_device_aggregator_flush_sensor(void * context, int sensor)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_flush_sensor(self->hw_devices[idx].dev, sensor);
}

int inv_device_aggregator_set_sensor_bias(void * context, int sensor,
		const float bias[3])
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_set_sensor_bias(self->hw_devices[idx].dev, sensor, bias);
}

int inv_device_aggregator_get_sensor_bias(void * context, int sensor,
		float bias[3])
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_get_sensor_bias(self->hw_devices[idx].dev, sensor, bias);
}

int inv_device_aggregator_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9])
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_set_sensor_mounting_matrix(self->hw_devices[idx].dev, sensor, matrix);
}

int inv_device_aggregator_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_get_sensor_data(self->hw_devices[idx].dev, sensor, event);
}

int inv_device_aggregator_set_sensor_config(void * context, int sensor, int setting,
		const void * value, unsigned size)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_set_sensor_config(self->hw_devices[idx].dev, sensor, setting, value, size);
}

int inv_device_aggregator_get_sensor_config(void * context, int sensor, int setting,
		void * value, unsigned size)
{
	inv_device_aggregator_t * self = (inv_device_aggregator_t *)context;

	unsigned idx = get_device_idx(self, sensor);

	if(idx >= (sizeof(self->hw_devices) / sizeof(self->hw_devices[0])))
		return INV_ERROR_BAD_ARG;

	return inv_device_get_sensor_config(self->hw_devices[idx].dev, sensor, setting, value, size);
}
