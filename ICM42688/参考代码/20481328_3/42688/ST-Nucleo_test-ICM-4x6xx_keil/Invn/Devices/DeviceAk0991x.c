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

#include "DeviceAk0991x.h"

#include "Invn/Utils/Message.h"
#include <string.h>

/* Virtual functions table ****************************************************/

static const inv_device_vt_t device_ak0991x_vt = {
	inv_device_ak0991x_whoami,
	inv_device_ak0991x_reset,
	inv_device_ak0991x_setup,
	inv_device_ak0991x_cleanup,
	0,
	inv_device_ak0991x_poll,
	inv_device_ak0991x_self_test,
	0,
	inv_device_ak0991x_ping_sensor,
	0,
	inv_device_ak0991x_enable_sensor,
	inv_device_ak0991x_set_sensor_period_us,
	0,
	0,
	0,
	0,
	inv_device_ak0991x_set_sensor_mounting_matrix,
	0,
	inv_device_ak0991x_set_sensor_config,
	inv_device_ak0991x_get_sensor_config,
	0,
	0
};

/* For BW compatibility
 * wrapper function to adapt prototype of read_reg() as defined in inv_host_serif
 * to expected prototype for inv_serif_hal
 */
static int host_serif_read_reg_legacy(void * context, uint8_t reg, uint8_t * data, uint32_t len)
{
	inv_host_serif_t * serif = (inv_host_serif_t *)context;

	return serif->read_reg(reg, data, len);
}

/* For BW compatibility
 * wrapper function to adapt prototype of write_reg() as defined in inv_host_serif
 * to expected prototype for inv_serif_hal
 */
static int host_serif_write_reg_legacy(void * context, uint8_t reg, const uint8_t * data, uint32_t len)
{
	inv_host_serif_t * serif = (inv_host_serif_t *)context;

	return serif->write_reg(reg, data, len);
}

/* IDD function implementation ************************************************/

void inv_device_ak0991x_init(inv_device_ak0991x_t * self, 
        const inv_host_serif_t * serif, const inv_sensor_listener_t * listener)
{
	struct inv_ak0991x_serif ak0991x_serif;

	/* create an a inv_serif_hal_t object from a inv_host_serif_t */
	const inv_serif_hal_t serif_hal = {
		host_serif_read_reg_legacy, host_serif_write_reg_legacy, /* use small wrappers to adapt prototype */
		serif->max_read_size, serif->max_write_size,
		serif->serif_type, &self->legacy_serif
	};

	assert(self);
	memset(self, 0, sizeof(*self));

	/* build base */
	self->base.instance = self;
	self->base.vt       = &device_ak0991x_vt;
	self->base.listener = listener;
	self->setup_done    = 0;

	/* initialise mounting matrix of compass to identity */
	self->mounting_matrix_compass[0] = (1 << 30);
	self->mounting_matrix_compass[4] = (1 << 30);
	self->mounting_matrix_compass[8] = (1 << 30);

	/* initialize ak0991x serif structure */
	ak0991x_serif.context   = serif_hal.context;
	ak0991x_serif.read_reg  = serif_hal.read_reg;
	ak0991x_serif.write_reg = serif_hal.write_reg;
	ak0991x_serif.max_read  = serif_hal.max_read_transaction_size;
	ak0991x_serif.max_write = serif_hal.max_write_transaction_size;
	ak0991x_serif.is_spi    = !!(serif_hal.serif_type == INV_SERIF_HAL_TYPE_SPI);

	/* reset ak0991x driver states */
	inv_ak0991x_reset_states(&self->sensor_states, &ak0991x_serif);

	/* keep a copy of the user inv_host_serif_t (used in the _legacy callbacks) */
	self->legacy_serif = *serif;
}

/* Apply mounting matrix in Q30 to 16bits raw vector */
static void apply_mounting_matrix(const int32_t mmatrix[9], const int16_t raw_vect[3], int32_t out_vect[3])
{
	unsigned i;

	for(i = 0; i < 3; i++) {
		out_vect[i]  = (mmatrix[3*i+0] >> 30)*raw_vect[0];
		out_vect[i] += (mmatrix[3*i+1] >> 30)*raw_vect[1];
		out_vect[i] += (mmatrix[3*i+2] >> 30)*raw_vect[2];
	}
}

static void build_sensor_event_raw_mag(inv_device_ak0991x_t * self,
		int sensorid, uint64_t timestamp, const int16_t * raw_data, inv_sensor_event_t * event)
{
	assert(event);

	memset(event, 0, sizeof(*event));
	event->sensor	 = sensorid;
	event->timestamp = timestamp;
	event->status	 = INV_SENSOR_STATUS_DATA_UPDATED;

	apply_mounting_matrix(self->mounting_matrix_compass, raw_data, event->data.raw3d.vect);

	(void)self;
}

int inv_device_ak0991x_poll(void * context)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	int16_t compass_data[3];
	inv_sensor_event_t event;
	int rc;

	uint64_t timestamp = inv_ak0991x_get_time_us();
	if(timestamp - self->last_timestamp >= self->odr_requested) {
		rc = inv_ak0991x_poll_data(&self->sensor_states, compass_data);
		if(rc == 0) {
			build_sensor_event_raw_mag(self, INV_SENSOR_TYPE_RAW_MAGNETOMETER, timestamp, compass_data, &event);
			inv_sensor_listener_notify(self->base.listener, &event);
			self->last_timestamp = timestamp;
		}
		else 
			return rc;
	}else /* no data polled */
		return 1;

	return 0;
}

int inv_device_ak0991x_self_test(void * context, int sensor)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	if(sensor != INV_SENSOR_TYPE_RAW_MAGNETOMETER)
		return INV_ERROR_BAD_ARG;

	return inv_ak0991x_run_selftest(&self->sensor_states);
}

int inv_device_ak0991x_whoami(void * context, uint8_t * whoami)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	assert(whoami);

	return inv_ak0991x_get_whoami(&self->sensor_states, whoami);
}

int inv_device_ak0991x_reset(void * context)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;
	int rc;

	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reseting device...");

	if((rc = inv_ak0991x_soft_reset(&self->sensor_states)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "ak0991x soft reset returned %d", rc);
		return rc;
	}

	return 0;
}

int inv_device_ak0991x_setup(void * context)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;
	int rc;
	uint8_t whoami;

	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up Ak0991x...");

	/* Device soft reset */
	rc = inv_ak0991x_soft_reset(&self->sensor_states);
	if(rc != 0)
		return rc;
 
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reading Ak0991x WHOAMI...");
	if((rc = inv_device_ak0991x_whoami(self, &whoami)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d when reading WHOAMI value", rc);
		return rc;
	}

	if(whoami == 0 || whoami == 0xff) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected WHOAMI value 0x%x. Aborting setup.", whoami);
		return INV_ERROR;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "AK0991x WHOAMI value: 0x%x", whoami);
	}
	/* register sensitivity adjustment values */
	inv_ak0991x_retrieve_asa_values(&self->sensor_states);

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	/* set default ODR = 5Hz */
	inv_device_ak0991x_set_sensor_period_us(context, INV_SENSOR_TYPE_RAW_MAGNETOMETER, 200000/*us*/);

	self->setup_done = 1;

	return 0;
}

int inv_device_ak0991x_cleanup(void * context)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	return inv_ak0991x_soft_reset(&self->sensor_states);
}

int inv_device_ak0991x_ping_sensor(void * context, int sensor)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	/* HW sensors */
	if((sensor == INV_SENSOR_TYPE_RAW_MAGNETOMETER) && (self->setup_done))
		return 0;

	(void)context;

	return INV_ERROR_BAD_ARG;
}

int inv_device_ak0991x_enable_sensor(void * context, int sensor, inv_bool_t en)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	if(inv_device_ak0991x_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	return inv_ak0991x_enable_sensor(&self->sensor_states, en);
}

int inv_device_ak0991x_set_sensor_period_us(void * context, int sensor, uint32_t period)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	if(inv_device_ak0991x_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	if(period < INV_DEVCIE_AK0991X_ODR_MIN_DELAY) 
		period = INV_DEVCIE_AK0991X_ODR_MIN_DELAY;

	self->odr_requested = period;

	return 0;
}

int inv_device_ak0991x_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9])
{
	int i;
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *) context;

	if(inv_device_ak0991x_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	for(i = 0; i < 9; ++i)
		self->mounting_matrix_compass[i] = (int32_t)(matrix[i] * (1 << 30));

	return INV_ERROR_SUCCESS;
}

int inv_device_ak0991x_set_sensor_config(void * context, int sensor, int setting, const void * arg, unsigned size)
{
	(void)size;

	if(inv_device_ak0991x_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	switch (setting) {
		case INV_SENSOR_CONFIG_MOUNTING_MATRIX:
		{
			const inv_sensor_config_mounting_mtx_t * mtx = (const inv_sensor_config_mounting_mtx_t *)arg;
			return inv_device_ak0991x_set_sensor_mounting_matrix(context, sensor, mtx->matrix);
		}
		default:
			break;
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_ak0991x_get_sensor_config(void * context, int sensor, int setting, void * arg, unsigned size)
{
	inv_device_ak0991x_t * self = (inv_device_ak0991x_t *)context;

	if(inv_device_ak0991x_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	switch (setting) {
		case INV_SENSOR_CONFIG_MOUNTING_MATRIX:
		{
			int32_t * out = (int32_t *)arg;
			int i;

			if(size < 9*sizeof(int32_t))
				return INV_ERROR_BAD_ARG;

			switch (sensor) {
			case INV_SENSOR_TYPE_RAW_GYROSCOPE:
				for (i = 0; i < 9; i++)
					out[i] = self->mounting_matrix_compass[i];
				break;
			default:
				return INV_ERROR_BAD_ARG;
			}
			return (9*sizeof(int32_t));
		}
		default:
			return INV_ERROR_BAD_ARG;
	}
}