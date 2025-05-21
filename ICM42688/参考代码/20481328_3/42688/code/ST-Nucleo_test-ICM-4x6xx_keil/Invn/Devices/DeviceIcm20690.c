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

#include "DeviceIcm20690.h"

#include "Invn/Utils/Message.h"

static const inv_device_vt_t device_icm20690_vt = {
	inv_device_icm20690_whoami,
	inv_device_icm20690_reset,
	inv_device_icm20690_setup,
	inv_device_icm20690_cleanup,
	0,//inv_device_icm20690_load,
	inv_device_icm20690_poll,
	inv_device_icm20690_self_test,
	0,//inv_device_icm20690_get_fw_info,
	inv_device_icm20690_ping_sensor,
	0,//inv_device_icm20690_set_running_state,
	inv_device_icm20690_enable_sensor,
	inv_device_icm20690_set_sensor_period_us,
	0,//inv_device_icm20690_set_sensor_timeout,
	0,//inv_device_icm20690_flush_sensor,
	0,//inv_device_icm20690_set_sensor_bias,
	0,//inv_device_icm20690_get_sensor_bias,
	inv_device_icm20690_set_sensor_mounting_matrix,
	0,//inv_device_icm20690_get_sensor_data,
	inv_device_icm20690_set_sensor_config,
	inv_device_icm20690_get_sensor_config,
	inv_device_icm20690_write_mems_register,
	inv_device_icm20690_read_mems_register,
};

static enum inv_icm20690_sensor idd_sensortype_2_driver(int sensor)
{
	switch(sensor) {
	case INV_SENSOR_TYPE_RAW_ACCELEROMETER: return INV_ICM20690_SENSOR_ACCEL;
	case INV_SENSOR_TYPE_RAW_GYROSCOPE:     return INV_ICM20690_SENSOR_GYRO;
	case INV_SENSOR_TYPE_RAW_MAGNETOMETER:  return INV_ICM20690_SENSOR_COMPASS;
	case INV_SENSOR_TYPE_RAW_TEMPERATURE:   return INV_ICM20690_SENSOR_TEMPERATURE;
	case INV_SENSOR_TYPE_OIS:               return INV_ICM20690_SENSOR_OIS;
	case INV_SENSOR_TYPE_FSYNC_EVENT:       return INV_ICM20690_SENSOR_FSYNC_EVENT;
	default:                                return INV_ICM20690_SENSOR_MAX;
	}
}

void inv_device_icm20690_init2(inv_device_icm20690_t * self,
		const inv_serif_hal_t * serif, const inv_sensor_listener_t * listener)
{
	struct inv_icm20690_serif icm20690_serif;

	assert(self);

	memset(self, 0, sizeof(*self));

	/* build base */
	self->base.instance = self;
	self->base.vt       = &device_icm20690_vt;
	self->base.listener = listener;

	/* initialise mounting matrix to identity */
	self->mounting_matrix[0] = (1 << 30);
	self->mounting_matrix[4] = (1 << 30);
	self->mounting_matrix[8] = (1 << 30);

	/* initialise mounting matrix of compass to identity */
	self->mounting_matrix_secondary_compass[0] = (1 << 30);
	self->mounting_matrix_secondary_compass[4] = (1 << 30);
	self->mounting_matrix_secondary_compass[8] = (1 << 30);

	/* set default threshold value */
	self->wom_threshold = 0x20;

	/* initialize icm20690 serif structure */
	icm20690_serif.context   = serif->context;
	icm20690_serif.read_reg  = serif->read_reg;
	icm20690_serif.write_reg = serif->write_reg;
	icm20690_serif.max_read  = serif->max_read_transaction_size;
	icm20690_serif.max_write = serif->max_write_transaction_size;
	icm20690_serif.is_spi    = !!(serif->serif_type == INV_SERIF_HAL_TYPE_SPI);

	/* reset icm20690 driver states */
	inv_icm20690_reset_states(&self->basesensor_states, &icm20690_serif);
}

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

void inv_device_icm20690_init(inv_device_icm20690_t * self,
		const inv_host_serif_t * serif, const inv_sensor_listener_t * listener)
{
	/* create an a inv_serif_hal_t object from a inv_host_serif_t */
	const inv_serif_hal_t serif_hal = {
		host_serif_read_reg_legacy, host_serif_write_reg_legacy, /* use small wrappers to adapt prototype */
		serif->max_read_size, serif->max_write_size,
		serif->serif_type, &self->legacy_serif
	};

	/* call the 'constructor' */
	inv_device_icm20690_init2(self, &serif_hal, listener);
	/* keep a copy of the user inv_host_serif_t (used in the _legacy callbacks) */
	self->legacy_serif = *serif;
}

void inv_device_icm20690_init_serif_ois2(inv_device_icm20690_t * self,
		const inv_serif_hal_t * serif_ois)
{
	struct inv_icm20690_serif icm20690_serif_ois;

	assert(self);

	icm20690_serif_ois.context   = serif_ois->context;
	icm20690_serif_ois.read_reg  = serif_ois->read_reg;
	icm20690_serif_ois.write_reg = serif_ois->write_reg;
	icm20690_serif_ois.max_read  = serif_ois->max_read_transaction_size;
	icm20690_serif_ois.max_write = serif_ois->max_write_transaction_size;
	icm20690_serif_ois.is_spi    = !!(serif_ois->serif_type == INV_SERIF_HAL_TYPE_SPI);

	/* reset icm20690 driver states */
	inv_icm20690_reset_states_serif_ois(&self->basesensor_states, &icm20690_serif_ois);
}

void inv_device_icm20690_init_serif_ois(inv_device_icm20690_t * self,
		const inv_host_serif_t * serif)
{
	/* create an a inv_serif_hal_t object from a inv_host_serif_t */
	const inv_serif_hal_t serif_hal = {
		host_serif_read_reg_legacy, host_serif_write_reg_legacy, /* use small wrappers to adapt prototype */
		serif->max_read_size, serif->max_write_size,
		serif->serif_type, &self->legacy_serif_ois
	};

	inv_device_icm20690_init_serif_ois2(self, &serif_hal);
	/* keep a copy of the user inv_host_serif_t (used in the _legacy callbacks) */
	self->legacy_serif_ois = *serif;
}

void inv_device_icm20690_init_aux_compass(inv_device_icm20690_t * self,
	int aux_compass_id, uint8_t aux_compass_addr
)
{
	/* register auxiliary compass (assuming AK09912) */
	inv_icm20690_register_aux_compass(&self->basesensor_states,
			(enum inv_icm20690_compass_id)aux_compass_id, aux_compass_addr);
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

static void build_sensor_event_raw(inv_device_icm20690_t * self, const int16_t * raw_data, inv_sensor_event_t * event)
{
	assert(event);

	apply_mounting_matrix(self->mounting_matrix, raw_data, event->data.raw3d.vect);
}

static void build_sensor_event_raw_compass(inv_device_icm20690_t * self, const int16_t * raw_data, inv_sensor_event_t * event)
{
	assert(event);

	apply_mounting_matrix(self->mounting_matrix_secondary_compass, raw_data, event->data.raw3d.vect);
}

static void build_sensor_event(inv_device_icm20690_t * self,
		int sensorid, uint64_t timestamp, inv_sensor_event_t * event)
{
	assert(event);

	memset(event, 0, sizeof(*event));
	event->sensor	 = sensorid;
	event->timestamp = timestamp;
	event->status	 = INV_SENSOR_STATUS_DATA_UPDATED;

	(void)self;
}

int inv_device_icm20690_poll(void * context)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	int rc;
	uint8_t int_status;
	uint8_t wom_status, ddry;
	uint64_t timestamp;
	inv_sensor_event_t event;
	
	/* if all sensors are of we skip the polling to avoid high idle consumption in idle*/
	if(inv_icm20690_all_sensors_off(&self->basesensor_states))
		return 0;

	if((rc = inv_icm20690_get_int_status(&self->basesensor_states, &int_status)) != 0)
		return rc;

	wom_status = inv_icm20690_check_wom_status(&self->basesensor_states, int_status);
	ddry       = inv_icm20690_check_drdy(&self->basesensor_states, int_status);

	if(wom_status || ddry) {
		timestamp = inv_icm20690_get_time_us();
	}

	if(wom_status) {
		build_sensor_event(self, INV_SENSOR_TYPE_WOM, timestamp, &event);
		event.data.wom.flags = wom_status;
		inv_sensor_listener_notify(self->base.listener, &event);
	}

	if(ddry) {
		struct inv_icm20690_fifo_states fifo_states;

		rc = inv_icm20690_poll_fifo_data_setup(&self->basesensor_states, &fifo_states, int_status);
		if(rc == 1) {
			/* overflow detected */
			INV_MSG(INV_MSG_LEVEL_WARNING, "FIFO overflow detected!");
			return inv_icm20690_reset_fifo(&self->basesensor_states);
		}
		else if(rc < 0) {
			return rc;
		}
		else if(fifo_states.packet_count > 0 && fifo_states.packet_size > 0) {
			/* Read FIFO only when data is expected in FIFO */

			/* read the FIFO */
			uint64_t ts_interval = 0;
			int16_t acc_data[3], temp_data, gyro_data[3], compass_data[3];

			if(fifo_states.packet_count > 1) {
				ts_interval = (timestamp - self->last_timestamp) / fifo_states.packet_count;
				timestamp = self->last_timestamp;
			}

			while((rc = inv_icm20690_poll_fifo_data(&self->basesensor_states, &fifo_states, acc_data, &temp_data, gyro_data, compass_data)) > 0) {

				timestamp += ts_interval;
                
				/* Poll FSYNC tag and FSYNC ODR delay counter */
				if(inv_icm20690_is_sensor_enabled(&self->basesensor_states, INV_ICM20690_SENSOR_FSYNC_EVENT)) {
					int16_t delay_count = 0;
					int ret = inv_icm20690_poll_delay_count(&self->basesensor_states, &delay_count);
					
					if(ret & (1 << INV_ICM20690_SENSOR_FSYNC_EVENT)) {
						/* new FSYNC tag */
						build_sensor_event(self, INV_SENSOR_TYPE_FSYNC_EVENT, timestamp, &event);
						event.data.fsync_event.delay_count = delay_count;
						inv_sensor_listener_notify(self->base.listener, &event);
					}
				}

				/* new accel data */
				if(rc & (1 << INV_ICM20690_SENSOR_ACCEL)) {
					build_sensor_event(self, INV_SENSOR_TYPE_RAW_ACCELEROMETER, timestamp, &event);
					build_sensor_event_raw(self, acc_data, &event);
					inv_sensor_listener_notify(self->base.listener, &event);
				}

				/* new temp data */
				if(rc & (1 << INV_ICM20690_SENSOR_TEMPERATURE)) {
					build_sensor_event(self, INV_SENSOR_TYPE_RAW_TEMPERATURE, timestamp, &event);
					event.data.rawtemp.raw = temp_data;
					inv_sensor_listener_notify(self->base.listener, &event);
				}

				/* new gyro data */
				if(rc & (1 << INV_ICM20690_SENSOR_GYRO)) {
					build_sensor_event(self, INV_SENSOR_TYPE_RAW_GYROSCOPE, timestamp, &event);
					build_sensor_event_raw(self, gyro_data, &event);
					inv_sensor_listener_notify(self->base.listener, &event);
				}

				/* new compass data */
				if(rc & (1 << INV_ICM20690_SENSOR_COMPASS)) {
					build_sensor_event(self, INV_SENSOR_TYPE_RAW_MAGNETOMETER, timestamp, &event);
					build_sensor_event_raw_compass(self, compass_data, &event);
					inv_sensor_listener_notify(self->base.listener, &event);
				}
			}

			self->last_timestamp = timestamp;
		}
	}

	/* Poll OIS data if enabled (only if secondary SERIF HAL registered on init) */
	if(inv_icm20690_is_sensor_enabled(&self->basesensor_states, INV_ICM20690_SENSOR_OIS)) {
		int16_t ois_gyro_data[3];
		rc = inv_icm20690_poll_ois_gyro_data(&self->basesensor_states, ois_gyro_data);

		if(rc & (1 << INV_ICM20690_SENSOR_OIS)) {
			timestamp = inv_icm20690_get_time_us();
			build_sensor_event(self, INV_SENSOR_TYPE_OIS, timestamp, &event);
			build_sensor_event_raw(self, ois_gyro_data, &event);
			inv_sensor_listener_notify(self->base.listener, &event);
		}
	}

	return 0;
}

int inv_device_icm20690_self_test(void * context, int sensor)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	if(inv_device_icm20690_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	if((sensor != INV_SENSOR_TYPE_RAW_ACCELEROMETER) &&
			(sensor != INV_SENSOR_TYPE_ACCELEROMETER) &&
			(sensor != INV_SENSOR_TYPE_RAW_GYROSCOPE) &&
			(sensor != INV_SENSOR_TYPE_GYROSCOPE) &&
			(sensor != INV_SENSOR_TYPE_RAW_MAGNETOMETER) &&
			(sensor != INV_SENSOR_TYPE_MAGNETOMETER))
		return INV_ERROR_BAD_ARG;

	return (inv_icm20690_run_selftest(&self->basesensor_states) > 0) ? 0 : INV_ERROR;
}

int inv_device_icm20690_whoami(void * context, uint8_t * whoami)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	assert(whoami);

	return inv_icm20690_get_whoami(&self->basesensor_states, whoami);
}

int inv_device_icm20690_reset(void * context)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;
	int rc;
	unsigned i;
	const int sensors[] = {
		INV_SENSOR_TYPE_RAW_ACCELEROMETER,
		INV_SENSOR_TYPE_RAW_GYROSCOPE,
		INV_SENSOR_TYPE_RAW_TEMPERATURE,
		INV_SENSOR_TYPE_RAW_MAGNETOMETER,
		INV_SENSOR_TYPE_OIS,
	};

	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reseting device...");

	/* Stop sensors */
	for(i = 0; i < sizeof(sensors)/sizeof(sensors[0]); ++i) {
		inv_icm20690_enable_sensor(&self->basesensor_states,
				idd_sensortype_2_driver(sensors[i]), 0);
	}

	if((rc = inv_icm20690_soft_reset(&self->basesensor_states)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "icm20690 soft reset returned %d", rc);
		return rc;
	}

	/* Re-init the device */
	inv_device_icm20690_setup(self);

	return 0;
}

int inv_device_icm20690_setup(void * context)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;
	int rc;
	uint8_t whoami;
	const int sensors[] = {
		INV_SENSOR_TYPE_RAW_ACCELEROMETER,
		INV_SENSOR_TYPE_RAW_GYROSCOPE,
		INV_SENSOR_TYPE_RAW_TEMPERATURE,
		INV_SENSOR_TYPE_RAW_MAGNETOMETER,
	};
	unsigned i;

	INV_MSG(INV_MSG_LEVEL_INFO, "Booting up icm20690...");

	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Reading WHOAMI...");
	if((rc = inv_device_icm20690_whoami(self, &whoami)) != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d when reading WHOAMI value", rc);
		return rc;
	}

	if(whoami == 0 || whoami == 0xff) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Unexpected WHOAMI value 0x%x. Aborting setup.", whoami);
		return INV_ERROR;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "WHOAMI value: 0x%x", whoami);
	}

	/* set default power mode */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Putting icm20690 in sleep mode...");
	if((rc = inv_icm20690_initialize(&self->basesensor_states)) != 0)
		goto error;

	/* we should be good to go ! */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "We're good to go !");

	/* set default ODR = 5Hz */
	for(i = 0; i < sizeof(sensors)/sizeof(sensors[0]); ++i) {
		inv_device_icm20690_set_sensor_period_us(context, sensors[i], 200000 /*us*/);
	}
	return 0;
error:
	INV_MSG(INV_MSG_LEVEL_ERROR, "Error %d while setting-up device.", rc);

	return rc;
}

int inv_device_icm20690_cleanup(void * context)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	return inv_icm20690_soft_reset(&self->basesensor_states);
}

int inv_device_icm20690_ping_sensor(void * context, int sensor)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	/* HW sensors */
	if((sensor == INV_SENSOR_TYPE_RAW_ACCELEROMETER) ||
			(sensor == INV_SENSOR_TYPE_RAW_GYROSCOPE) ||
			(sensor == INV_SENSOR_TYPE_RAW_TEMPERATURE) ||
			(sensor == INV_SENSOR_TYPE_WOM)) {
		return 0;
	}
	/* Advanced sensors */
	else if((sensor == INV_SENSOR_TYPE_OIS) || 
			(sensor == INV_SENSOR_TYPE_FSYNC_EVENT)) {
		if(inv_icm20690_is_advanced_features_supported())
			return 0;
	}
	/* Optionnal aux sensors */
	else if(sensor == INV_SENSOR_TYPE_RAW_MAGNETOMETER) {
		if(inv_icm20690_is_compass_registered(&self->basesensor_states))
			return 0;
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_icm20690_enable_sensor(void * context, int sensor, inv_bool_t en)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	if(inv_device_icm20690_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	if(sensor == INV_SENSOR_TYPE_WOM) {
		uint8_t threshold = (en) ? self->wom_threshold : 0;
		return inv_icm20690_configure_accel_wom(&self->basesensor_states, threshold);
	}
	else {
		return inv_icm20690_enable_sensor(&self->basesensor_states,
				idd_sensortype_2_driver(sensor), en);
	}
}

int inv_device_icm20690_set_sensor_period_us(void * context,
		int sensor, uint32_t period)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	if(inv_device_icm20690_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	/* convert period back to ms as this is what expects the driver for now */
	period /= 1000;

	return inv_icm20690_set_sensor_period(&self->basesensor_states,
			idd_sensortype_2_driver(sensor), period);
}

int inv_device_icm20690_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9])
{
	int i;
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	if(inv_device_icm20690_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	if ((sensor == INV_SENSOR_TYPE_MAGNETOMETER) ||
		(sensor == INV_SENSOR_TYPE_RAW_MAGNETOMETER)) {
		for(i = 0; i < 9; ++i)
			self->mounting_matrix_secondary_compass[i] = (int32_t)(matrix[i] * (1 << 30));
	} else {
		for(i = 0; i < 9; ++i)
			self->mounting_matrix[i] = (int32_t)(matrix[i] * (1 << 30));
	}

	return INV_ERROR_SUCCESS;
}


int inv_device_icm20690_get_sensor_config(void * context, int sensor, int setting, void * arg, unsigned size)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	if(inv_device_icm20690_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	switch (setting) {
		case INV_DEVICE_ICM20690_CONFIG_BIAS_ST:
		{
			int st_bias[12];
			int32_t * out = (int32_t *)arg;
			int i;

			if(size < 6*sizeof(int32_t))
				return INV_ERROR_BAD_ARG;

			inv_icm20690_get_st_bias(&self->basesensor_states, st_bias);

			switch (sensor) {
			case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			case INV_SENSOR_TYPE_GYROSCOPE:
				for (i = 0; i < 6; i++)
					out[i] = st_bias[i];
				break;
			case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			case INV_SENSOR_TYPE_ACCELEROMETER:
				for (i = 0; i < 6; i++)
					out[i] = st_bias[i + 6];
				break;
			default:
				return INV_ERROR_BAD_ARG;
			}
			return (6*sizeof(int32_t));
		}
		default:
			return INV_ERROR_BAD_ARG;
	}
}

int inv_device_icm20690_set_sensor_config(void * context, int sensor, int setting, const void * arg, unsigned size)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	if(inv_device_icm20690_ping_sensor(context, sensor) != 0)
		return INV_ERROR_BAD_ARG;

	switch (setting) {
		case INV_DEVICE_ICM20690_CONFIG_MOUNTING_MATRIX:
		{
			const inv_sensor_config_mounting_mtx_t * mtx = (const inv_sensor_config_mounting_mtx_t *)arg;
			return inv_device_icm20690_set_sensor_mounting_matrix(context, sensor, mtx->matrix);
		}
		case INV_SENSOR_CONFIG_FSR:
		{
			const inv_sensor_config_fsr_t * fsr = (const inv_sensor_config_fsr_t *)arg;

			switch(sensor) {
			case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			case INV_SENSOR_TYPE_GYROSCOPE:
				return inv_icm20690_set_gyro_fullscale(&self->basesensor_states,
						inv_icm20690_gyro_fsr_2_reg(fsr->fsr));
			case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			case INV_SENSOR_TYPE_ACCELEROMETER:
				return inv_icm20690_set_accel_fullscale(&self->basesensor_states,
						inv_icm20690_accel_fsr_2_reg(fsr->fsr));
			default:
				break;
			}
			break;
		}
		case INV_DEVICE_ICM20690_CONFIG_WOM_THRESHOLD:
			switch(sensor) {
			case INV_SENSOR_TYPE_WOM:
				self->wom_threshold = (uint8_t)(*(const inv_device_icm20690_config_wom_threshold_t *)arg);
				return 0;
			default:
				break;
			}
			break;
		case INV_DEVICE_ICM20690_CONFIG_BIAS_ST:
		{
			int st_bias[12] = {0};
			const int * in = (const int *) arg;
			int i;

			if(size < 6*sizeof(int32_t))
				return INV_ERROR_BAD_ARG;

			switch(sensor) {
			case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			case INV_SENSOR_TYPE_GYROSCOPE:
				for (i = 0; i < 6; i++)
					st_bias[i] = in[i];
				break;
			case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			case INV_SENSOR_TYPE_ACCELEROMETER:
				for (i = 0; i < 6; i++)
					st_bias[i + 6] = in[i];
				break;
			default:
				return INV_ERROR_BAD_ARG;
			}
			inv_icm20690_set_st_bias(&self->basesensor_states, st_bias);
			return 0;
		}
		default:
			break;
	}

	return INV_ERROR_BAD_ARG;
}

int inv_device_icm20690_write_mems_register(void * context, int sensor, uint16_t reg_addr,
		const void * data, unsigned size)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	(void)sensor;

	return inv_icm20690_write_reg(&self->basesensor_states, (uint8_t)reg_addr, (uint8_t*)data, size);
}

int inv_device_icm20690_read_mems_register(void * context, int sensor, uint16_t reg_addr,
		void * data, unsigned size)
{
	inv_device_icm20690_t * self = (inv_device_icm20690_t *)context;

	(void)sensor;

	return inv_icm20690_read_reg(&self->basesensor_states, (uint8_t)reg_addr, (uint8_t*)data, size);
}
