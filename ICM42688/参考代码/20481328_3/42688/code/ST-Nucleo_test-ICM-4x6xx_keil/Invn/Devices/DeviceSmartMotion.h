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

/** @defgroup DeviceSmartMotion DeviceSmartMotion
 *	@brief    Device implementation that connects to other IDD devices and emulates missing motion features
 *            using software libraries.
 *
 *            It uses the InvenSense VSensor framework to connect a HW device to a InvenSense CModel
 *            processing graph.
 *
 *            The graph consists in:
 *             - Few roots that get data from underlying IDD device
 *             - Leaves for all emulated sensors 
 *             - Intermediary nodes that call SmartMotion library
 *
 *            Upon SmartMotion device initialization, you must provide a reference to an already intialiazed
 *            and setup inv_device_t (eg: ICM20602).
 *
 *            This device implementation takes ownership of ther underlying device.
 *            Except for very specific methods (eg: setup(), load(), self_test(), ...), handle to SmartMotion
 *            device should be used to control sensors.
 *
 *            When calling setup(), SmartMotion device will ping all sensors for the underlying HW device
 *            and keep track their availibitly.
 *
 *            When starting a sensor, if is is present at HW level, SmartMotion device will forward the request
 *            to the underlying device. If not, it will enable the corresponding algorithm and start required
 *            base sensor to provide requested data. Eg: if user requests 'GAME_ROTATION_VECTOR' and it
 *            is not directly available from the HW device, both 'RAW_GYROSCOPE' and 'RAW_ACCELEROMETER'
 *            will be started at HW level, and 'GAME_ROTATION_VECTOR' algorithm enabled.
 *
 *            SmartMotion device only emulates the following sensors:
 *             - INV_SENSOR_TYPE_ACCELEROMETER
 *             - INV_SENSOR_TYPE_GYROSCOPE
 *             - INV_SENSOR_TYPE_UNCAL_GYROSCOPE
 *             - INV_SENSOR_TYPE_MAGNETOMETER
 *             - INV_SENSOR_TYPE_UNCAL_MAGNETOMETER
 *             - INV_SENSOR_TYPE_GAME_ROTATION_VECTOR
 *             - INV_SENSOR_TYPE_ROTATION_VECTOR
 *             - INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR
 *             - INV_SENSOR_TYPE_BAC
 *             - INV_SENSOR_TYPE_STEP_COUNTER
 *             - INV_SENSOR_TYPE_STEP_DETECTOR
 *             - INV_SENSOR_TYPE_SMD
 *             - INV_SENSOR_TYPE_TILT_DETECTOR
 *             - INV_SENSOR_TYPE_PICK_UP_GESTURE
 *             - INV_SENSOR_TYPE_GRAVITY
 *             - INV_SENSOR_TYPE_LINEAR_ACCELERATION
 *             - INV_SENSOR_TYPE_ORIENTATION
 *
 *            Required base sensor are:
 *             - INV_SENSOR_TYPE_RAW_ACCELEROMETER
 *             - INV_SENSOR_TYPE_RAW_GYROSCOPE
 *             - INV_SENSOR_TYPE_RAW_MAGNETOMETER
 *
 *            @warning Batching is currently not emulated
 *
 *            @warning Only one instance of this device can be used in the same application
 *                     SmartMotion algorithms are currently integrated using static variables.
 *                     Only one instance of this device must be used at once, to avoid re-entrancy issue.
 *
 *  @ingroup  Device
 *	@{
 */

#ifndef _INV_DEVICE_SMART_MOTION_H_
#define _INV_DEVICE_SMART_MOTION_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/Device.h"
#include "Invn/VSensor/VSensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Internal definition for graph leaves (based on VSensorListener)
 */
struct inv_device_smart_motion_vlistener {
	VSensorListener vlistener;                               /**< base VSensorListener states */
	void (*build_event)(const void *, inv_sensor_event_t *); /**< callback to convert VSensorData
	                                                              to IDD sensor event */
	uint8_t idd_type;                                        /**< corresponding IDD sensor type */
	InvList node;                                            /**< pointer to next element when stored in a list */
};

/** @brief Internal definition for graph roots (based on VSensor)
 */
struct inv_device_smart_motion_vsensor {
	VSensor vsensor;                                                /**< base VSensor states */
	void (*build_vsensor_data)(const inv_sensor_event_t *, void *); /**< callback to convert 
	                                                                     IDD sensor event to VSensorData */
	uint8_t idd_type;                                               /**< corresponding IDD sensor type */
};

/** @brief States for SmartMotion device implementation
 */
typedef struct inv_device_smart_motion
{
	inv_device_t          base;                 /**< base inv_device_t states */
	inv_device_t *        hw_device;            /**< reference to inv_device_t that will provides
	                                                 sensor data for base sensor */
	inv_sensor_listener_t private_idd_listener; /**< internal IDD listener to catch events from HW device */
	InvList               leaves_list;          /**< list of graph leaves (corresponds to sensors not
	                                                 present at hardware level and that are emulated) */
	struct {
		struct inv_device_smart_motion_vsensor  s;    /**< placeholder for roots VSensor */
		struct inv_device_smart_motion_vlistener l;   /**< placeholder for leaves corresponding
		                                             to root VSensor */
	} graph_roots[4];                           /**< placehold holder for graph roots objects */
	struct {
		struct inv_device_smart_motion_vlistener l;   /**< placeholder for leaves VSensorListener */
	} graph_leaves[24];                          /**< placeholder for leaves corresponding to emulated sensors */
	uint64_t hw_sensor_available_mask;          /**< mask to keep track of HW available sensor from underlying device */
} inv_device_smart_motion_t;

/** @brief Allowed config settings for DeviceSmartMotion
 */
enum inv_smartmotion_config_setting {
	INV_SMARTMOTION_CONFIG_GAIN = INV_SENSOR_CONFIG_GAIN,
	INV_SMARTMOTION_CONFIG_SENSITIVITY = INV_SENSOR_CONFIG_CUSTOM, /**< sensitivity (LSB to SI unit factor) */
	INV_SMARTMOTION_RV_THRESH_GYR_STOP_CONVERG,
	INV_SMARTMOTION_RV_THRESH_CAL_GYR,
};

/** @brief Expected type for SENSITIVITY config settings
 *  Value is coded in Q16 and expressed SI unit / LSB
 *  SI unit being g for ACC, uT for MAG and dps for GYR
 */
typedef uint32_t inv_smartmotion_config_value_sensitivity_t;

/** @brief Initialize states for SmartMotion device implementation
 *
 *  Assumes underlying device is already initialized.
 *  Will take ownership of the underlying device and re-use its listener.
 *  No HW access are done in this function.
 *
 *  @param[in] self      handle to current device
 *  @param[in] hw_device reference to inv_device_t used to retrieve base data
 */
void INV_EXPORT inv_device_smart_motion_init(inv_device_smart_motion_t * self, 
		inv_device_t * hw_device);

/** @brief Return generic handle to base inv_device_t
 */
static inline inv_device_t * inv_device_smart_motion_get_base(inv_device_smart_motion_t * self)
{
	return (self) ? &self->base : 0;
}

int INV_EXPORT inv_device_smart_motion_reset(void * context);

/** @brief Initialize SmartMotion graph
 *
 *  Will perform HW access through underlying device (eg: calling ping()).
 *  Underlying device should be setup and all FW/images loaded before calling this method
 */
int INV_EXPORT inv_device_smart_motion_setup(void * context);

/** @brief Initialize SmartMotion graph
 *
 *  Will perform HW access through underlying device (eg: calling ping()).
 *  Underlying device should be setup and all FW/images loaded before calling this method
 */
int INV_EXPORT inv_device_smart_motion_cleanup(void * context);

int INV_EXPORT inv_device_smart_motion_ping_sensor(void * context, int sensor);

int INV_EXPORT inv_device_smart_motion_enable_sensor(void * context, int sensor, inv_bool_t en);

int INV_EXPORT inv_device_smart_motion_set_sensor_period(void * context,
		int sensor, uint32_t period);

int INV_EXPORT inv_device_smart_motion_flush_sensor(void * context, int sensor);

int INV_EXPORT inv_device_smart_motion_set_sensor_bias(void * context, int sensor,
		const float bias[3]);

int INV_EXPORT inv_device_smart_motion_get_sensor_bias(void * context, int sensor,
		float bias[3]);

int INV_EXPORT inv_device_smart_motion_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9]);

int INV_EXPORT inv_device_smart_motion_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event);

int INV_EXPORT inv_device_smart_motion_set_sensor_config(void * context, int sensor, int setting,
		const void * value, unsigned size);

int INV_EXPORT inv_device_smart_motion_poll(void * context);

int INV_EXPORT inv_device_smart_motion_self_test(void * context, int sensor);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_SMART_MOTION_H_ */
