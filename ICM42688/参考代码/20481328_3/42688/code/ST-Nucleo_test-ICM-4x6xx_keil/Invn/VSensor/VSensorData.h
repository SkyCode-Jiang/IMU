/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2016 InvenSense Inc. All rights reserved.
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

/** @defgroup VSensorData VSensorData
 *  @brief    Definitions of the data published by a VSensor
 *  @ingroup  VSensor
 *  @{
 */

#ifndef _V_SENSOR_DATA_H_
#define _V_SENSOR_DATA_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief Helper type for Q30 fix point integer (1:2^30)
 */
typedef int32_t intq30_t;

/** @brief Helper type for Q16 fix point integer (1:2^15)
 */
typedef int32_t intq16_t;

/** @brief Helper type for Q31 fix point integer (1:2^31)
 */
typedef int32_t intq31_t;

/** @brief Helper type for Q15 fix point integer (1:2^15)
*/
typedef int32_t intq15_t;

/** @brief Base VSensor data structure
 *
 *  Upon NEW_DATA event, event data are expected to point to a VSensorData.
 *
 *  Any data structure should inherit from it.
 */
typedef struct VSensorData {
	uint32_t  timestamp;    /**< data timestamp */
	uint32_t  meta_data;    /**< additionnal status/info regarding the sensor event
	                          * bit0-2:  accuracy flag
	                          * bit3-31: reserved (must be ignored)
	                          */
} VSensorData;

/** @brief Mask to retrieve accuracy value from sensor event meta-data
 */
#define VSENSOR_DATA_ACCURACY_MASK    ((uint32_t)0x7)

/** @brief Value for UNKNOWN accuracy in sensor event meta-data
 */
#define VSENSOR_DATA_ACCURACY_UNKNOWN 0

/** @brief Value for LOW accuracy in sensor event meta-data
 */
#define VSENSOR_DATA_ACCURACY_LOW     1

/** @brief Value for MEDIUM accuracy in sensor event meta-data
 */
#define VSENSOR_DATA_ACCURACY_MEDIUM  2

/** @brief Value for HIGH accuracy in sensor event meta-data
 */
#define VSENSOR_DATA_ACCURACY_HIGH    3

/** @brief Structore to hold any VSensor data
 */
typedef struct VSensorDataAny {
	VSensorData	base;
	/* use a union to ensure correct pointer alignement */
	union {
		uint8_t  u8[64];
		uint16_t u16[32];
		uint32_t u32[16];
	} data;
} VSensorDataAny;

/** @brief Maximum size for VSensor data structure
 */
#define VSENSOR_DATA_SIZE_MAX (sizeof(VSensorDataAny))

/** @brief Data for RAW_ACCELEROMETER, RAW_MAGNETOMETER, RAW_GYROMETER VSensor
 */
typedef struct VSensorDataRaw3d {
	VSensorData base;    /**< base */
	int32_t     x, y, z; /**< x, y, z raw value */
	intq16_t    scale;   /**< theorical scale to convert raw value to SI unit */
} VSensorDataRaw3d;

/** @brief Data for ACCELEROMETER, MAGNETOMETER, GYROMETER VSensor
 *
 *  Also used for GRAVITY and LINEARACC
 *
 *  SI unit are expected to be:
 *   - g's for ACCELEROMETER based sensor (1g = 2^16)
 *   - µT for MAGNETOMETER based sensor   (1uT = 2^16)
 *   - dps for GYROMETER based sensor     (1dps = 2^16)
 *
 *  .bx, by, bz are expected to be filled only for UNCALIBRATED variant.
 *  Accuracy flag value can be retrieved from the .base.meta_data
 */
typedef struct VSensorDataCal3d {
	VSensorData base;    /**< base */
	intq16_t x, y, z;    /**< x, y, z value in SI */
	intq16_t bx, by, bz; /**< x, y, z bias value in SI */
} VSensorDataCal3d;

/*
 * Aliases
 */
typedef VSensorDataCal3d          VSensorDataAccelerometer;
typedef VSensorDataCal3d          VSensorDataGyroscope;
typedef VSensorDataCal3d          VSensorDataMagnetometer;
typedef VSensorDataCal3d          VSensorDataAccelerometerUncal;
typedef VSensorDataCal3d          VSensorDataGyroscopeUncal;
typedef VSensorDataCal3d          VSensorDataMagnetometerUncal;
typedef VSensorDataAccelerometer  VSensorDataLinearAcc;
typedef VSensorDataAccelerometer  VSensorDataGravity;
typedef VSensorDataGyroscope      VSensorDataGyrometer;
typedef VSensorDataGyroscopeUncal VSensorDataGyrometerUncal;

/** @brief Data for GAME_ROTATION_VECTOR, ROTATION_VECTOR,
 *         GEOMAG_ROTATION_VECTOR, ACC_ROTATION_VECTRO, VSensor
 */
typedef struct {
	VSensorData base;       /**< base */
	intq30_t    w, x, y, z; /**< quaternion (scale 1:2^30) */
	intq16_t    accuracy;   /**< heading accuracy in deg (scale 1:2^16) */
} VSensorDataQuaternion;

/** @brief Data for ORIENTATION VSensor
 *
 *  Represents device orientation as angles.
 *  Depending on VSensor type, convention might differ (Google's vs Euler's)
 */
typedef struct {
	VSensorData base;    /**< base */
	intq16_t    x, y, z; /**< orientation around x, y, z axis in deg (scale 1:2^16) */
} VSensorData3dAngles;

/** @brief Data for STEP_COUNTER VSensor
 */
typedef struct {
	VSensorData base;  /**< base */
	int32_t     count; /**< step count */
} VSensorDataStepCount;

/** @brief Data for PROXIMITY VSensor
 */
typedef struct {
	VSensorData base;     /**< base */
	int32_t     distance; /**< distance in mm (scale 1:1) */
} VSensorDataProximity;

/** @brief Data for LIGHT VSensor
 */
typedef struct {
	VSensorData base;  /**< base */
	int32_t     light; /**< light level in lux (scale 1:1) */
} VSensorDataLight;

/** @brief Data for PRESSURE VSensor
 */
typedef struct {
	VSensorData base;     /**< base */
	int32_t     pressure; /**< pressure level in Pa (scale 1:1) */
} VSensorDataPressure;

/** @brief Data for AMBIENT_TEMPERATURE and TEMPERATURE VSensor
 */
typedef struct {
	VSensorData base;        /**< base */
	intq16_t    temperature; /**< temperature in deg celcius (scale 1:2^16) */
} VSensorDataTemperature;

/** @brief Data for HUMIDITY VSensor
 */
typedef struct {
	VSensorData base;     /**< base */
	intq16_t    humidity; /**< humidity level in percent (scale 1:2^16) */
} VSensorDataHumidity;

/** @brief Data for HEART_RATE VSensor
 */
typedef struct {
	VSensorData base; /**< base */
	intq16_t    ppm;  /**< heart rate in pulse per minute (scale 1:2^16) */
} VSensorDataHeartRate;

/** @brief Data for AAR VSensor
 */
typedef struct {
	VSensorData base; /**< base */
	enum VSensorDataAAREvent {
		VSENSOR_DATA_AAR_UNKNOWN          =  0,
		VSENSOR_DATA_AAR_IN_VEHICLE_START =  1,
		VSENSOR_DATA_AAR_IN_VEHICLE_END   = -1,
		VSENSOR_DATA_AAR_WALKING_START    =  2,
		VSENSOR_DATA_AAR_WALKING_END      = -2,
		VSENSOR_DATA_AAR_RUNNING_START    =  3,
		VSENSOR_DATA_AAR_RUNNING_END      = -3,
		VSENSOR_DATA_AAR_ON_BICYCLE_START =  4,
		VSENSOR_DATA_AAR_ON_BICYCLE_END   = -4,
		VSENSOR_DATA_AAR_TILT_START       =  5,
		VSENSOR_DATA_AAR_TILT_END         = -5,
		VSENSOR_DATA_AAR_STILL_START      =  6,
		VSENSOR_DATA_AAR_STILL_END        = -6,
	} event;           /**< aar activity event */
} VSensorDataAAR;

#ifdef __cplusplus
}
#endif

#endif /* _V_SENSOR_DATA_H_ */

/** @} **/
