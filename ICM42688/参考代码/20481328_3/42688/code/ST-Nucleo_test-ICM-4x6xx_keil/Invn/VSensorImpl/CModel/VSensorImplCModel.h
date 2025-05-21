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

/** @defgroup 	VSensorImplCModel	VSensorImplCModel
 	@brief 		This Vsensor is responsible for providing sensor outputs for Cmodel.
	
				\e Outputs:
				- <b>Calibrated Accelerometer</b> in m/s^2
				- <b>Calibrated Gyroscope</b> in rad/s
				- <b>Calibrated Magnetometer</b> in µT
				- <b>Game Rotation Vector</b>  orientation of the device based on accelerometer and gyroscope
				- <b>Rotation Vector</b>  orientation of the device based on accelerometer, gyroscope and magnetometer
				- <b>GeoMagRotation Vector</b>  orientation of the device based on accelerometer and magnetometer
				- <b>AAR</b>  Advanced Activity Recognition : BAC, StepDetector, StepCounter, SMD and Tilt
				\e Supported commands:
					- <b>Get/Set config</b> to configure sensor
    @ingroup 	VSensorImpl
    @{
*/

#ifndef _VSENSOR_IMPL_CMODEL_H_
#define _VSENSOR_IMPL_CMODEL_H_

#include "Invn/VSensor/VSensor.h"

#include <string.h>

/** @brief Specific VSensor configuration for VSensorImplCModel
 */
enum VSensorImplCModelConfigType {
	VSENSOR_IMPL_CMODEL_CONFIG_TYPE_SENSITIVITY = VSENSOR_CONFIG_TYPE_CUSTOM,
	VSENSOR_IMPL_CMODEL_CONFIG_TYPE_RV_THRESHOLD_GYR_STOP_CONVERGENCE,
	VSENSOR_IMPL_CMODEL_CONFIG_TYPE_CAL_GYR,
};

/** @brief Configuration data for SENSITIVITY identifier
 */
typedef struct VSensorImplCModelConfigSensitivity
{
	VSensorConfig base;        /**< base */
	int32_t       sensitivity; /**< sensitivity to convert from LSB to SI unit */
} VSensorImplCModelConfigSensitivity;

/** @brief Configuration data for RV_THRESHOLD_GYR_STOP_CONVERGENCE identifier
 */
typedef struct VSensorImplCModelConfigRvThreshGyrStop
{
	VSensorConfig base;               /**< base */
	int32_t       rv_thresh_gyr_stop; /**< RV threshold */
} VSensorImplCModelConfigRvThreshGyrStop;


/** @brief Configuration data for CAL_GYR identifier
 */
typedef struct VSensorImplCModelConfigGyrCal
{
	VSensorConfig base;         /**< base */
	int32_t sampleNum_log2;     /**< Log2 of number of samples to compute the bias,
	                                 e.g. when sampleNum_log2 = 6, number of samples is 2^6=64 samples.
	                                 Default value is 7. */
	int32_t	data_diff_thrsh;    /**< Threshold that detect motion.
	                                 Restart algorithm when the difference 2 consequetive samples exceed the threshold.
	                                 Default value is 38. */
	int32_t	fnm_sampleNum_log2; /**< Log2 of number of samples to detect Fast No Motion (FNM),
	                                 e.g. when sampleNum_log2 = 7, number of samples is 2^7=128 samples.
	                                 Default value is 9. */
	int32_t	fnm_moment_thrsh;   /**< Threshold that detect motion.
	                                 Restart algorithm when the high order variance exceed the threshold.
	                                 Default value is 100. */
	int32_t	fnm_data_abs_thrsh; /**< Threshold that detect motion.
	                                 Reject bias higher than the threshold.
	                                 Default value is 40 dps => 40*(2^15/2000)*2^15. */
} VSensorImplCModelConfigGyrCal;

/** @brief Initialise CalAccVsensor
 *  @param[in] raw acc	pointer to raw accelerometer Vsensor
 */
void VSensorImplCModelCalAcc_init(VSensor * raw_acc);

/** @brief Get VSensor CalAcc handle
 *  @return pointer to VSensor CalAcc
 */
VSensor * VSensorImplCModelCalAcc_getHandle(void);

/** @brief Initialise CalGyrVsensor
 *  @param[in] raw gyr	pointer to raw gyroscope Vsensor
 */
void VSensorImplCModelCalGyr_init(VSensor * raw_gyr);

/** @brief Get VSensor CalGyr handle
 *  @return pointer to VSensor CalGyr
 */
VSensor * VSensorImplCModelCalGyr_getHandle(void);

/** @brief Initialise UncalGyrVsensor
 *  @param[in] gyr	pointer to gyroscope Vsensor
 */
void VSensorImplCModelUnCalGyr_init(VSensor * gyr);

/** @brief Get VSensor UnCalGyr handle
 *  @return pointer to VSensor UnCalGyr
 */
VSensor * VSensorImplCModelUnCalGyr_getHandle(void);

/** @brief Initialise CalMagVsensor
 *  @param[in] raw mag	pointer to raw mag Vsensor
 */
void VSensorImplCModelCalMag_init(VSensor * raw_mag);

/** @brief Get VSensor CalMag handle
 *  @return pointer to VSensor CalMag
 */
VSensor * VSensorImplCModelCalMag_getHandle(void);

/** @brief Initialise UnCalMagVsensor
 *  @param[in] raw mag	pointer to raw mag Vsensor
 */
void VSensorImplCModelUnCalMag_init(VSensor * raw_mag);

/** @brief Get VSensor UnCalMag handle
 *  @return pointer to VSensor UnCalMag
 */
VSensor * VSensorImplCModelUnCalMag_getHandle(void);

/** @brief Initialise GRV
 *  @param[in] gyr	pointer to gyroscope Vsensor
 *  @param[in] acc	pointer to accelerometer Vsensor
 */
void VSensorImplCModelGRV_init(VSensor * gyr, VSensor * acc);

/** @brief Get VSensor GRV handle
 *  @return pointer to VSensor GRV
 */
VSensor * VSensorImplCModelGRV_getHandle(void);

/** @brief Initialise RV
 *  @param[in] acc	pointer to accelerometer Vsensor
 *  @param[in] gyr	pointer to gyroscope Vsensor
 *  @param[in] mag	pointer to magnetometer Vsensor
 */
void VSensorImplCModelRV_init(VSensor * acc, VSensor * gyr, VSensor * mag);

/** @brief Get VSensor RV handle
 *  @return pointer to VSensor RV
 */
VSensor * VSensorImplCModelRV_getHandle(void);

/** @brief Initialise GeoRV
 *  @param[in] acc	pointer to accelerometer Vsensor
 *  @param[in] mag	pointer to magnetometer Vsensor
 */
void VSensorImplCModelGeoRV_init(VSensor * acc, VSensor * mag);

/** @brief Get VSensor GeoRV handle
 *  @return pointer to VSensor GeoRV
 */
VSensor * VSensorImplCModelGeoRV_getHandle(void);

/** @brief Initialise AAR
 *  @param[in] acc	pointer to accelerometer Vsensor
 */
void VSensorImplCModelAAR_init(VSensor * acc);

/** @brief Get VSensor AAR handle
 *  @return pointer to VSensor AAR
 */
VSensor * VSensorImplCModelAAR_getHandleAAR(void);

/** @brief Get VSensor StepC handle
 *  @return pointer to VSensor StepC
 */
VSensor * VSensorImplCModelAAR_getHandleStepC(void);

/** @brief Get VSensor StepD handle
 *  @return pointer to VSensor StepD
 */
VSensor * VSensorImplCModelAAR_getHandleStepD(void);

/** @brief Get VSensor SMD handle
 *  @return pointer to VSensor SMD
 */
VSensor * VSensorImplCModelAAR_getHandleSMD(void);

/** @brief Get VSensor Tilt handle
 *  @return pointer to VSensor Tilt
 */
VSensor * VSensorImplCModelAAR_getHandleTilt(void);

/** @brief Initialise PickUp
 *  @param[in] acc	pointer to accelerometer Vsensor
 */
void VSensorImplCModelPickUp_init(VSensor * acc);

/** @brief Get VSensor PickUp handle
 *  @return pointer to VSensor PickUp
 */
VSensor * VSensorImplCModelPickUp_getHandle(void);

/** @brief Get VSensor handle to Gravity sensor
 *  @return pointer to unique instance of CModel Gravity VSensor
 */
VSensor * VSensorImplCModelGravity_getHandle(void);

/** @brief Initialize states of CModel Gravity VSensor
 *  @param[in] grv handle to the VSensor that should provide GRV data
 */
void VSensorImplCModelGravity_init(VSensor * grv);

/** @brief Get VSensor handle to Linear Acc sensor
 *  @return pointer to unique instance of CModel Linear Acc VSensor
 */
VSensor * VSensorImplCModelLinearAcc_getHandle(void);

/** @brief Initialize states of CModel Linear Acceleration VSensor
 *  @param[in] gra handle to the VSensor that should provide gravity data
 *  @param[in] acc handle to the VSensor that should provide accelerometer data
 */
void VSensorImplCModelLinearAcc_init(VSensor * gra, VSensor * acc);

/** @brief Get VSensor handle to Orientation sensor
 *  @return pointer to unique instance of CModel Orientatio VSensor
 */
VSensor * VSensorImplCModelOrientation_getHandle(void);

/** @brief Initialize states of CModel Orientation VSensor
 *  @param[in] ori handle to the VSensor that should provide quaternion data (usualy ROTATION VECTOR)
 */
void VSensorImplCModelOrientation_init(VSensor * ori);

#endif	/* _VSENSOR_IMPL_CMODEL_H_ */

/** @} */

