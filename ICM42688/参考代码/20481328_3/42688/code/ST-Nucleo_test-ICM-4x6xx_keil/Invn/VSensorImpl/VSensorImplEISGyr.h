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

/** @defgroup 	VSensorImplEISGyr	VSensorImplEISGyr
 	@brief 		This Vsensor is responsible for providing sensor outputs for EIS Gyroscope.
    @ingroup 	VSensorImpl
    @{
*/

#ifndef _VSENSOR_IMPL_EIS_GYR_H_
#define _VSENSOR_IMPL_EIS_GYR_H_

#include "Invn/VSensor/VSensor.h"

/** @brief Invn FSYNC event
 */
#define VSENSOR_TYPE_FSYNC_EVENT                (VSENSOR_TYPE_CUSTOM_MAX + 1)

/** @brief Invn EIS sensor
 */
#define VSENSOR_TYPE_EIS                        (VSENSOR_TYPE_CUSTOM_MAX + 2)

#include <string.h>

/** @brief Data for EIS VSensor
 */
typedef struct VSensorDataEIS {
	VSensorDataGyroscope gyr;
	int16_t delta_ts;
} VSensorDataEIS;

/** @brief Get VSensor handle to Orientation sensor
 *  @return pointer to unique instance of EIS Gyroscope VSensor
 */
VSensor * VSensorImplEISGyr_getHandle(void);

/** @brief Initialize states of EIS Gyroscope VSensor
 *  @param[in] eis_event handle to the VSensor that should FSYNC event
 *  @param[in] gyr	     pointer to gyroscope Vsensor
 */
void VSensorImplEISGyr_init(VSensor * eis_event, VSensor *gyr);

#endif	/* _VSENSOR_IMPL_EIS_GYR_H_ */

/** @} */

