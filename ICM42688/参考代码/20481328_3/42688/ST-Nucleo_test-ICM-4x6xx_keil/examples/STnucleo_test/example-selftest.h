/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
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
#ifndef _EXAMPLE_SELFTEST_H_
#define _EXAMPLE_SELFTEST_H_

#include <stdint.h>
/*#include "Invn/Drivers/Icm406xx/Icm406xxTransport.h"
#include "Invn/Drivers/Icm406xx/Icm406xxDefs.h"
#include "Invn/Drivers/Icm406xx/Icm406xxDriver_HL.h"
#include "Invn/Drivers/Icm406xx/Icm406xxSelfTest.h"*/
#include "Icm406xxTransport.h"
#include "Icm406xxDefs.h"
#include "Icm406xxDriver_HL.h"
#include "Icm406xxSelfTest.h"

/**
 * \brief This function is in charge of reseting and initializing Icm406xx device. It should
 * be succesfully executed before any access to Icm406xx device.
 *
 * It builds a serial interface structure that describes the interface to be used to
 * communicate with Icm406xx. Then it calls initialization function of Icm406xx driver.
 * Once successfully executed it reads Icm406xx whoami register to check if serial link 
 * is working properly.
 * 
 * \param[in] read_reg : pointer on a function that reads on the requested Icm406xx serial
 *                      interface.
 * \param[in] write_reg : pointer on a function that writes on the requested Icm406xx 
 *                       serial interface.
 * \param[in] serif_type : type of serial interface requested to access Icm406xx. Enum is
 *                        defined in Icm406xxTransport.h
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(int (*read_reg)(void * context, uint8_t reg, uint8_t * buf, uint32_t len),
				   int (*write_reg)(void * context, uint8_t reg, const uint8_t * buf, uint32_t len),
				   ICM406XX_SERIAL_IF_TYPE_t serif_type);

/*!
 * \brief Run Self Test on Invensense device
 */
void RunSelfTest(void);

/*!
 * \brief Get Bias values calculated from selftest
 */
void GetBias(void);


#endif /* !_EXAMPLE_SELFTEST_H_ */
