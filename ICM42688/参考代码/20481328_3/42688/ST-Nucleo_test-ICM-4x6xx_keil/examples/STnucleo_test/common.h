/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
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

/** @defgroup common common
 *  @brief    Implementation of API shared by several modules
 *
 *	@ingroup  Low_Level_Driver
 *  @{
*/

#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>

/**
  * @brief  Disable core interrupts while supporting nested interrupts.
  * To fully support nesting, interrupts must be enabled with enable_irq() function.
  */
void disable_irq(void);

/**
  * @brief  Enable core interrupts while supporting nested interrupts.
  * To fully support nesting, interrupts must be enabled with disable_irq() function.
  */
void enable_irq(void);

/**
  * @brief  Configure source clock to be from an external source
  *
  * This function will not do anything if the clock was already coming
  * from an external source.
  * Since it performs clock source changes, it shall be used after coming
  * out of the reset handler.
  * @note
  * This function shall not be called on systems where sourcing clock from
  * an external source is not possible.
  * @return 0 on success, negative error code otherwise
  */
int force_source_clock_ext(void);

#endif //__COMMON_H__

/** @} */
