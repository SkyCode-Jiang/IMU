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
/** @defgroup	DriverIcm20690AuxTransport Icm20690 secondary driver transport
 *  @brief      Low-level Icm20690 secondary interface access
 *  @ingroup 	DriverIcm20690
 *  @{
*/
#ifndef _INV_ICM20690_AUX_TRANSPORT_H_
#define _INV_ICM20690_AUX_TRANSPORT_H_

#ifdef __cplusplus
extern "C" {
#endif

/* forward declaration */
struct inv_icm20690;

/** @brief Secondary I2C channel usage :
 * - channel 0 is reserved for compass reading data
 * - channel 1 is reserved for compass writing one-shot acquisition register
 */
#define COMPASS_I2C_SLV_READ		0
#define COMPASS_I2C_SLV_WRITE		1

/** @brief Initializes the register for the i2c communication
 */
void inv_icm20690_init_secondary(struct inv_icm20690 * s);

/** @brief Reads data in i2c a secondary device
 *  @param[in] index  The i2c slave what you would use 
 *  @param[in] addr   I2c address slave of the secondary slave
 *  @param[in] reg    the register to be read on the secondary device
 *  @param[in] len    Size of data to be read
 *  @return 0 in case of success, -1 for any error
 */
int inv_icm20690_read_secondary(struct inv_icm20690 * s, int index, unsigned char addr, unsigned char reg, char len);

/** @brief Reads data in i2c a secondary device directly 
 *  @param[in] index  The i2c slave what you would use 
 *  @param[in] addr   i2c address slave of the secondary slave
 *  @param[in] reg    the register to be read on the secondary device
 *  @param[in] len    Size of data to be read
 *  @param[out] d     pointer to the data to be read
 *  @return 0 in case of success, -1 for any error
 */
int inv_icm20690_execute_read_secondary(struct inv_icm20690 * s, int index, unsigned char addr, int reg, int len, unsigned char *d);

/** @brief Writes data in i2c a secondary device
 *  @param[in] index The i2c slave what you would use 
 *  @param[in] addr  i2c address slave of the secondary slave
 *  @param[in] reg   the register to be write on the secondary device
 *  @param[in] v     the data to be written
 *  @return 0 in case of success, -1 for any error
 */
int inv_icm20690_write_secondary(struct inv_icm20690 * s, int index, unsigned char addr, unsigned char reg, char v);

/** @brief Writes data in i2c a secondary device directly
 *  @param[in] index The i2c slave what you would use 
 *  @param[in] addr  i2c address slave of the secondary slave
 *  @param[in] reg   the register to be write on the secondary device
 *  @param[in] v      the data to be written
 *  @return 0 in case of success, -1 for any error
 */
int inv_icm20690_execute_write_secondary(struct inv_icm20690 * s, int index, unsigned char addr, int reg, unsigned char v);

/** @brief Stop one secondary I2C channel by writing 0 in its control register
 *  @param[in] index  	the channel id to be stopped
 *  @return 	   		0 in case of success, -1 for any error
 *  @warning It does not stop I2C secondary interface, just one channel
 */
int inv_icm20690_secondary_stop_channel(struct inv_icm20690 * s, int index);

/** @brief Enable secondary I2C interface
 *  @return 0 in case of success, -1 for any error
 */
int inv_icm20690_secondary_enable_i2c(struct inv_icm20690 * s);

/** @brief Stop secondary I2C interface
 *  @return  0 in case of success, -1 for any error
 *  @warning It stops all I2C transactions, whatever the channel status
 */
int inv_icm20690_secondary_disable_i2c(struct inv_icm20690 * s);

#ifdef __cplusplus
}
#endif

#endif // _INV_ICM20690_AUX_TRANSPORT_H_

/** @} */
