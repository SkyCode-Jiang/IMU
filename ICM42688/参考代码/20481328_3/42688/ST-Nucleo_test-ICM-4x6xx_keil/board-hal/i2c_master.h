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


/** @defgroup I2C master peripheral
	@ingroup  Driver
	@{
*/
#ifndef __I2C_MASTER_H__
#define __I2C_MASTER_H__

/** @brief Configures I2C master peripheral
*/
void i2c_master_init(void);

#define ICM_I2C_ADDR     0x68

/** @brief Desactivates the I2C master peripheral
*/
void i2c_master_deinit(void);

/** @brief Read a register through the control interface I2C 
* @param[in] address, I2c 7bit-address
* @param[in] register_addr, register address (location) to access
* @param[in] register_len, length value to read
* @param[in] register_value, pointer on byte value to read
* @retval 0 if correct communication, else wrong communication
*/
unsigned long i2c_master_read_register(unsigned char address, unsigned char register_addr, 
                                          unsigned short register_len, unsigned char *register_value);

/** @brief Write a register through the control interface I2C  
* @param[in] address, I2c 7bit-address
* @param[in] register_addr, register address (location) to access
* @param[in] register_len, length value to write
* @param[in] register_value, pointer on byte value to write
* @retval 0 if correct communication, else wrong communication
*/
unsigned long i2c_master_write_register(unsigned char address, unsigned char register_addr, 
                                           unsigned short register_len, const unsigned char *register_value);
//--yd 
int i2c_write_1B(unsigned short offset,unsigned char data);
int i2c_read_1B(unsigned short offset, unsigned char * data);
int i2c_read(unsigned i2c_addr, unsigned short offset, unsigned char * data, unsigned short len);
int i2c_read_raw(unsigned char slave_addr, unsigned char *data,unsigned short length);
int i2c_write_raw(unsigned char slave_addr, unsigned char *data,unsigned short length);
unsigned long i2c_master_read_raw(unsigned char Address, unsigned short RegisterLen, unsigned char *RegisterValue);
unsigned long i2c_master_write_raw(unsigned char Address, unsigned short RegisterLen, const unsigned char *RegisterValue);

#endif /* __I2C_MASTER_H__ */


