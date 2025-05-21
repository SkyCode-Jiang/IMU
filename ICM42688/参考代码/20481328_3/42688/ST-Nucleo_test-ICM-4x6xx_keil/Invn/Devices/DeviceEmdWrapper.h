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

/** @defgroup DeviceEmdWrapper DeviceEmdWrapper
 *	@brief    emd wrapper implementation for device interface
 *  @ingroup  Device
 *	@{
 */

#ifndef _INV_DEVICE_EMD_WRAPPER_H_
#define _INV_DEVICE_EMD_WRAPPER_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/Device.h"

#include "Invn/Devices/Drivers/IddWrapper/IddWrapperTransportUart.h"
#include "Invn/Devices/Drivers/IddWrapper/IddWrapperProtocol.h"

#ifdef __cplusplus
extern "C" {
#endif

extern void inv_sleep_ms(unsigned ms);

typedef struct inv_device_emd_wrapper
{
	inv_device_t			base;

	int (*serial_write_cb)(const uint8_t * data, unsigned len, void * cookie);
	int (*serial_read_cb)(uint8_t * data, unsigned len, void * cookie);
	uint8_t * serial_read_buf;
	uint32_t serial_read_buf_size;
	uint8_t * serial_write_buf;
	uint32_t serial_write_buf_size;
	uint32_t serial_write_buf_cnt;
	void * serial_cookie;

	inv_iddwrapper_protocol_t  iddwrapper_protocol;
	inv_iddwrapper_transport_uart_t iddwrapper_transport;

	union {
		int rc;
	} response_data;
	inv_bool_t response_event;

} inv_device_emd_wrapper_t;


struct inv_device_emd_wrapper_serial {
	int (*serial_read_cb)(uint8_t * data, unsigned len, void * cookie);
	uint8_t * serial_read_buf;
	uint32_t serial_read_buf_size;
	int (*serial_write_cb)(const uint8_t * data, unsigned len, void * cookie);
	uint8_t * serial_write_buf;
	uint32_t serial_write_buf_size;
};

/** @brief constructor-like function for IDDWrapper device
 */
void INV_EXPORT inv_device_emd_wrapper_init(inv_device_emd_wrapper_t * self,
	const inv_sensor_listener_t * listener,
	const struct inv_device_emd_wrapper_serial * serial, void * serial_cookie
);

static inline inv_device_t * inv_device_emd_wrapper_get_base(inv_device_emd_wrapper_t * self)
{
	if(self)
		return &self->base;

	return 0;
}

int INV_EXPORT inv_device_emd_wrapper_is_hw_handshake_supported(inv_device_emd_wrapper_t * self);

int INV_EXPORT inv_device_emd_wrapper_whoami(void * context, uint8_t * whoami);

int INV_EXPORT inv_device_emd_wrapper_reset(void * context);

int INV_EXPORT inv_device_emd_wrapper_setup(void * context);

int INV_EXPORT inv_device_emd_wrapper_cleanup(void * context);

int INV_EXPORT inv_device_emd_wrapper_load(void * context, int what,
		const uint8_t * image, uint32_t size, inv_bool_t verify, inv_bool_t force);

int INV_EXPORT inv_device_emd_wrapper_get_fw_info(void * context,
		struct inv_fw_version * version);

int INV_EXPORT inv_device_emd_wrapper_set_running_state(void * context, inv_bool_t state);

int INV_EXPORT inv_device_emd_wrapper_ping_sensor(void * context, int sensor);

int INV_EXPORT inv_device_emd_wrapper_enable_sensor(void * context, int sensor, inv_bool_t en);

int INV_EXPORT inv_device_emd_wrapper_set_sensor_period_us(void * context,
		int sensor, uint32_t period);

int INV_EXPORT inv_device_emd_wrapper_set_sensor_timeout(void * context,
		int sensor, uint32_t timeout);

int INV_EXPORT inv_device_emd_wrapper_flush_sensor(void * context, int sensor);

int INV_EXPORT inv_device_emd_wrapper_set_sensor_bias(void * context, int sensor,
		const float bias[3]);

int INV_EXPORT inv_device_emd_wrapper_get_sensor_bias(void * context, int sensor,
		float bias[3]);

int INV_EXPORT inv_device_emd_wrapper_set_sensor_mounting_matrix(void * context,
		int sensor, const float matrix[9]);

int INV_EXPORT inv_device_emd_wrapper_get_sensor_data(void * context, int sensor,
		inv_sensor_event_t * event);

int INV_EXPORT inv_device_emd_wrapper_poll(void * context);

int INV_EXPORT inv_device_emd_wrapper_self_test(void * context, int sensor);

#ifdef __cplusplus
}
#endif

#endif /* _INV_DEVICE_EMD_WRAPPER_H_ */
