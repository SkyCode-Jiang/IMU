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

#include "DeviceEmdWrapper.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "Invn/Utils/Message.h"

static const inv_device_vt_t device_emd_wrapper_vt = {
	inv_device_emd_wrapper_whoami,
	inv_device_emd_wrapper_reset,
	inv_device_emd_wrapper_setup,
	inv_device_emd_wrapper_cleanup,
	0,
	inv_device_emd_wrapper_poll,
	inv_device_emd_wrapper_self_test,
	0,
	inv_device_emd_wrapper_ping_sensor,
	0,
	inv_device_emd_wrapper_enable_sensor,
	inv_device_emd_wrapper_set_sensor_period_us,
	inv_device_emd_wrapper_set_sensor_timeout,
	inv_device_emd_wrapper_flush_sensor,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

/* Handle event from IddWrapper Transport layer
   - Feed IddWrapper Protocol to process incomming packet
   - Forward IddWrapper Protocol buffers after encapulsation by Transport layer
     to SERIAL link
*/
static void iddwrapper_transport_event_cb(enum inv_iddwrapper_transport_event e,
	union inv_iddwrapper_transport_event_data data, void * cookie)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)cookie;
	int rc;

	switch(e) {
	/* feed IddWrapper Protocol with bytes receveives from the IddWrapper Transport */
	case INV_IDDWRAPPER_TRANSPORT_EVENT_ERROR:
		INV_MSG(INV_MSG_LEVEL_VERBOSE, "DeviceEmdWrapper: ERROR event with value %d received from IddWrapper transport", data.error);
		break;
	case INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_SIZE:
		// INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: PKT_SIZE event with value %d received from IddWrapper transport", data.pkt_size);
		break;
	case INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_BYTE:
		/* Feed IddWrapperProtocol to look for packet */
		rc = inv_iddwrapper_protocol_process_pkt_byte(&self->iddwrapper_protocol, data.pkt_byte);
		if(rc < 0) {
			/* If there was an error in processing a byte in the idd_wrapper_protocol, drop the current frame */
			inv_iddwrapper_transport_uart_rx_process_reset(&self->iddwrapper_transport);
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "DeviceEmdWrapper: inv_iddwrapper_protocol_process_pkt_byte(%02x) returned %d", data.pkt_byte, rc);
		}
		break;
	case INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_END:
		// INV_MSG(INV_MSG_LEVEL_DEBUG, "DeviceEmdWrapper: PKT_END event received from IddWrapper transport");
		break;

	/* forward buffer from IddWrapper Transport, to the SERIAL */
	case INV_IDDWRAPPER_TRANSPORT_EVENT_TX_START:
		self->serial_write_buf_cnt = 0;
		break;
	case INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE:
		self->serial_write_buf[self->serial_write_buf_cnt++] = data.tx_byte;
		/* If buffer is full, write what we have now */
		if(self->serial_write_buf_cnt == self->serial_write_buf_size) {
			self->serial_write_cb(self->serial_write_buf, self->serial_write_buf_cnt, self->serial_cookie);
			self->serial_write_buf_cnt = 0;
		}
		break;
	case INV_IDDWRAPPER_TRANSPORT_EVENT_TX_END:
		/* Do send byte of the SERIAL link */
		if(self->serial_write_buf_cnt) {
			self->serial_write_cb(self->serial_write_buf, self->serial_write_buf_cnt, self->serial_cookie);
		}
		break;
	}
}

static void handle_response(inv_device_emd_wrapper_t * self,
		enum inv_iddwrapper_protocol_eid eid,
		const inv_iddwrapper_protocol_edata_t * edata
)
{
	switch(eid) {
	case INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I:
	case INV_IDDWRAPPER_PROTOCOL_EID_RESET:
	case INV_IDDWRAPPER_PROTOCOL_EID_SETUP:
	case INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP:
	case INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST:
	case INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD:
	case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
	case INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG:
		self->response_data.rc = edata->d.response.rc;
		self->response_event = 1;
		break;
	default:
		break;
	}
}

static void handle_async(inv_device_emd_wrapper_t * self,
		enum inv_iddwrapper_protocol_eid eid,
		const inv_iddwrapper_protocol_edata_t * edata
)
{
	switch(eid) {
	case INV_IDDWRAPPER_PROTOCOL_EID_NEW_SENSOR_DATA:
		/* notify the world */
		inv_sensor_listener_notify(self->base.listener, &edata->d.async.sensor_event);
		break;
	default:
		break;
	}
}

/* Handle and dispatch events from IddWrapper Protocol layer
   - Call IDD sensor event listeners for async sensor data
   - Copy response data to internal states
   - Ignore command events as this layer is not supposed to receive any
*/
static void iddwrapper_protocol_event_cb(
	enum inv_iddwrapper_protocol_etype etype,
	enum inv_iddwrapper_protocol_eid eid,
	const inv_iddwrapper_protocol_edata_t * edata,
	void * cookie
)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)cookie;

	switch(etype) {
	case INV_IDDWRAPPER_PROTOCOL_ETYPE_CMD:
		INV_MSG(INV_MSG_LEVEL_WARNING, "DeviceEmdWrapper: CMD packet received. Ignored.");
		break;
	case INV_IDDWRAPPER_PROTOCOL_ETYPE_RESP:
		handle_response(self, eid, edata);
		break;
	case INV_IDDWRAPPER_PROTOCOL_ETYPE_ASYNC:
		handle_async(self, eid, edata);
		break;
	default:
		break; /* no suppose to happen */
	}
}

static int send_cmd(inv_device_emd_wrapper_t * self,
	enum inv_iddwrapper_protocol_eid eid,
	int sensor_id, inv_iddwrapper_protocol_edata_t * edata, int timeout)
{
	uint8_t buffer[128];
	uint16_t len;

	int rc;

	self->response_event = 0; /* reset response event flag */

	edata->sensor_id = sensor_id; /* finish-up building command args */

	/* create packet */
	rc = inv_iddwrapper_protocol_encode_command(&self->iddwrapper_protocol,
			eid, edata, buffer, sizeof(buffer), &len);

	if(rc != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "DeviceEmdWrapper: error formating command");
		return rc;
	}

	/* send packet */
	rc = inv_iddwrapper_transport_uart_tx(&self->iddwrapper_transport, buffer, len);
	if(rc != 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "DeviceEmdWrapper: error sending packet to transport layer");
		return rc;
	}

	/* wait for response */
	while(self->response_event == 0) {
		inv_device_emd_wrapper_poll(self);

		inv_sleep_ms(10);
		timeout -= 10;

		if(timeout <= 0) {
			return INV_ERROR_TIMEOUT;
		}
	}

	return 0;
}

void inv_device_emd_wrapper_init(inv_device_emd_wrapper_t * self,
	const inv_sensor_listener_t * listener,
	const struct inv_device_emd_wrapper_serial * serial, void * serial_cookie
)
{
	assert(self && listener);

	memset(self, 0, sizeof(*self));

	/* build base */
	self->base.instance = self;
	self->base.listener = listener;
	self->base.vt       = &device_emd_wrapper_vt;

	/* init IddWrapper */
	inv_iddwrapper_transport_uart_init(&self->iddwrapper_transport, iddwrapper_transport_event_cb, self);
	inv_iddwrapper_protocol_init(&self->iddwrapper_protocol, iddwrapper_protocol_event_cb, self);

	/* init other states */
	self->serial_read_cb        = serial->serial_read_cb;
	self->serial_read_buf       = serial->serial_read_buf;
	self->serial_read_buf_size  = serial->serial_read_buf_size;
	self->serial_write_cb       = serial->serial_write_cb;
	self->serial_write_buf      = serial->serial_write_buf;
	self->serial_write_buf_size = serial->serial_write_buf_size;
	self->serial_cookie         = serial_cookie;
}

int inv_device_emd_wrapper_is_hw_handshake_supported(inv_device_emd_wrapper_t * self)
{
	int rc;
	struct inv_iddwrapper_protocol_edata edata;
	
	edata.d.command.reg_addr = INV_IDDWRAPPER_PROTOCOL_EREG_HANDSHAKE_SUPPORT;
	rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG, 0, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_poll(void * context)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	/* call underlying UART to read as many bytes the buffer can store */
	const int rlen = self->serial_read_cb(self->serial_read_buf, self->serial_read_buf_size, self->serial_cookie);

	if(rlen < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "serial_read_cb() returned %d", rlen);
	}
	else {
		int idx;
		for(idx = 0; idx < rlen; ++idx) {
			inv_iddwrapper_transport_uart_rx_process_byte(&self->iddwrapper_transport, self->serial_read_buf[idx]);
		}
	}

	return 0;
}

int inv_device_emd_wrapper_setup(void * context)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_SETUP, 0, &edata, 5000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_cleanup(void * context)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP, 0, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_whoami(void * context, uint8_t * whoami)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I, 0, &edata, 1000);

	if(rc == 0) {
		*whoami = (int)self->response_data.rc;
	}

	return rc;
}

int inv_device_emd_wrapper_reset(void * context)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_RESET, 0, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_ping_sensor(void * context, int sensor)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR, sensor, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_enable_sensor(void * context, int sensor, inv_bool_t en)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	enum inv_iddwrapper_protocol_eid eid;
	struct inv_iddwrapper_protocol_edata edata;
	int rc;

	eid = (en) ? INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR : INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR;
	rc = send_cmd(self, eid, sensor, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_set_sensor_period_us(void * context,
		int sensor, uint32_t period)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	int rc;

	edata.d.command.period = period;
	rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD, sensor, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_set_sensor_timeout(void * context,
		int sensor, uint32_t timeout)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	int rc;

	edata.d.command.timeout = timeout;
	rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT, sensor, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_flush_sensor(void * context, int sensor)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR, sensor, &edata, 1000);

	return (rc == 0) ? self->response_data.rc : rc;
}

int inv_device_emd_wrapper_self_test(void * context, int sensor)
{
	inv_device_emd_wrapper_t * self = (inv_device_emd_wrapper_t *)context;

	struct inv_iddwrapper_protocol_edata edata;
	const int rc = send_cmd(self, INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST, sensor, &edata, 5000);

	return (rc == 0) ? self->response_data.rc : rc;
}
