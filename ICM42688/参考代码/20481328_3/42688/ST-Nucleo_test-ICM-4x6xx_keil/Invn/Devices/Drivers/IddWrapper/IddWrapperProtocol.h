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

#ifndef _IDDWRAPPER_PROTOCOL_H_
#define _IDDWRAPPER_PROTOCOL_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/SensorTypes.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define INV_IDDWRAPPER_PROTOCOL_GROUP_ID  2
#define INV_IDDWRAPPER_PROTOCOL_VERSION "0.0.1-dev1"

/* Commented label are currently not implemented */

enum inv_iddwrapper_protocol_eid {
	/* Protocol */
	INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION = 0x00,

	/* IDD methods */
	INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I           = 0x10,
	INV_IDDWRAPPER_PROTOCOL_EID_RESET              = 0x11,
	INV_IDDWRAPPER_PROTOCOL_EID_SETUP              = 0x12,
	INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP            = 0x13,
	// INV_IDDWRAPPER_PROTOCOL_EID_LOAD               = 0x14,
	INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST          = 0x15,
	// INV_IDDWRAPPER_PROTOCOL_EID_GET_FW_INFO        = 0x16,
	INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR        = 0x17,
	// INV_IDDWRAPPER_PROTOCOL_EID_SET_RUNNING_STATE  = 0x18,
	INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR       = 0x19,
	INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR        = 0x1A,
	INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD  = 0x1B,
	INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT = 0x1C,
	INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR       = 0x1D,
	// INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_BIAS    = 0x1E,
	// INV_IDDWRAPPER_PROTOCOL_EID_GET_SENSOR_BIAS    = 0x1F,
	// INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_MMATRIX = 0x20,
	// INV_IDDWRAPPER_PROTOCOL_EID_GET_SENSOR_DATA    = 0x21,
	INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG         = 0x22,
	
	/* Events */
	INV_IDDWRAPPER_PROTOCOL_EID_NEW_SENSOR_DATA    = 0x30,
};

enum inv_iddwrapper_protocol_etype {
	INV_IDDWRAPPER_PROTOCOL_ETYPE_CMD   = 0,
	INV_IDDWRAPPER_PROTOCOL_ETYPE_RESP  = 1,
	INV_IDDWRAPPER_PROTOCOL_ETYPE_ASYNC = 2
};

enum inv_iddwrapper_protocol_ereg {
	INV_IDDWRAPPER_PROTOCOL_EREG_MAJOR_VERSION   = 0,
	INV_IDDWRAPPER_PROTOCOL_EREG_MINOR_VERSION,
	INV_IDDWRAPPER_PROTOCOL_EREG_REV_VERSION,
	INV_IDDWRAPPER_PROTOCOL_EREG_SUFFIX_VERSION,
	INV_IDDWRAPPER_PROTOCOL_EREG_HANDSHAKE_SUPPORT
};

typedef struct inv_iddwrapper_protocol_edata {
	int sensor_id; /* 0 if not applicable */
	union {
		union {
			uint32_t period;  /* for EID_SET_SENSOR_PERIOD */
			uint32_t timeout; /* for EID_SET_SENSOR_TIMEOUT */
			uint8_t reg_addr; /* for EID GET_SW_REG */
		} command;
		union {
			char version[16]; /* IDD WRAPPER PROTOCOL version, for EID_PROTOCOLVERSION */
			int rc; /* return code as of @ref inv_error for other IDD methods*/
		} response; /* returned data if applicable */
		union {
			inv_sensor_event_t sensor_event; /* for EID_NEW_SENSOR_DATA */
		} async;
	} d;
} inv_iddwrapper_protocol_edata_t;

typedef void (*inv_iddwrapper_protocol_event_cb)(
		enum inv_iddwrapper_protocol_etype etype,
		enum inv_iddwrapper_protocol_eid eid,
		const inv_iddwrapper_protocol_edata_t * edata,
		void * cookie
	);

typedef struct inv_iddwrapper_protocol {
	inv_iddwrapper_protocol_event_cb event_cb;
	void *                          event_cb_cookie;
	struct {
		enum sm {
			PROTOCOL_STATE_IDLE = 0,
			PROTOCOL_STATE_GID = PROTOCOL_STATE_IDLE,
			PROTOCOL_STATE_CID,
			PROTOCOL_STATE_PAYLOAD,
		} state;
		uint8_t event_type;
		uint8_t group_id;
		uint8_t cmd_id;
		uint16_t expected_size;
		uint16_t received_size;
		uint8_t  tmp_buffer[256];
	} decode_state_machine;
} inv_iddwrapper_protocol_t;

void INV_EXPORT inv_iddwrapper_protocol_init(inv_iddwrapper_protocol_t * self,
		inv_iddwrapper_protocol_event_cb event_cb, void * event_cb_cookie);

int INV_EXPORT inv_iddwrapper_protocol_process_pkt_byte(inv_iddwrapper_protocol_t * self, uint8_t rcv_byte);

int INV_EXPORT inv_iddwrapper_protocol_encode_async(inv_iddwrapper_protocol_t * self,
		enum inv_iddwrapper_protocol_eid eid, const inv_iddwrapper_protocol_edata_t * edata,
		uint8_t * out_buffer, uint16_t max_buffer_size, uint16_t *out_buffer_size);

int INV_EXPORT inv_iddwrapper_protocol_encode_response(inv_iddwrapper_protocol_t * self,
		enum inv_iddwrapper_protocol_eid eid, const inv_iddwrapper_protocol_edata_t * edata,
		uint8_t * out_buffer, uint16_t max_buffer_size, uint16_t *out_buffer_size);

int INV_EXPORT inv_iddwrapper_protocol_encode_command(inv_iddwrapper_protocol_t * self,
		enum inv_iddwrapper_protocol_eid eid, const inv_iddwrapper_protocol_edata_t * edata,
		uint8_t * out_buffer, uint16_t max_buffer_size, uint16_t *out_buffer_size);

#ifdef __cplusplus
}
#endif

#endif /* _IDDWRAPPER_PROTOCOL_H_ */
