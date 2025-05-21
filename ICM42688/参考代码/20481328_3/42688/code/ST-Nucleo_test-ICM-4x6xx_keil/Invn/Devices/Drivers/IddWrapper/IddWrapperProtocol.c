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

#include "IddWrapperProtocol.h"
#include "IddWrapperTransport.h"

#include "Invn/Utils/Message.h"
#include "Invn/Utils/DataConverter.h"

#include <string.h>

/** A frame transfer have the format bellow
	<EVENT_TYPE|IDD_WRAPPER_GRP> <PACKET>
	 According to the Event Type: 
	- CMD=0 
	- RESPONSE=1
	- ASYNC_EVENT=2
	Packet should contains : 
	- <CMD> <ARG>
	- <CMD> <ERROR_STATUS> <ARG>
	- <SENSOR_ID|STATUS> <TIMESTAMP> <DATA>
	*/

#define MAX_EXPECTECD_PAYLOAD (sizeof((*(inv_iddwrapper_protocol_t *)(0)).decode_state_machine.tmp_buffer))

#define EVENT_TYPE_CMD   (0 << 6)
#define EVENT_TYPE_RESP  (1 << 6)
#define EVENT_TYPE_ASYNC (2 << 6)
#define EVENT_TYPE_MASK 0xC0

#define PROTOCOL_ACCELEROMETER_PRECISION     11
#define PROTOCOL_MAGNETOMETER_PRECISION       4
#define PROTOCOL_GYROSCOPE_PRECISION          4
#define PROTOCOL_QUATERNION_PRECISION        14
#define PROTOCOL_RAW_PRECISION                0
#define PROTOCOL_ORIENTATION_PRECISION        6

#define PROTOCOL_ACCURACY_PRECISION           8

#define PROTOCOL_FLT_TO_SFIX16(value, shift)	( (int16_t)  ((float)(value)*(1ULL << (shift)) + ( (value>=0)-0.5f )) )
#define PROTOCOL_SFIX16_TO_FLT(value, shift)	( (float)  (int16_t)(value) / (float)(1ULL << (shift)) )

static inline int inv_iddwrapper_protocol_decode_vect16(const uint8_t * bytes, float * out, unsigned len, int qx)
{
	unsigned i;
	for(i = 0; i < len; ++i) {
		out[i] = PROTOCOL_SFIX16_TO_FLT((((int16_t)bytes[2*i+1] << 8) | bytes[2*i]), qx);
	}

	return 2*len;
}

static inline int inv_iddwrapper_protocol_encode_vect16(const float * in, unsigned len, int qx, uint8_t * bytes)
{
	unsigned i;
	for(i = 0; i < len; ++i) {
		int16_t x = PROTOCOL_FLT_TO_SFIX16(in[i], qx);
		bytes[2*i]   = (uint8_t)((uint16_t)x & 0xFF);
		bytes[2*i+1] = (uint8_t)(((uint16_t)x & 0xFF00) >> 8U);
	}

	return 2*len;
}

static uint16_t get_payload(inv_iddwrapper_protocol_t * self)
{
	const uint8_t event_type = self->decode_state_machine.event_type;
	const enum inv_iddwrapper_protocol_eid cmd_id = (enum inv_iddwrapper_protocol_eid)self->decode_state_machine.cmd_id;

	if(self->decode_state_machine.received_size == 0) {
		switch(event_type) {
		case EVENT_TYPE_CMD:
			switch(cmd_id) {
			case INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION:    return 0;
			case INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I:           return 0;
			case INV_IDDWRAPPER_PROTOCOL_EID_RESET:              return 0;
			case INV_IDDWRAPPER_PROTOCOL_EID_SETUP:              return 0;
			case INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP:            return 0;
			case INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST:          return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR:        return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR:       return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR:        return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD:  return 1 + 4;
			case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT: return 1 + 4;
			case INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR:       return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG:         return 1 + 1;
			default:
				break;
			}
			break;

		case EVENT_TYPE_RESP:
			switch(cmd_id) {
			case INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION:    return 16;
			case INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I:           return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_RESET:              return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_SETUP:              return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP:            return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST:          return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR:        return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR:       return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR:        return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD:  return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT: return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR:       return 1;
			case INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG:         return 1;
			default:
				break;
			}
			break;

		case EVENT_TYPE_ASYNC:
			switch(cmd_id) {
			case INV_IDDWRAPPER_PROTOCOL_EID_NEW_SENSOR_DATA:
				/* need at least one more byte to determine payload */
				return 1;
			default:
				break;
			}
			break;

		default:
			break;
		}
	}
	else {

		const uint8_t sensor_id = self->decode_state_machine.tmp_buffer[0] & ~0xC0;

		switch(event_type) {
		case EVENT_TYPE_ASYNC:
			switch(sensor_id) {
			case INV_SENSOR_TYPE_RESERVED:
				return self->decode_state_machine.received_size;
			case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
			case INV_SENSOR_TYPE_RAW_MAGNETOMETER:
			case INV_SENSOR_TYPE_RAW_GYROSCOPE:
			case INV_SENSOR_TYPE_OIS:
				return 4+6;
			case INV_SENSOR_TYPE_EIS:
				return 4+6+6+2;
			case INV_SENSOR_TYPE_RAW_TEMPERATURE:
				return 4+4;
			case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
			case INV_SENSOR_TYPE_GRAVITY:
			case INV_SENSOR_TYPE_ORIENTATION:
				return 4+6;
			case INV_SENSOR_TYPE_ACCELEROMETER: 
			case INV_SENSOR_TYPE_GYROSCOPE:
			case INV_SENSOR_TYPE_MAGNETOMETER:
				return 4+6+1;
			case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
				return 4+12;
			case INV_SENSOR_TYPE_DATA_ENCRYPTION:
				return 4+24;
			case INV_SENSOR_TYPE_3AXIS:
			case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
				return 4+8+1;
			case INV_SENSOR_TYPE_ROTATION_VECTOR:
			case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
				return 4+10+1;
			case INV_SENSOR_TYPE_B2S:
			case INV_SENSOR_TYPE_SHAKE:
			case INV_SENSOR_TYPE_DOUBLE_TAP:
			case INV_SENSOR_TYPE_SEDENTARY_REMIND:
			case INV_SENSOR_TYPE_SMD:
			case INV_SENSOR_TYPE_STEP_DETECTOR:
			case INV_SENSOR_TYPE_TILT_DETECTOR:
			case INV_SENSOR_TYPE_WAKE_GESTURE:
			case INV_SENSOR_TYPE_GLANCE_GESTURE:
			case INV_SENSOR_TYPE_PICK_UP_GESTURE:
			case INV_SENSOR_TYPE_PRESSURE:
			case INV_SENSOR_TYPE_LIGHT:
				return 4+4;
			case INV_SENSOR_TYPE_WOM:
			case INV_SENSOR_TYPE_BAC:
				return 4+1+4;
			case INV_SENSOR_TYPE_STEP_COUNTER:
				return 4+4+4;
			case INV_SENSOR_TYPE_PROXIMITY:
				return 4+2;
			default:
				/* undefined for now */
				return -1;
			}
			break;

		default:
			/* do not need to update expected payload */
			return self->decode_state_machine.expected_size;
		}
	}

	INV_MSG(INV_MSG_LEVEL_WARNING, "IddWrapperProtocol: returned payload is -1");

	return -1;
}

static int decode_pkt_command(inv_iddwrapper_protocol_t * self,
		struct inv_iddwrapper_protocol_edata * edata)
{
	const uint8_t * buf = self->decode_state_machine.tmp_buffer;

	if(self->decode_state_machine.cmd_id == INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION) {
		edata->sensor_id = 0;
	}
	else {
		edata->sensor_id = buf[0];

		switch(self->decode_state_machine.cmd_id) {
		case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD:
			edata->d.command.period = (uint32_t)inv_dc_little8_to_int32(&buf[1]);
			break;
		case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
			edata->d.command.timeout = (uint32_t)inv_dc_little8_to_int32(&buf[1]);
			break;
		case INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG:
			edata->d.command.reg_addr = buf[1];
			break;
			
		default:
			break;
		}
	}

	return 0;
}

static int decode_pkt_response(inv_iddwrapper_protocol_t * self,
		struct inv_iddwrapper_protocol_edata * edata)
{
	const uint8_t * buf = self->decode_state_machine.tmp_buffer;

	if(self->decode_state_machine.cmd_id == INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION) {
		memcpy(edata->d.response.version, buf, sizeof(edata->d.response.version) - 1);
		edata->d.response.version[sizeof(edata->d.response.version)-1] = '\0';
	}
	else {
		edata->d.response.rc = (int)(int8_t)buf[0];
	}

	return 0;
}

static uint32_t decode_full_timestamp(const uint8_t * buffer)
{
	return (uint32_t)buffer[0] | ((uint32_t)buffer[1] << 8) | ((uint32_t)buffer[2] << 16) | ((uint32_t)buffer[3] << 24);
}

static int build_sensor_event(const uint8_t * buffer, uint16_t size,
		inv_sensor_event_t * sensor_event)
{
	uint16_t idx = 0;

	(void)size; // FIXME check for size

	memset(sensor_event, 0, sizeof(*sensor_event));

	sensor_event->sensor    = (int)(buffer[0] & ~0xC0);
	sensor_event->status    = (enum inv_sensor_status)((buffer[0] & 0xC0) >> 6);
	sensor_event->timestamp = (uint32_t)buffer[1] | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3] << 16);

	idx += 4;

	switch(sensor_event->sensor) {
	case INV_SENSOR_TYPE_RESERVED:
		break;
	case INV_SENSOR_TYPE_ACCELEROMETER:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.acc.vect, 3, PROTOCOL_ACCELEROMETER_PRECISION);
		sensor_event->data.acc.accuracy_flag = buffer[idx++];
		break;
	case INV_SENSOR_TYPE_GRAVITY:
	case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.acc.vect, 3, PROTOCOL_ACCELEROMETER_PRECISION);
		break;
	case INV_SENSOR_TYPE_MAGNETOMETER:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.mag.vect,3, PROTOCOL_MAGNETOMETER_PRECISION);
		sensor_event->data.mag.accuracy_flag = buffer[idx++];
		break;
	case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.mag.vect, 3, PROTOCOL_MAGNETOMETER_PRECISION);
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.mag.bias, 3, PROTOCOL_MAGNETOMETER_PRECISION);
		break;
	case INV_SENSOR_TYPE_GYROSCOPE:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.gyr.vect,3, PROTOCOL_GYROSCOPE_PRECISION);
		sensor_event->data.gyr.accuracy_flag = buffer[idx++];
		break;
	case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.gyr.vect, 3, PROTOCOL_GYROSCOPE_PRECISION);
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.gyr.bias, 3, PROTOCOL_GYROSCOPE_PRECISION);
		break;
	case INV_SENSOR_TYPE_3AXIS:
	case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.quaternion.quat, 4, PROTOCOL_QUATERNION_PRECISION);
		sensor_event->data.quaternion.accuracy_flag = buffer[idx++];
		break;
	case INV_SENSOR_TYPE_ROTATION_VECTOR:
	case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.quaternion.quat, 4, PROTOCOL_QUATERNION_PRECISION);
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], &sensor_event->data.quaternion.accuracy, 1, PROTOCOL_ACCURACY_PRECISION);
		sensor_event->data.quaternion.accuracy_flag = buffer[idx++];
		break;

	case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
	case INV_SENSOR_TYPE_RAW_GYROSCOPE:
	case INV_SENSOR_TYPE_RAW_MAGNETOMETER:
	case INV_SENSOR_TYPE_OIS:
		sensor_event->data.raw3d.vect[0] = inv_dc_le_to_int16(&buffer[idx]);
		sensor_event->data.raw3d.vect[1] = inv_dc_le_to_int16(&buffer[idx+2]);
		sensor_event->data.raw3d.vect[2] = inv_dc_le_to_int16(&buffer[idx+4]);
		break;
	case INV_SENSOR_TYPE_DATA_ENCRYPTION:
		sensor_event->data.dataencryption.table[0] = buffer[idx];
		sensor_event->data.dataencryption.table[1] = buffer[idx+4];
		sensor_event->data.dataencryption.table[2] = buffer[idx+8];
		sensor_event->data.dataencryption.table[3] = buffer[idx+12];
		sensor_event->data.dataencryption.table[4] = buffer[idx+16];
		sensor_event->data.dataencryption.table[5] = buffer[idx+20];
		break;

	case INV_SENSOR_TYPE_RAW_TEMPERATURE:
		sensor_event->data.rawtemp.raw = inv_dc_little8_to_int32(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_STEP_COUNTER:
		sensor_event->data.step.count = (uint64_t)inv_dc_little8_to_int32(&buffer[idx]);
		sensor_event->timestamp = decode_full_timestamp(&buffer[idx+4]);
		break;

	case INV_SENSOR_TYPE_BAC:
		sensor_event->data.bac.event = (int)(int8_t)buffer[idx++];
		sensor_event->timestamp = decode_full_timestamp(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_WOM:
		sensor_event->data.wom.flags = buffer[idx++];
		sensor_event->timestamp = decode_full_timestamp(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_B2S:
	case INV_SENSOR_TYPE_SHAKE:
	case INV_SENSOR_TYPE_DOUBLE_TAP:
	case INV_SENSOR_TYPE_SEDENTARY_REMIND:
	case INV_SENSOR_TYPE_SMD:
	case INV_SENSOR_TYPE_STEP_DETECTOR:
	case INV_SENSOR_TYPE_TILT_DETECTOR:
	case INV_SENSOR_TYPE_WAKE_GESTURE:
	case INV_SENSOR_TYPE_GLANCE_GESTURE:
	case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		sensor_event->data.event = true;
		sensor_event->timestamp = decode_full_timestamp(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_PRESSURE:
		sensor_event->data.pressure.pressure = inv_dc_little8_to_int32(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_ORIENTATION:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], &sensor_event->data.orientation.x, 1, PROTOCOL_ORIENTATION_PRECISION);
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], &sensor_event->data.orientation.y, 1, PROTOCOL_ORIENTATION_PRECISION);
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], &sensor_event->data.orientation.z, 1, PROTOCOL_ORIENTATION_PRECISION);
		break;

	case INV_SENSOR_TYPE_LIGHT:
		sensor_event->data.light.level = inv_dc_little8_to_int32(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_PROXIMITY:
		sensor_event->data.proximity.distance = (uint32_t)inv_dc_le_to_int16(&buffer[idx]);
		break;

	case INV_SENSOR_TYPE_EIS:
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.eis.vect, 3, PROTOCOL_GYROSCOPE_PRECISION);
		idx += inv_iddwrapper_protocol_decode_vect16(&buffer[idx], sensor_event->data.eis.bias, 3, PROTOCOL_GYROSCOPE_PRECISION);
		sensor_event->data.eis.delta_ts = inv_dc_le_to_int16(&buffer[idx]);
		break;
	default:
		return -1;
	}
	
	return 0;
}

static int decode_pkt_async(inv_iddwrapper_protocol_t * self,
		struct inv_iddwrapper_protocol_edata * edata)
{
	const uint8_t * buf = self->decode_state_machine.tmp_buffer;
	const uint16_t len = self->decode_state_machine.received_size;

	switch(self->decode_state_machine.cmd_id) {
	case INV_IDDWRAPPER_PROTOCOL_EID_NEW_SENSOR_DATA:
		return build_sensor_event(buf, len, &edata->d.async.sensor_event);
	default:
		return -1;
	}
}

static inline void call_event_cb(inv_iddwrapper_protocol_t * self,
	enum inv_iddwrapper_protocol_etype etype,
	enum inv_iddwrapper_protocol_eid eid,
	const inv_iddwrapper_protocol_edata_t * edata
)
{
	if(self->event_cb) {
		self->event_cb(etype, eid, edata, self->event_cb_cookie);
	}
}

static int do_process(inv_iddwrapper_protocol_t * self)
{
	struct inv_iddwrapper_protocol_edata edata;
	enum inv_iddwrapper_protocol_etype etype;
	int rc;

	self->decode_state_machine.state = PROTOCOL_STATE_IDLE;

	switch(self->decode_state_machine.event_type) {
	case EVENT_TYPE_CMD:
		rc = decode_pkt_command(self, &edata);
		etype = INV_IDDWRAPPER_PROTOCOL_ETYPE_CMD;
		break;
	case EVENT_TYPE_RESP:
		rc = decode_pkt_response(self, &edata);
		etype = INV_IDDWRAPPER_PROTOCOL_ETYPE_RESP;
		break;
	case EVENT_TYPE_ASYNC:
		rc = decode_pkt_async(self, &edata);
		etype = INV_IDDWRAPPER_PROTOCOL_ETYPE_ASYNC;
		break;
	default:
		INV_MSG(INV_MSG_LEVEL_WARNING, "IddWrapperProtocol: Unexpected packet type");
		return -1;
	}

	if(rc == 0) {
		call_event_cb(self, etype, (enum inv_iddwrapper_protocol_eid)self->decode_state_machine.cmd_id, &edata);
		return 1;
	} else {
		INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Unexpected packet received.");
	}

	return rc;
}

void inv_iddwrapper_protocol_init(inv_iddwrapper_protocol_t * self,
		inv_iddwrapper_protocol_event_cb event_cb, void * event_cb_cookie)
{
	memset(self, 0, sizeof(*self));

	self->event_cb        = event_cb;
	self->event_cb_cookie = event_cb_cookie;
}

int inv_iddwrapper_protocol_process_pkt_byte(inv_iddwrapper_protocol_t * self, uint8_t rcv_byte)
{
	switch(self->decode_state_machine.state) {
	case PROTOCOL_STATE_GID:
		self->decode_state_machine.event_type = (rcv_byte & EVENT_TYPE_MASK);
		self->decode_state_machine.group_id   = (rcv_byte & ~EVENT_TYPE_MASK);
		self->decode_state_machine.state      = PROTOCOL_STATE_CID;

		if(self->decode_state_machine.group_id != INV_IDDWRAPPER_PROTOCOL_GROUP_ID) {
			self->decode_state_machine.state  = PROTOCOL_STATE_IDLE;
			INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Invalid group ID");
			return -1;
		}
		break;
	case PROTOCOL_STATE_CID:
		self->decode_state_machine.cmd_id     = rcv_byte;
		self->decode_state_machine.state      = PROTOCOL_STATE_PAYLOAD;
		self->decode_state_machine.received_size = 0;
		self->decode_state_machine.expected_size = 0;
		self->decode_state_machine.expected_size = get_payload(self);

		if(self->decode_state_machine.expected_size == UINT16_MAX) {
			/* assume frame is unknown, return error */
			self->decode_state_machine.state  = PROTOCOL_STATE_IDLE;
			INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Unknown frame");
			return -1;
		}

		if(self->decode_state_machine.expected_size == 0) {
			return do_process(self);
		}
		break;
	case PROTOCOL_STATE_PAYLOAD:
		if(self->decode_state_machine.received_size >= MAX_EXPECTECD_PAYLOAD) {
			INV_MSG(INV_MSG_LEVEL_WARNING, "IddWrapperProtocol: internal buffer size full");
		}
		else {
			self->decode_state_machine.tmp_buffer[self->decode_state_machine.received_size] = rcv_byte;
		}
		self->decode_state_machine.received_size++;
		/* update expected payload, in case actual payload cannot be determined using only CID */
		self->decode_state_machine.expected_size = get_payload(self);

		if(self->decode_state_machine.expected_size == UINT16_MAX) {
			/* assume frame is unknown, return error */
			self->decode_state_machine.state  = PROTOCOL_STATE_IDLE;
			INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Unknown frame");
			return -1;
		}

		if(self->decode_state_machine.received_size == self->decode_state_machine.expected_size) {
			return do_process(self);
		}
		break;
	}

	return 0;
}

int inv_iddwrapper_protocol_encode_command(inv_iddwrapper_protocol_t * self,
		enum inv_iddwrapper_protocol_eid eid, const inv_iddwrapper_protocol_edata_t * edata,
		uint8_t * out_buffer, uint16_t max_buffer_size, uint16_t *out_buffer_size)
{
	uint16_t idx = 0;

	(void)self;

	*out_buffer_size = 0;

	if(max_buffer_size < 2)
		goto error_size;

	out_buffer[idx]  = EVENT_TYPE_CMD; // Set event type
	out_buffer[idx++] |= (INV_IDDWRAPPER_PROTOCOL_GROUP_ID & ~EVENT_TYPE_MASK); // Set group ID 
	out_buffer[idx++] = (uint8_t)eid;
	
	switch(eid) {
	case INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION:
	case INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I:
	case INV_IDDWRAPPER_PROTOCOL_EID_RESET:
	case INV_IDDWRAPPER_PROTOCOL_EID_SETUP:
	case INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP:
		break;
		
	case INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST:
	case INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR:
		if((max_buffer_size - idx) < 1)
			goto error_size;
		out_buffer[idx++] = (uint8_t)edata->sensor_id;
		break;

	case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD:
		if((max_buffer_size - idx) < 5)
			goto error_size;
		out_buffer[idx++] = (uint8_t)edata->sensor_id;
		inv_dc_int32_to_little8(edata->d.command.period, &out_buffer[idx]);
		idx += 4;
		break;
	
	case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
		if((max_buffer_size - idx) < 5)
			goto error_size;
		out_buffer[idx++] = (uint8_t)edata->sensor_id;
		inv_dc_int32_to_little8(edata->d.command.timeout, &out_buffer[idx]);
		idx += 4;
		break;

	case INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG:
		if((max_buffer_size - idx) < 2)
			goto error_size;
		out_buffer[idx++] = (uint8_t)edata->sensor_id;
		out_buffer[idx++] = (uint8_t)edata->d.command.reg_addr;
		break;

	default:
		INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Unexpected argument for encode_command()");	
		return -1;
	}

	*out_buffer_size = (idx);
	return 0;

error_size:
	INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: output buffer size too small");
	return -1;
}

int inv_iddwrapper_protocol_encode_response(inv_iddwrapper_protocol_t * self,
		enum inv_iddwrapper_protocol_eid eid, const inv_iddwrapper_protocol_edata_t * edata,
		uint8_t * out_buffer, uint16_t max_buffer_size, uint16_t *out_buffer_size)
{
	uint16_t idx = 0;

	(void)self;

	*out_buffer_size = 0;

	if(max_buffer_size < 2)
		goto error_size;

	out_buffer[idx]  = EVENT_TYPE_RESP; // Set event type
	out_buffer[idx++] |= INV_IDDWRAPPER_PROTOCOL_GROUP_ID & ~EVENT_TYPE_MASK; // Set group ID 
	out_buffer[idx++] = (uint8_t)eid;

	switch(eid) {
	case INV_IDDWRAPPER_PROTOCOL_EID_PROTOCOLVERSION:
		if((max_buffer_size - idx) < 16)
			goto error_size;
		memcpy(&out_buffer[idx], edata->d.response.version, 15);
		out_buffer[idx+15] = '\0';
		idx += 16;
		break;

	case INV_IDDWRAPPER_PROTOCOL_EID_WHO_AM_I:
	case INV_IDDWRAPPER_PROTOCOL_EID_RESET:
	case INV_IDDWRAPPER_PROTOCOL_EID_SETUP:
	case INV_IDDWRAPPER_PROTOCOL_EID_CLEANUP:
	case INV_IDDWRAPPER_PROTOCOL_EID_SELF_TEST:
	case INV_IDDWRAPPER_PROTOCOL_EID_PING_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_START_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_STOP_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_FLUSH_SENSOR:
	case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_PERIOD:
	case INV_IDDWRAPPER_PROTOCOL_EID_SET_SENSOR_TIMEOUT:
	case INV_IDDWRAPPER_PROTOCOL_EID_GET_SW_REG:
		if((max_buffer_size - idx) < 1)
			goto error_size;
		out_buffer[idx++] = (uint8_t)edata->d.response.rc;
		break;

	default:
		INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Unexpected argument for encode_response()");	
		return -1;
	}

	*out_buffer_size = (idx);

	return 0;

error_size:
	INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: output buffer size too small");
	return -1;
}

static int encode_full_timestamp(uint8_t * out_buffer, uint32_t timestamp)
{
	out_buffer[0] = (timestamp & 0xFF);
	out_buffer[1] = (timestamp & 0xFF00) >> 8;
	out_buffer[2] = (timestamp & 0xFF0000) >> 16;
	out_buffer[3] = (timestamp & 0xFF000000) >> 24;

	return 4;
}

static int encode_sensor_event(const inv_sensor_event_t * sensor_event,
		uint8_t * out_buffer, uint16_t max_buffer_size)
{
	uint16_t idx = 0;
	uint32_t timestamp = (uint32_t)sensor_event->timestamp;

	if(max_buffer_size < 4) {
		goto error_size;
	}

	out_buffer[idx] = (uint8_t)sensor_event->sensor & ~0xC0;
	out_buffer[idx++] |= (((uint8_t)sensor_event->status & 0x03) << 6);
	out_buffer[idx++] = timestamp & 0x000000FF;
	out_buffer[idx++] = (timestamp & 0x0000FF00) >> 8;
	out_buffer[idx++] = (timestamp & 0x00FF0000) >> 16;

	max_buffer_size -= idx;

	switch(sensor_event->sensor) {
	case INV_SENSOR_TYPE_RESERVED:
		/* no data */
		break;
	case INV_SENSOR_TYPE_ACCELEROMETER:
		if(max_buffer_size < 6+1)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.acc.vect, 3, PROTOCOL_ACCELEROMETER_PRECISION, &out_buffer[idx]);
		out_buffer[idx++] = sensor_event->data.acc.accuracy_flag;
		break;
	case INV_SENSOR_TYPE_GRAVITY:
	case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		if(max_buffer_size < 6)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.acc.vect, 3, PROTOCOL_ACCELEROMETER_PRECISION, &out_buffer[idx]);
		break;
	case INV_SENSOR_TYPE_MAGNETOMETER:
		if(max_buffer_size < 6+1)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.mag.vect,3, PROTOCOL_MAGNETOMETER_PRECISION, &out_buffer[idx]);
		out_buffer[idx++] = sensor_event->data.mag.accuracy_flag;
		break;
	case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		if(max_buffer_size < 12)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.mag.vect, 3, PROTOCOL_MAGNETOMETER_PRECISION, &out_buffer[idx]);
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.mag.bias, 3, PROTOCOL_MAGNETOMETER_PRECISION, &out_buffer[idx]);
		break;
	case INV_SENSOR_TYPE_GYROSCOPE:
		if(max_buffer_size < 6+1)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.gyr.vect,3, PROTOCOL_GYROSCOPE_PRECISION, &out_buffer[idx]);
		out_buffer[idx++] = sensor_event->data.gyr.accuracy_flag;
		break;
	case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
		if(max_buffer_size < 12)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.gyr.vect, 3, PROTOCOL_GYROSCOPE_PRECISION, &out_buffer[idx]);
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.gyr.bias, 3, PROTOCOL_GYROSCOPE_PRECISION, &out_buffer[idx]);
		break;
	case INV_SENSOR_TYPE_3AXIS:
	case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		if(max_buffer_size < 9)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.quaternion.quat, 4, PROTOCOL_QUATERNION_PRECISION, &out_buffer[idx]);
		out_buffer[idx++] = sensor_event->data.quaternion.accuracy_flag;
		break;
	case INV_SENSOR_TYPE_ROTATION_VECTOR:
	case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		if(max_buffer_size < 11)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.quaternion.quat, 4, PROTOCOL_QUATERNION_PRECISION, &out_buffer[idx]);
		idx += inv_iddwrapper_protocol_encode_vect16(&sensor_event->data.quaternion.accuracy, 1, PROTOCOL_ACCURACY_PRECISION, &out_buffer[idx]);
		out_buffer[idx++] = sensor_event->data.quaternion.accuracy_flag;
		break;
	case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
	case INV_SENSOR_TYPE_RAW_GYROSCOPE:
	case INV_SENSOR_TYPE_RAW_MAGNETOMETER:
	case INV_SENSOR_TYPE_OIS:
		if(max_buffer_size < 6)
			goto error_size;
		inv_dc_int16_to_little8(sensor_event->data.raw3d.vect[0], &out_buffer[idx]);
		inv_dc_int16_to_little8(sensor_event->data.raw3d.vect[1], &out_buffer[idx+2]);
		inv_dc_int16_to_little8(sensor_event->data.raw3d.vect[2], &out_buffer[idx+4]);
		idx += 6;
		break;

	case INV_SENSOR_TYPE_RAW_TEMPERATURE:
		if(max_buffer_size < 4)
			goto error_size;
		inv_dc_int32_to_little8(sensor_event->data.rawtemp.raw, &out_buffer[idx]);
		idx += 4;
		break;
	case INV_SENSOR_TYPE_STEP_COUNTER:
		if(max_buffer_size < 8)
			goto error_size;
		inv_dc_int32_to_little8((int32_t)sensor_event->data.step.count, &out_buffer[idx]);
		idx += 4;
		idx += encode_full_timestamp(&out_buffer[idx], timestamp);
		break;

	case INV_SENSOR_TYPE_WOM:
		if(max_buffer_size < 5)
			goto error_size;
		out_buffer[idx++] = (uint8_t)sensor_event->data.wom.flags;
		idx += encode_full_timestamp(&out_buffer[idx], timestamp);
		break;

	case INV_SENSOR_TYPE_BAC:
		if(max_buffer_size < 5)
			goto error_size;
		out_buffer[idx++] = (uint8_t)(int8_t)sensor_event->data.bac.event;
		idx += encode_full_timestamp(&out_buffer[idx], timestamp);
		break;

	case INV_SENSOR_TYPE_B2S:
	case INV_SENSOR_TYPE_SHAKE:
	case INV_SENSOR_TYPE_DOUBLE_TAP:
	case INV_SENSOR_TYPE_SEDENTARY_REMIND:
	case INV_SENSOR_TYPE_SMD:
	case INV_SENSOR_TYPE_STEP_DETECTOR:
	case INV_SENSOR_TYPE_TILT_DETECTOR:
	case INV_SENSOR_TYPE_WAKE_GESTURE:
	case INV_SENSOR_TYPE_GLANCE_GESTURE:
	case INV_SENSOR_TYPE_PICK_UP_GESTURE:
		if(max_buffer_size < 4)
			goto error_size;
		idx += encode_full_timestamp(&out_buffer[idx], timestamp);
		break;

	case INV_SENSOR_TYPE_PRESSURE:
		if(max_buffer_size < 4)
			goto error_size;
		inv_dc_int32_to_little8(sensor_event->data.pressure.pressure, &out_buffer[idx]);
		idx += 4;
		break;

	case INV_SENSOR_TYPE_ORIENTATION:
		if(max_buffer_size < 6)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(&sensor_event->data.orientation.x, 1, PROTOCOL_ORIENTATION_PRECISION, &out_buffer[idx]);
		idx += inv_iddwrapper_protocol_encode_vect16(&sensor_event->data.orientation.y, 1, PROTOCOL_ORIENTATION_PRECISION, &out_buffer[idx]);
		idx += inv_iddwrapper_protocol_encode_vect16(&sensor_event->data.orientation.z, 1, PROTOCOL_ORIENTATION_PRECISION, &out_buffer[idx]);
		break;

	case INV_SENSOR_TYPE_LIGHT:
		if(max_buffer_size < 4)
			goto error_size;
		inv_dc_int32_to_little8(sensor_event->data.light.level, &out_buffer[idx]);
		idx += 4;
		break;

	case INV_SENSOR_TYPE_PROXIMITY:
		if(max_buffer_size < 2)
			goto error_size;
		inv_dc_int16_to_little8((int16_t)sensor_event->data.proximity.distance, &out_buffer[idx]);
		idx += 2;
		break;

	case INV_SENSOR_TYPE_EIS:
		if(max_buffer_size < 14)
			goto error_size;
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.eis.vect, 3, PROTOCOL_GYROSCOPE_PRECISION, &out_buffer[idx]);
		idx += inv_iddwrapper_protocol_encode_vect16(sensor_event->data.eis.bias, 3, PROTOCOL_GYROSCOPE_PRECISION, &out_buffer[idx]);
		inv_dc_int16_to_little8(sensor_event->data.eis.delta_ts, &out_buffer[idx]);
		idx += 2;
		break;
	default:
		return -1;
	}

	return idx;
error_size:
	return max_buffer_size + idx + 1; /* +1 to indicate buffer is too small */
}

int inv_iddwrapper_protocol_encode_async(inv_iddwrapper_protocol_t * self,
		enum inv_iddwrapper_protocol_eid eid, const inv_iddwrapper_protocol_edata_t * edata,
		uint8_t * out_buffer, uint16_t max_buffer_size, uint16_t *out_buffer_size)
{
	uint16_t idx = 0;
	int len;

	(void)self;

	*out_buffer_size = 0;

	if(max_buffer_size < 2)
		goto error_size;

	out_buffer[idx]  = EVENT_TYPE_ASYNC; // Set event type
	out_buffer[idx++] |= INV_IDDWRAPPER_PROTOCOL_GROUP_ID & ~EVENT_TYPE_MASK; // Set group ID 
	out_buffer[idx++] = (uint8_t)eid;

	switch(eid) {
	case INV_IDDWRAPPER_PROTOCOL_EID_NEW_SENSOR_DATA:
		len = encode_sensor_event(&edata->d.async.sensor_event, &out_buffer[idx], max_buffer_size - idx);
		if(len == -1) {
			goto error_arg;
		}
		else if(len > max_buffer_size - idx) {
			goto error_size;
		}

		idx += len;
		break;

	default:
		goto error_arg;
	}

	*out_buffer_size = (idx);

	return 0;

error_arg:
	INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: Unexpected argument for encode_async()");	
	return -1;

error_size:
	INV_MSG(INV_MSG_LEVEL_ERROR, "IddWrapperProtocol: output buffer size too small");
	return -1;
}
