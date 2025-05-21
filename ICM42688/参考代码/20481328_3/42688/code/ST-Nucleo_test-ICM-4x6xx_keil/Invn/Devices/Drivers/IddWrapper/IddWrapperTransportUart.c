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

#include "IddWrapperTransportUart.h"

#include "Invn/Utils/Message.h"

#include <stdint.h>
#include <string.h>

/** TX/RX frames have the following format: 0x55 0xAA <NB_BYTES (2)> <PACKET (n)> */

#define SYNC_BYTE_0         0x55
#define SYNC_BYTE_1         0xAA

void inv_iddwrapper_transport_uart_init(inv_iddwrapper_transport_uart_t * self,
		inv_iddwrapper_transport_event_cb event_cb, void * cookie)
{
	self->event_cb        = event_cb;
	self->event_cb_cookie = cookie;
	self->rx_sm_state     = RECEIVER_STATE_IDLE;
}

void inv_iddwrapper_transport_uart_rx_process_reset(inv_iddwrapper_transport_uart_t * self)
{
	self->rx_sm_state = RECEIVER_STATE_IDLE;
}

static inline void call_event_cb(inv_iddwrapper_transport_uart_t * self, 
		enum inv_iddwrapper_transport_event event,
		union inv_iddwrapper_transport_event_data data)
{
	if(self->event_cb) {
		self->event_cb(event, data, self->event_cb_cookie);
	}
}

int inv_iddwrapper_transport_uart_rx_process_byte(inv_iddwrapper_transport_uart_t * self, uint8_t rcv_byte)
{
	union inv_iddwrapper_transport_event_data udata;

	switch(self->rx_sm_state) {
	case RECEIVER_STATE_IDLE:
		if(rcv_byte == SYNC_BYTE_0) {
			self->rx_sm_state = RECEIVER_STATE_SYNC_1;
		}
		else {
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "IddWrapperTransportUart: unexpected SYNC0 byte %x recevied", rcv_byte);
			self->rx_sm_state = RECEIVER_STATE_IDLE;
			udata.error = -1;
			call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_ERROR, udata);
			return -1;
		}
		break;

	case RECEIVER_STATE_SYNC_1:
		if(rcv_byte == SYNC_BYTE_1) {
			self->rx_sm_state = RECEIVER_STATE_SIZE_BYTE_0;
		}
		else {
			INV_MSG(INV_MSG_LEVEL_VERBOSE, "IddWrapperTransportUart: unexpected SYNC1 byte %x recevied", rcv_byte);
			self->rx_sm_state = RECEIVER_STATE_IDLE;
			udata.error = -1;
			call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_ERROR, udata);
			return -1;
		}
		break;

	case RECEIVER_STATE_SIZE_BYTE_0:
		self->rx_expected_bytes = (uint16_t)rcv_byte;
		self->rx_sm_state = RECEIVER_STATE_SIZE_BYTE_1;
		break;

	case RECEIVER_STATE_SIZE_BYTE_1:
		self->rx_expected_bytes |= ((uint16_t)rcv_byte << 8U);
		self->rx_received_bytes = 0;
		self->rx_sm_state = RECEIVER_STATE_PACKET_DATA;
		udata.pkt_size = self->rx_expected_bytes;
		call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_SIZE, udata);
		return 1;

	case RECEIVER_STATE_PACKET_DATA:
		self->rx_received_bytes++;
		udata.pkt_size = rcv_byte;
		call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_BYTE, udata);
		if(self->rx_received_bytes == self->rx_expected_bytes) {
			self->rx_sm_state = RECEIVER_STATE_IDLE;
			udata.pkt_size = self->rx_received_bytes;
			call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_END, udata);
		}
		return 1;
	}

	return 0;
}

int inv_iddwrapper_transport_uart_tx(inv_iddwrapper_transport_uart_t * self,
	const uint8_t * buffer, uint16_t size)
{
	union inv_iddwrapper_transport_event_data udata;
	const uint32_t total_bytes = 4 + size;
	uint16_t i;

	udata.tx_start = total_bytes;
	call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_START, udata);
	udata.tx_byte = SYNC_BYTE_0;
	call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE, udata);
	udata.tx_byte = SYNC_BYTE_1;
	call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE, udata);
	udata.tx_byte = (size & 0x00FF);
	call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE, udata);
	udata.tx_byte = (size & 0xFF00) >> 8;
	call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE, udata);

	for(i = 0; i < size; ++i) {
		udata.tx_byte = buffer[i];
		call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE, udata);		
	}

	call_event_cb(self, INV_IDDWRAPPER_TRANSPORT_EVENT_TX_END, udata);

	return 0;
}
