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

#ifndef _IDDWRAPPER_TRANSPORT_UART_H_
#define _IDDWRAPPER_TRANSPORT_UART_H_

#include "Invn/InvExport.h"

#include "Invn/Devices/Drivers/IddWrapper/IddWrapperTransport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct inv_iddwrapper_transport_uart {
	inv_iddwrapper_transport_event_cb event_cb;
	void *                   event_cb_cookie;


	enum inv_iddwrapper_transport_rcv_sm {
		RECEIVER_STATE_IDLE   = 0,
		RECEIVER_STATE_SYNC_0 = RECEIVER_STATE_IDLE,
		RECEIVER_STATE_SYNC_1,
		RECEIVER_STATE_SIZE_BYTE_0,
		RECEIVER_STATE_SIZE_BYTE_1,
		RECEIVER_STATE_PACKET_DATA
	} rx_sm_state;
	uint16_t rx_expected_bytes;
	uint16_t rx_received_bytes;
} inv_iddwrapper_transport_uart_t;

void INV_EXPORT inv_iddwrapper_transport_uart_init(inv_iddwrapper_transport_uart_t * self,
		inv_iddwrapper_transport_event_cb event_cb, void * cookie);

void INV_EXPORT inv_iddwrapper_transport_uart_rx_process_reset(inv_iddwrapper_transport_uart_t * self);
int INV_EXPORT inv_iddwrapper_transport_uart_rx_process_byte(inv_iddwrapper_transport_uart_t * self, uint8_t rcv_byte);

int INV_EXPORT inv_iddwrapper_transport_uart_tx(inv_iddwrapper_transport_uart_t * self,
	const uint8_t * buffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* _IDDWRAPPER_TRANSPORT_UART_H_ */
