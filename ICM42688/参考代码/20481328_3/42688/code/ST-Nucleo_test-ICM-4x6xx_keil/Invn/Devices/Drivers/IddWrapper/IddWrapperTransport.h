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

#ifndef _IDDWRAPPER_TRANSPORT_H_
#define _IDDWRAPPER_TRANSPORT_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum inv_iddwrapper_transport_event {
	INV_IDDWRAPPER_TRANSPORT_EVENT_ERROR,
	INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_SIZE,
	INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_BYTE,
	INV_IDDWRAPPER_TRANSPORT_EVENT_PKT_END,
	INV_IDDWRAPPER_TRANSPORT_EVENT_TX_START,
	INV_IDDWRAPPER_TRANSPORT_EVENT_TX_BYTE,
	INV_IDDWRAPPER_TRANSPORT_EVENT_TX_END,
};

union inv_iddwrapper_transport_event_data {
	int error;
	uint16_t pkt_size;
	uint8_t  pkt_byte;
	uint32_t tx_start;
	uint8_t  tx_byte;
};

typedef void (*inv_iddwrapper_transport_event_cb)(enum inv_iddwrapper_transport_event e,
	union inv_iddwrapper_transport_event_data data, void * cookie);

#ifdef __cplusplus
}
#endif

#endif /* _IDDWRAPPER_TRANSPORT_H_ */
