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

/** @defgroup Uart peripheral
	@ingroup  Driver
	@{
*/
#ifndef __UART_H__
#define __UART_H__

#include <stdint.h>

#include "stm32f4xx.h"
#include "Invn/EmbUtils/RingByteBuffer.h"

typedef enum uart_num {
	UART_MAIN,     /**< USB - USART2 */
	UART_LOG,      /**< USART6 */
	UART_LOG_HDSK  /**< USART1 with handshakes */
}uart_num_t;

typedef struct uart_struct_t {
	int irqs_on;
	int hw_flowcontrol_on;
	USART_TypeDef * usartx;
	RingByteBuffer * ptr_TxBuffer;
	RingByteBuffer * ptr_RxBuffer;
}uart_struct_t;

/** @brief Configures UART peripheral to communicate with host
* @param[in] uart       UART peripheral
* @param[in] tx_buffer  memory buffer allocated for UART TX FIFO
* @param[in] rx_buffer  memory buffer allocated for UART RX FIFO
* @param[in] tx_size    buffer size for UART TX FIFO
* @param[in] rx_size    buffer size for UART RX FIFO
* @param[in] baudrate   UART baudrate speed
* @param[in] irqs_on    UART RX/TX IRQs enable option
* @return 0 on success, negative value otherwise
*/
int uart_config(uart_num_t uart, uint8_t * tx_buffer, uint8_t * rx_buffer,
        uint16_t tx_size, uint16_t rx_size, int baudrate, int irqs_on);

/** @brief Prints a text on UART 
* @param[in] uart UART peripheral
* @param[in] s    the text to be sent to the console
*/
int uart_puts(uart_num_t uart, const char * s);

/** @brief Prints a character on UART 
* @param[in] uart UART peripheral
* @param[in] ch   character be sent to the console
*/
int uart_putc(uart_num_t uart, int ch);

/** @brief Gets a character received on UART 
* @param[in] uart     UART peripheral
*/
int uart_getc(uart_num_t uart);

/** @brief Gets the number of bytes available received on UART. This API is only relevant when using IRQ.
* @param[in] uart     UART peripheral
* @return byte number available 
*/
uint8_t uart_available(uart_num_t uart);

/** @brief Return the HW flow control configuration for the UART
* @param[in] uart     UART peripheral
* @return 0 is HW flow control actived, -1 otherwise 
*/
int uart_get_hw_flow_control_configuration(uart_num_t uart);

#endif // __UART_H__

/** @} */
