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

#include "uart.h"

#include <stdio.h>

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h" 
#include "stm32f4xx_exti.h"
#include "stm32f4xx_rcc.h"

/********************************* Defines ************************************/

//////////////////  USART1
#define USARTx_HDSK                      USART1
#define USARTx_HDSK_CLK                  RCC_APB2Periph_USART1
#define USARTx_HDSK_CLK_INIT             RCC_APB2PeriphClockCmd
#define USARTx_HDSK_IRQn                 USART1_IRQn

#define USARTx_HDSK_TX_PIN               GPIO_Pin_9
#define USARTx_HDSK_TX_GPIO_PORT         GPIOA
#define USARTx_HDSK_TX_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define USARTx_HDSK_TX_SOURCE            GPIO_PinSource9
#define USARTx_HDSK_TX_AF                GPIO_AF_USART1

#define USARTx_HDSK_RX_PIN               GPIO_Pin_10
#define USARTx_HDSK_RX_GPIO_PORT         GPIOA
#define USARTx_HDSK_RX_GPIO_CLK          RCC_AHB1Periph_GPIOA
#define USARTx_HDSK_RX_SOURCE            GPIO_PinSource10
#define USARTx_HDSK_RX_AF                GPIO_AF_USART1

#define USARTx_HDSK_CTS_PIN              GPIO_Pin_11
#define USARTx_HDSK_CTS_GPIO_PORT        GPIOA
#define USARTx_HDSK_CTS_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define USARTx_HDSK_CTS_SOURCE           GPIO_PinSource11
#define USARTx_HDSK_CTS_AF               GPIO_AF_USART1

#define USARTx_HDSK_RTS_PIN              GPIO_Pin_12
#define USARTx_HDSK_RTS_GPIO_PORT        GPIOA
#define USARTx_HDSK_RTS_GPIO_CLK         RCC_AHB1Periph_GPIOA
#define USARTx_HDSK_RTS_SOURCE           GPIO_PinSource12
#define USARTx_HDSK_RTS_AF               GPIO_AF_USART1

//////////////////  USART2
#define USARTx                           USART2
#define USARTx_CLK                       RCC_APB1Periph_USART2
#define USARTx_CLK_INIT                  RCC_APB1PeriphClockCmd
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

#define USARTx_TX_PIN                    GPIO_Pin_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_TX_SOURCE                 GPIO_PinSource2
#define USARTx_TX_AF                     GPIO_AF_USART2

#define USARTx_RX_PIN                    GPIO_Pin_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define USARTx_RX_SOURCE                 GPIO_PinSource3
#define USARTx_RX_AF                     GPIO_AF_USART2

//////////////////  USART6
#define USARTx_LOG                       USART6
#define USARTx_LOG_CLK                   RCC_APB2Periph_USART6
#define USARTx_LOG_CLK_INIT              RCC_APB2PeriphClockCmd
#define USARTx_LOG_IRQn                  USART6_IRQn

#define USARTx_LOG_TX_PIN                GPIO_Pin_11
#define USARTx_LOG_TX_GPIO_PORT          GPIOA
#define USARTx_LOG_TX_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define USARTx_LOG_TX_SOURCE             GPIO_PinSource11
#define USARTx_LOG_TX_AF                 GPIO_AF_USART6

#define USARTx_LOG_RX_PIN                GPIO_Pin_12
#define USARTx_LOG_RX_GPIO_PORT          GPIOA
#define USARTx_LOG_RX_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define USARTx_LOG_RX_SOURCE             GPIO_PinSource12
#define USARTx_LOG_RX_AF                 GPIO_AF_USART6

/********************************* Globals ************************************/

static RingByteBuffer sUARTBufferRx;
static RingByteBuffer sUARTBufferTx;
static RingByteBuffer sUARTLOGBufferTx;
static RingByteBuffer sUARTLOGBufferRx;
static RingByteBuffer sUARTHDSKBufferTx;
static RingByteBuffer sUARTHDSKBufferRx;

static uart_struct_t UART_main, UART_log, UART_hdsk;

/********************************* Prototypes *********************************/

static uart_struct_t * uart_get_instance(uart_num_t uart)
{
	void * pInstance = NULL;

	if(uart == UART_MAIN)
		pInstance = &UART_main;
	else if(uart == UART_LOG)
		pInstance = &UART_log;
	else if(uart == UART_LOG_HDSK)
		pInstance = &UART_hdsk;

	return pInstance;
}

int uart_config(uart_num_t uart, uint8_t * tx_buffer, uint8_t * rx_buffer,
        uint16_t tx_size, uint16_t rx_size, int baudrate, int irqs_on)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* interrupt handling needs memory buffer */
	if ((irqs_on) && ((tx_size == 0) && (rx_size == 0)))
		return -1;

	/* Configure USART GPIO as alternate function push-pull */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* Configure USART TX/RX - 8bits - 1Stop - No parity */
	USART_InitStructure.USART_BaudRate = baudrate;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* Configure USRAT interrupt */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	if(uart == UART_MAIN) {
		/* init structure */
		UART_main.irqs_on = irqs_on;
		UART_main.usartx = USARTx;
		UART_main.hw_flowcontrol_on = 0;
		RingByteBuffer_init(&sUARTBufferRx, rx_buffer, rx_size);
		RingByteBuffer_init(&sUARTBufferTx, tx_buffer, tx_size);
		UART_main.ptr_RxBuffer = &sUARTBufferRx;
		UART_main.ptr_TxBuffer = &sUARTBufferTx;

		/* Peripheral Clock Enable -------------------------------------------------*/
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(USARTx_TX_GPIO_CLK | USARTx_RX_GPIO_CLK, ENABLE);

		/* Enable USART clock */
		USARTx_CLK_INIT(USARTx_CLK, ENABLE);

		/* USARTx GPIO configuration -----------------------------------------------*/ 
		/* Connect USART pins to corresponding AF */
		GPIO_PinAFConfig(USARTx_TX_GPIO_PORT, USARTx_TX_SOURCE, USARTx_TX_AF);
		GPIO_PinAFConfig(USARTx_RX_GPIO_PORT, USARTx_RX_SOURCE, USARTx_RX_AF);

		/* Configure USART Tx and Rx as alternate function push-pull */
		GPIO_InitStructure.GPIO_Pin = USARTx_TX_PIN;
		GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = USARTx_RX_PIN;
		GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStructure);

		if(UART_main.irqs_on) {
			/* Enable the USARTx Interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = USARTx_IRQn;
			NVIC_Init(&NVIC_InitStructure);
		}

		USART_Init(USARTx, &USART_InitStructure);
		USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);

		/* Enable USART */
		USART_Cmd(USARTx, ENABLE);

	} else if(uart == UART_LOG) {
		/* init structure */
		UART_log.irqs_on = irqs_on;
		UART_log.hw_flowcontrol_on = 0;
		UART_log.usartx = USARTx_LOG;
		RingByteBuffer_init(&sUARTLOGBufferRx, rx_buffer, rx_size);
		RingByteBuffer_init(&sUARTLOGBufferTx, tx_buffer, tx_size);
		UART_log.ptr_RxBuffer = &sUARTLOGBufferRx;
		UART_log.ptr_TxBuffer = &sUARTLOGBufferTx;

		/* Peripheral Clock Enable -------------------------------------------------*/
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(USARTx_LOG_TX_GPIO_CLK | USARTx_LOG_RX_GPIO_CLK, ENABLE);

		/* Enable USART clock */
		USARTx_LOG_CLK_INIT(USARTx_LOG_CLK, ENABLE);

		/* USARTx GPIO configuration -----------------------------------------------*/ 
		/* Connect USART pins to corresponding AF */
		GPIO_PinAFConfig(USARTx_LOG_TX_GPIO_PORT, USARTx_LOG_TX_SOURCE, USARTx_LOG_TX_AF);
		GPIO_PinAFConfig(USARTx_LOG_RX_GPIO_PORT, USARTx_LOG_RX_SOURCE, USARTx_LOG_RX_AF);

		GPIO_InitStructure.GPIO_Pin = USARTx_LOG_TX_PIN;
		GPIO_Init(USARTx_LOG_TX_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = USARTx_LOG_RX_PIN;
		GPIO_Init(USARTx_LOG_RX_GPIO_PORT, &GPIO_InitStructure);

		if(UART_log.irqs_on) {
			/* Enable the USARTx Interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = USARTx_LOG_IRQn;
			NVIC_Init(&NVIC_InitStructure);
		}

		USART_Init(USARTx_LOG, &USART_InitStructure);
		USART_ITConfig(USARTx_LOG, USART_IT_RXNE, ENABLE);

		/* Enable USART */
		USART_Cmd(USARTx_LOG, ENABLE);

	} else if(uart == UART_LOG_HDSK) {
		/* init structure */
		UART_hdsk.irqs_on = irqs_on;
		UART_hdsk.hw_flowcontrol_on = 1;
		UART_hdsk.usartx = USARTx_HDSK;
		RingByteBuffer_init(&sUARTHDSKBufferRx, rx_buffer, rx_size);
		RingByteBuffer_init(&sUARTHDSKBufferTx, tx_buffer, tx_size);
		UART_hdsk.ptr_RxBuffer = &sUARTHDSKBufferRx;
		UART_hdsk.ptr_TxBuffer = &sUARTHDSKBufferTx;

		/* Peripheral Clock Enable -------------------------------------------------*/
		/* Enable GPIO clock */
		RCC_AHB1PeriphClockCmd(USARTx_HDSK_TX_GPIO_CLK | USARTx_HDSK_RX_GPIO_CLK 
				| USARTx_HDSK_CTS_GPIO_CLK | USARTx_HDSK_RTS_GPIO_CLK, ENABLE);

		/* Enable USART clock */
		USARTx_HDSK_CLK_INIT(USARTx_HDSK_CLK, ENABLE);

		/* USARTx GPIO configuration -----------------------------------------------*/ 
		/* Connect USART pins to corresponding AF */
		GPIO_PinAFConfig(USARTx_HDSK_TX_GPIO_PORT, USARTx_HDSK_TX_SOURCE, USARTx_HDSK_TX_AF);
		GPIO_PinAFConfig(USARTx_HDSK_RX_GPIO_PORT, USARTx_HDSK_RX_SOURCE, USARTx_HDSK_RX_AF);
		GPIO_PinAFConfig(USARTx_HDSK_CTS_GPIO_PORT, USARTx_HDSK_CTS_SOURCE, USARTx_HDSK_RX_AF);
		GPIO_PinAFConfig(USARTx_HDSK_RTS_GPIO_PORT, USARTx_HDSK_RTS_SOURCE, USARTx_HDSK_RX_AF);

		GPIO_InitStructure.GPIO_Pin = USARTx_HDSK_TX_PIN;
		GPIO_Init(USARTx_HDSK_TX_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = USARTx_HDSK_RX_PIN;
		GPIO_Init(USARTx_HDSK_RX_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = USARTx_HDSK_CTS_PIN;
		GPIO_Init(USARTx_HDSK_CTS_GPIO_PORT, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin = USARTx_HDSK_RTS_PIN;
		GPIO_Init(USARTx_HDSK_RTS_GPIO_PORT, &GPIO_InitStructure);

		if(UART_hdsk.irqs_on) {
			/* Enable the USARTx Interrupt */
			NVIC_InitStructure.NVIC_IRQChannel = USARTx_HDSK_IRQn;
			NVIC_Init(&NVIC_InitStructure);
		}

		/* Active Hardware Flow Control */
		USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;

		USART_Init(USARTx_HDSK, &USART_InitStructure);
		USART_ITConfig(USARTx_HDSK, USART_IT_RXNE, ENABLE);

		/* Enable USART */
		USART_Cmd(USARTx_HDSK, ENABLE);
	}
	return 0;
}

int uart_putc(uart_num_t uart, int ch)
{
	uart_struct_t * pInstance = uart_get_instance(uart);

	if(pInstance->irqs_on) {
		__disable_irq();
		if(!RingByteBuffer_isFull(pInstance->ptr_TxBuffer))
		{
			RingByteBuffer_pushByte(pInstance->ptr_TxBuffer, (uint8_t) ch);
			
			if(USART_GetITStatus(pInstance->usartx, USART_IT_TXE) == RESET)
				USART_ITConfig(pInstance->usartx, USART_IT_TXE, ENABLE);
		}
		else
		{
			//return error code or you will push incomplete packets into the buffer
			ch = EOF;
		}
		__enable_irq();
	} else {
		USART_SendData(pInstance->usartx, (uint8_t) ch);

		// Loop until the end of transmission 
		while (USART_GetFlagStatus(pInstance->usartx, USART_FLAG_TC) == RESET)
		{}
	}
	return ch;
}

int uart_getc(uart_num_t uart)
{
	uart_struct_t * pInstance = uart_get_instance(uart);

	if(pInstance->irqs_on) {
		int data = EOF;
		__disable_irq();
		if(!RingByteBuffer_isEmpty(pInstance->ptr_RxBuffer))
		{
			data = RingByteBuffer_popByte(pInstance->ptr_RxBuffer);
		}
		__enable_irq();
		return data;
	} else {
		if (USART_GetFlagStatus(pInstance->usartx, USART_FLAG_RXNE) == SET)
			return USART_ReceiveData(pInstance->usartx);
	}
	return EOF;
}

uint8_t uart_available(uart_num_t uart)
{
	uart_struct_t * pInstance = uart_get_instance(uart);

	if(pInstance->irqs_on) {
		uint8_t size;
		__disable_irq();
		size = RingByteBuffer_size(pInstance->ptr_RxBuffer);
		__enable_irq();
		return size;
	} else
		return 0;
}

int uart_puts(uart_num_t uart, const char * s)
{
	int n = 0;
	const char * pc = s;
    
    while(*pc != '\0')
    {
    	uart_putc(uart, *pc);
		++pc;
		++n;
	}

	return n;
}

int uart_get_hw_flow_control_configuration(uart_num_t uart)
{
	uart_struct_t * pInstance = uart_get_instance(uart);
	
	return pInstance->hw_flowcontrol_on;
}

/* Interrupt management ------------------------------------------------------*/

void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(UART_hdsk.usartx, USART_IT_RXNE) != RESET)
	{
		if(!RingByteBuffer_isFull(UART_hdsk.ptr_RxBuffer))
		{
			uint8_t lRxByte = USART_ReceiveData(UART_hdsk.usartx);
			RingByteBuffer_pushByte(UART_hdsk.ptr_RxBuffer, lRxByte);
		}
	}
	if(USART_GetITStatus(UART_hdsk.usartx, USART_IT_TXE) != RESET)
	{
		if(!RingByteBuffer_isEmpty(UART_hdsk.ptr_TxBuffer))
		{
			uint8_t lTxByte = RingByteBuffer_popByte(UART_hdsk.ptr_TxBuffer);
			USART_SendData(UART_hdsk.usartx, lTxByte);
		} 
		else
			USART_ITConfig(UART_hdsk.usartx, USART_IT_TXE, DISABLE);
	}
}

void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(UART_main.usartx, USART_IT_RXNE) != RESET)
	{
		if(!RingByteBuffer_isFull(UART_main.ptr_RxBuffer))
		{
			uint8_t lRxByte = USART_ReceiveData(UART_main.usartx);
			RingByteBuffer_pushByte(UART_main.ptr_RxBuffer, lRxByte);
		}
	}
	if(USART_GetITStatus(UART_main.usartx, USART_IT_TXE) != RESET)
	{
		if(!RingByteBuffer_isEmpty(UART_main.ptr_TxBuffer))
		{
			uint8_t lTxByte = RingByteBuffer_popByte(UART_main.ptr_TxBuffer);
			USART_SendData(UART_main.usartx, lTxByte);
		} 
		else
			USART_ITConfig(UART_main.usartx, USART_IT_TXE, DISABLE);
	}
}

void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(UART_log.usartx, USART_IT_RXNE) != RESET)
	{
		if(!RingByteBuffer_isFull(UART_log.ptr_RxBuffer))
		{
			uint8_t lRxByte = USART_ReceiveData(UART_log.usartx);
			RingByteBuffer_pushByte(UART_log.ptr_RxBuffer, lRxByte);
		}
	}
	if(USART_GetITStatus(UART_log.usartx, USART_IT_TXE) != RESET)
	{
		if(!RingByteBuffer_isEmpty(UART_log.ptr_TxBuffer))
		{
			uint8_t lTxByte = RingByteBuffer_popByte(UART_log.ptr_TxBuffer);
			USART_SendData(UART_log.usartx, lTxByte);
		} 
		else
			USART_ITConfig(UART_log.usartx, USART_IT_TXE, DISABLE);
	}
}

/* Embedded Utils hook implmentation ------------------------------------------*/

// needed for RingByteBuffer embedded utils
void InvAssert(const char *predicate, const char *file, unsigned line) 
{
	(void)predicate;
	(void)line;
	(void)file;
}
