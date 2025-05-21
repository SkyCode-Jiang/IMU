#ifndef _USART_H
#define _USART_H
//#include "sys.h"
#include "stdio.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口1初始化
//正点原子@ALIENTEK
//技术论坛:www.openedv.csom
//修改日期:2017/4/6
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.0修改说明
//////////////////////////////////////////////////////////////////////////////////
#define EN_USART1_RX 1
#define USART_REC_LEN 100
#define RXBUFFERSIZE 100

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);


#endif
