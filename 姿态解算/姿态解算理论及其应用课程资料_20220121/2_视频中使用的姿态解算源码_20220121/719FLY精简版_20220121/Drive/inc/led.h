/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-9-6
* �������ߣ�719������ʵ����
************************************************************************************************/
#ifndef   _LED_H
#define   _LED_H

#include "main.h"
#define LEDR_L GPIOB->BSRR |= GPIO_Pin_12 //����LED����Ϊ �ߵ�ƽ
#define LEDR_H GPIOB->BRR  |= GPIO_Pin_12 //����LED����Ϊ �͵�ƽ

#define LEDG_L GPIOB->BSRR |= GPIO_Pin_13 //����LED����Ϊ �ߵ�ƽ
#define LEDG_H GPIOB->BRR  |= GPIO_Pin_13 //����LED����Ϊ �͵�ƽ

#define LEDB_L GPIOB->BSRR |= GPIO_Pin_14 //����LED����Ϊ �ߵ�ƽ
#define LEDB_H GPIOB->BRR  |= GPIO_Pin_14 //����LED����Ϊ �͵�ƽ

void LED_Init(void);
void LEDR_1(void);
void LEDR_2(void);
void LEDR_3(void);

void LEDG_1(void);
void LEDG_2(void);
void LEDG_3(void);

void LEDB_1(void);
void LEDB_2(void);
void LEDB_3(void);
#endif

