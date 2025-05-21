/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/

#include "led.h"
uint8_t loop=0;
/************************************************************************************************
* ��  ����void LED_Init(void)
* ��  �ܣ��û�ָʾ�����ų�ʼ��
* ��  ������
* ����ֵ����
* ��  ע: ��
************************************************************************************************/
void LED_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStruct;   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);   
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);  
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14; 
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;											//�������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP;											//�������
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
}


void LEDR_1(void)
{
		LEDR_L ;
		Delay_ms (100);
		LEDR_H ;
		Delay_ms (100);
}

void LEDR_2(void)
{
		LEDR_L ;
		Delay_ms (100);
		LEDR_H ;
		Delay_ms (100);
		LEDR_L ;
		Delay_ms (100);
		LEDR_H ;
		Delay_ms (100);
}

void LEDR_3(void)
{
		LEDR_L ;
		Delay_ms (100);
		LEDR_H ;
		Delay_ms (100);
		LEDR_L ;
		Delay_ms (100);
		LEDR_H ;
		Delay_ms (100);
		LEDR_L ;
		Delay_ms (100);
		LEDR_H ;
		Delay_ms (100);
}


void LEDG_1(void)
{
		LEDG_L ;
		Delay_ms (100);
		LEDG_H ;
		Delay_ms (100);
}

void LEDG_2(void)
{
		LEDG_L ;
		Delay_ms (100);
		LEDG_H ;
		Delay_ms (100);
		LEDG_L ;
		Delay_ms (100);
		LEDG_H ;
		Delay_ms (100);
}

void LEDG_3(void)
{
		LEDG_L ;
		Delay_ms (100);
		LEDG_H ;
		Delay_ms (100);
		LEDG_L ;
		Delay_ms (100);
		LEDG_H ;
		Delay_ms (100);
		LEDG_L ;
		Delay_ms (100);
		LEDG_H ;
		Delay_ms (100);
}

void LEDB_1(void)
{
		LEDB_L ;
		Delay_ms (100);
		LEDB_H ;
		Delay_ms (100);
}

void LEDB_2(void)
{
		LEDB_L ;
		Delay_ms (100);
		LEDB_H ;
		Delay_ms (100);
		LEDB_L ;
		Delay_ms (100);
		LEDB_H ;
		Delay_ms (100);
}

void LEDB_3(void)
{
		LEDB_L ;
		Delay_ms (100);
		LEDB_H ;
		Delay_ms (100);
		LEDB_L ;
		Delay_ms (100);
		LEDB_H ;
		Delay_ms (100);
		LEDB_L ;
		Delay_ms (100);
		LEDB_H ;
		Delay_ms (100);
}


