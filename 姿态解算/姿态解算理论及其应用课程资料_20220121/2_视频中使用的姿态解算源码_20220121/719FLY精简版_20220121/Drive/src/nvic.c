/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/
#include "stm32f10x.h"

/*****************************************************************************
* ��  ����void NvicConfig(void)
* ��  �ܣ����ù����������жϵ����ȼ�
* ��  ������
* ����ֵ����
* ��  ע�������ȼ��жϲ�Ҫ������Ŷ
*****************************************************************************/
void NvicConfig(void)
{
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	
	NVIC_InitStruct.NVIC_IRQChannel=TIM4_IRQn;   						//TIM4�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=1;   	//��ռ���ȼ�0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;   				//�����ȼ�1
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;   						//ʹ��TIM4�ж�ͨ��
	NVIC_Init(&NVIC_InitStruct);   													//�ж����ȼ���ʼ������
	
	NVIC_InitStruct.NVIC_IRQChannel=USART1_IRQn;						//����1�ж�ͨ��  ��������λ��ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=1;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel=EXTI2_IRQn;   					//�����ⲿ�ж�ͨ��
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=2;   	//������ռ���ȼ�Ϊ0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=2;   				//���������ȼ�Ϊ1
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;   						//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStruct);   													//�ж����ȼ���ʼ������
																
	NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn ;						//�����ж���Դ��TIM2����ͨ��ʱ�������жϹ���һ���������������ڲ�ͨ�����Եı�־λ�������֣�
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;		//���������ȼ�Ϊ0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;		      //������ռ���ȼ�Ϊ3
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
																		
	NVIC_InitStruct.NVIC_IRQChannel = TIM1_CC_IRQn ;				//�����ж���Դ:TIM1���벶�񣨸߼���ʱ���жϷ��񻮷ָ�ϸ�£�
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;		//���������ȼ�Ϊ0
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 2;					//������ռ���ȼ�Ϊ2
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel = USART3_IRQn;           //�����ж���Դ:USART3
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;   
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
	
}

