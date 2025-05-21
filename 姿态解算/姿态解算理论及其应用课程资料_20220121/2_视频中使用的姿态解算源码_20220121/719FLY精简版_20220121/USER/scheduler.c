/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/

#include "stm32f10x.h"
#include "delay.h"
#include "main.h"
#include "stdio.h"
#include "led.h"
#include "remotedata.h"
#include "rc.h"
#include "string.h"

//�����������ִ�б�־λ
uint8_t LED_Scan = 0;
uint8_t IMU_Scan = 0;
uint8_t MPU_Scan = 0;
uint8_t IRQ_Scan = 0;
uint8_t Batt_Scan = 0;
uint8_t muartWaitFlag = 0;
uint8_t lock = 1;
uint8_t IMU_Init_Flag = 0;
uint8_t Init_Time = 0;

extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure_1;
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure_2;
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure_3;
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure_4;
extern TIM_ICUserValueTypeDef TIM_ICUserValueStructure_5;

extern uint8_t MPU6050_OffSet_Flag;

/****************************************************************************************************
* ��  ��: void USART1_IRQHandler(void)
* ��  ��: USART1�жϺ���
* ��  ��: ��
* ����ֵ: ��
* ��  ע: 
****************************************************************************************************/
void USART1_IRQHandler(void)
{
	uint8_t clear = clear; 																				//���������������Ա�����֡�û���õ�����������ľ�����ʾ
	
	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 				//�����ж�
	{ 
		
	}
	else if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET) 	//�����ж�
	{
		clear = USART1->SR; 																				//��SR�Ĵ���
		clear = USART1->DR; 																				//��DR�Ĵ������ȶ�SR,�ٶ�DR,����Ϊ�����IDIE�жϣ�
		
	}
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

/****************************************************************************************************
* ��  ��: void TIM4_IRQHandler(void) 
* ��  ��: TIM4��ʱ���жϣ�1ms��һ���ж�Ҳ����1000Hz
* ��  ��: ��
* ����ֵ: ��
* ��  ע: �˺������������������ʱ������ͬ���ж�ʱ���Ӧ��ͬƵ�ʣ�
*         ����һЩ����Ե���ʱ��Ҫ��Ƚ��ϸ�ʱ���ô˷�����
*         ɨ��Ƶ�� = 1000Hz/��Ƶϵ����
****************************************************************************************************/
void TIM4_IRQHandler(void)   //TIM4�жϷ�����
{
	static uint16_t ms2 = 0,ms5 = 0,ms10 = 0,ms20 = 0,ms100 = 0,ms200 = 0,ms400 = 0; //��Ƶϵ��
	if(TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET)	//�ж��Ƿ����TIM�����ж�
	{
		ms2++;
		ms5++;
		ms10++;	
		ms20++;	
		ms100++;
		ms200++;
		ms400++;  																//��������־��������������������Ƶ��
		if(ms2 >= 2)															//500Hz
		{
			ms2 = 0;
		}
		if(ms5 >= 5)															//200Hz
		{
			ms5 = 0;
			MPU_Scan = 1;
		}
		if(ms10 >= 10)														//100Hz
		{
			ms10 = 0;
			IMU_Scan = 1;
			
		}
		if(ms20 >= 20)														//50Hz
		{
			ms20 = 0;
		}
		if(ms100 >= 100)													//10Hz
		{
				ms100 = 0;
			
//				LED_Scan = 1;
			
				if( IMU_Init_Flag == 0 )
				{
					Init_Time ++;
				}
				
				if( Init_Time >= 100 && Att_Angle.pit < 0.3 && Att_Angle.rol <0.3 )//��ʼ���ȴ�ʱ���������IMU����Ƕ���������Χ��
				{
					Init_Time = 0;
					IMU_Init_Flag = 1;//��ʼ����־λΪ1����ʼ��ӡ
				}		
		}
		
		if(ms200 >= 200)													//5Hz
		{
			ms200 = 0;
			
			
//			OLED_ShowString(6,2,"pit",16);
//			OLED_Showdecimal(46,2,Att_Angle.pit,3,2,16);
//			OLED_ShowString(6,4,"rol",16);
//			OLED_Showdecimal(46,4,Att_Angle.rol,3,2,16);
//			OLED_ShowString(6,6,"yaw",16);
//			OLED_Showdecimal(46,6,Att_Angle.yaw,3,2,16);
//      OLED_Clear();
//      OLED��ʾ�����������Ƕ�
//			OLED_ShowString(6,4,"Tar",16);	//�����ڵ���Ŀ�����
//			OLED_Showdecimal(46,4,PID_ROL_Angle.OutPut,3,1,16);

		}
		if(ms400 >= 400)													//2.5Hz
		{
			ms400 = 0;
		}
	}
	TIM_ClearITPendingBit(TIM4,TIM_IT_Update);	//���TIM4�����ж�
}

/*****************************************************************************
* ��  ����void TIM2_IRQHandler (void)
* ��  �ܣ�TIM2���жϷ�����
* ��  ������
* ����ֵ����
* ��  ע:
*****************************************************************************/
void TIM2_IRQHandler (void)										//TIM2���жϷ�����  �����������4�����벶��ָ����жϺ������ø��Եı�־λ��������
{
	
	if(TIM_GetITStatus (TIM2 ,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit (TIM2 ,TIM_FLAG_Update);
	}	
	
	if(TIM_GetITStatus (TIM2 ,TIM_IT_CC1) != RESET)
	{
		if(TIM_ICUserValueStructure_1.Capture_StartFlag ==0)
		{
			TIM_ICUserValueStructure_1.Capture_CcrValue_a = TIM_GetCapture1 (TIM2 );
			TIM_OC1PolarityConfig(TIM2 , TIM_ICPolarity_Falling);
			TIM_ICUserValueStructure_1.Capture_StartFlag = 1;
		}
		else
		{
			TIM_ICUserValueStructure_1.Capture_CcrValue_b = TIM_GetCapture1 (TIM2 );
			TIM_OC1PolarityConfig(TIM2 ,TIM_ICPolarity_Rising);
			TIM_ICUserValueStructure_1.Capture_StartFlag = 0;
			TIM_ICUserValueStructure_1.Capture_FinishFlag = 1;
		}
		TIM_ClearITPendingBit (TIM2 ,TIM_IT_CC1 );
	}
	
	if(TIM_GetITStatus (TIM2 ,TIM_IT_CC2) != RESET)
	{
		if(TIM_ICUserValueStructure_2.Capture_StartFlag ==0)
		{
			TIM_ICUserValueStructure_2.Capture_CcrValue_a = TIM_GetCapture2 (TIM2 );
			TIM_OC2PolarityConfig(TIM2 , TIM_ICPolarity_Falling);
			TIM_ICUserValueStructure_2.Capture_StartFlag = 1;
		}
		else
		{
			TIM_ICUserValueStructure_2.Capture_CcrValue_b = TIM_GetCapture2 (TIM2 );
			TIM_OC2PolarityConfig(TIM2 ,TIM_ICPolarity_Rising);
			TIM_ICUserValueStructure_2.Capture_StartFlag = 0;
			TIM_ICUserValueStructure_2.Capture_FinishFlag = 1;
		}
		TIM_ClearITPendingBit (TIM2 ,TIM_IT_CC2 );
	}
	
	if(TIM_GetITStatus (TIM2 ,TIM_IT_CC3) != RESET)
	{
		if(TIM_ICUserValueStructure_3.Capture_StartFlag ==0)
		{
			TIM_ICUserValueStructure_3.Capture_CcrValue_a = TIM_GetCapture3 (TIM2 );
			TIM_OC3PolarityConfig(TIM2 , TIM_ICPolarity_Falling);
			TIM_ICUserValueStructure_3.Capture_StartFlag = 1;
		}
		else
		{
			TIM_ICUserValueStructure_3.Capture_CcrValue_b = TIM_GetCapture3 (TIM2 );
			TIM_OC3PolarityConfig(TIM2 ,TIM_ICPolarity_Rising);
			TIM_ICUserValueStructure_3.Capture_StartFlag = 0;
			TIM_ICUserValueStructure_3.Capture_FinishFlag = 1;
		}
		TIM_ClearITPendingBit (TIM2 ,TIM_IT_CC3 );
	}
	if(TIM_GetITStatus (TIM2 ,TIM_IT_CC4) != RESET)
	{
		if(TIM_ICUserValueStructure_4.Capture_StartFlag ==0)
		{
			TIM_ICUserValueStructure_4.Capture_CcrValue_a = TIM_GetCapture4 (TIM2 );
			TIM_OC4PolarityConfig(TIM2 , TIM_ICPolarity_Falling);
			TIM_ICUserValueStructure_4.Capture_StartFlag = 1;
		}
		
		else
		{
			TIM_ICUserValueStructure_4.Capture_CcrValue_b = TIM_GetCapture4 (TIM2 );
			TIM_OC4PolarityConfig(TIM2 ,TIM_ICPolarity_Rising);
			TIM_ICUserValueStructure_4.Capture_StartFlag = 0;
			TIM_ICUserValueStructure_4.Capture_FinishFlag = 1;
		}
		TIM_ClearITPendingBit (TIM2 ,TIM_IT_CC4 );
	}
	Remote_Data_ReceiveAnalysis();																		//���������������RC_Control��������4�����Է����һ������������
}

/*****************************************************************************
* ��  ����void TIM1_CC_IRQHandler (void)
* ��  �ܣ�TIM1���жϷ�����
* ��  ������
* ����ֵ����
* ��  ע:
*****************************************************************************/
void TIM1_CC_IRQHandler (void)																																												//TIM1���벶���жϷ�����  �����Ǹ߼���ʱ�������жϷ��������ֱ���ͨ��ʱ�������������TIM4��ϸ��
{
	
if(TIM_GetITStatus (TIM1 ,TIM_IT_CC1) != RESET)
	{
		if(TIM_ICUserValueStructure_5.Capture_StartFlag ==0)
		{
			TIM_ICUserValueStructure_5.Capture_CcrValue_a = TIM_GetCapture1 (TIM1 );
			TIM_ICUserValueStructure_5.Capture_Period = 0;
			TIM_OC1PolarityConfig(TIM1 , TIM_ICPolarity_Falling);
			TIM_ICUserValueStructure_5.Capture_StartFlag = 1;
		}
		else
		{
			TIM_ICUserValueStructure_5.Capture_CcrValue_b = TIM_GetCapture1 (TIM1 );
			TIM_OC1PolarityConfig(TIM1 ,TIM_ICPolarity_Rising);
			TIM_ICUserValueStructure_5.Capture_StartFlag = 0;
			TIM_ICUserValueStructure_5.Capture_FinishFlag = 1;
		}
		TIM_ClearITPendingBit (TIM1 ,TIM_IT_CC1 );
	}
	
	if((TIM_ICUserValueStructure_5.Capture_CcrValue_b-TIM_ICUserValueStructure_5.Capture_CcrValue_a)<2100&& (TIM_ICUserValueStructure_5.Capture_CcrValue_b-TIM_ICUserValueStructure_5.Capture_CcrValue_a)>900)
			RC_Control.BUTTON = TIM_ICUserValueStructure_5.Capture_CcrValue_b-TIM_ICUserValueStructure_5.Capture_CcrValue_a;//���������������RC_Control����	    printf("%d\r\n",RC_Control.BUTTON);
//			printf("%d\n",RC_Control.BUTTON);
}



