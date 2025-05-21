/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/
#include "rc.h"

TIM_ICUserValueTypeDef TIM_ICUserValueStructure_1;
TIM_ICUserValueTypeDef TIM_ICUserValueStructure_2;
TIM_ICUserValueTypeDef TIM_ICUserValueStructure_3;
TIM_ICUserValueTypeDef TIM_ICUserValueStructure_4;
TIM_ICUserValueTypeDef TIM_ICUserValueStructure_5;
RC_TYPE RC_Control;

//GPIO��ʼ��
static void GENERAL_TIM_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(GENERAL_TIM_GPIO_CLK ,ENABLE);
	GPIO_InitStructure.GPIO_Pin 	= 	GENERAL_TIM_CH1_PIN|GENERAL_TIM_CH2_PIN|GENERAL_TIM_CH3_PIN|GENERAL_TIM_CH4_PIN|GENERAL_TIM_CH5_PIN;
	GPIO_InitStructure.GPIO_Mode 	= 	GPIO_Mode_IN_FLOATING;
	GPIO_Init(GENERAL_TIM_PORT ,&GPIO_InitStructure);
}


//��ʱ����ʼ��
static void GENERAL_TIM_Mode_Config(void)
{
	GENERAL_TIM_APB1Clock_FUN(GENERAL_TIM2_CLK ,ENABLE);												//������ʱ��ʱ�ӣ��ڲ�ʱ��CK_INT=72M
	GENERAL_TIM_APB2Clock_FUN(GENERAL_TIM1_CLK ,ENABLE);												//������ʱ��ʱ�ӣ��ڲ�ʱ��CK_INT=72M
	//ʱ���ṹ���ʼ��
	    //TIM2
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;														
	TIM_TimeBaseStructure.TIM_Period = GENERAL_TIM_Period ;										//�Զ���װ�ؼĴ���ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»��ж�
	TIM_TimeBaseStructure.TIM_Prescaler = GENERAL_TIM_Prescaler;							//����CNT������ʱ��=Fck_int/(psc+1)
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;										//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode= TIM_CounterMode_Up;								//���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;													//�ظ���������ֵ
	TIM_TimeBaseInit(TIM2 ,&TIM_TimeBaseStructure);										//��ʼ����ʱ��
			//TIM1
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure2;														
	TIM_TimeBaseStructure2.TIM_Period = GENERAL_TIM_Period ;										//�Զ���װ�ؼĴ���ֵ���ۼ�TIM_Period+1��Ƶ�ʺ����һ�����»��ж�
	TIM_TimeBaseStructure2.TIM_Prescaler = GENERAL_TIM_Prescaler;							//����CNT������ʱ��=Fck_int/(psc+1)
	TIM_TimeBaseStructure2.TIM_ClockDivision = TIM_CKD_DIV1;										//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure2.TIM_CounterMode= TIM_CounterMode_Up;								//���ϼ���ģʽ
	TIM_TimeBaseStructure2.TIM_RepetitionCounter = 0;													//�ظ���������ֵ
	TIM_TimeBaseInit(TIM1 ,&TIM_TimeBaseStructure2);										//��ʼ����ʱ��
	
	//���벶��    �˴�δ�������벶��Ĵ�����ʽ�������ز���/�½��ز��񣩣���Ϊ��Ҫ���жϷ���������з�ת��������/�½��ش������벶��->���벶�񴥷��жϣ�
		//TIM2CH1
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = GENERAL_TIM_CHANNEL_1;									//�������벶���ͨ��
	TIM_ICInitStructure.TIM_ICPolarity = GENERAL_TIM_START_ICPolarity ;				//���벶���źż���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;						//����ͨ���Ͳ���ͨ��ӳ���ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;											//����ı������źŵķ�Ƶϵ��
	TIM_ICInitStructure.TIM_ICFilter = 0;																			//����ı������źŵ��˲�ϵ��
	TIM_ICInit(TIM2 , &TIM_ICInitStructure);														//��ʱ�����벶���ʼ��
		//TIM2CH2
	TIM_ICInitStructure.TIM_Channel = GENERAL_TIM_CHANNEL_2;									//�������벶���ͨ��
	TIM_ICInitStructure.TIM_ICPolarity = GENERAL_TIM_START_ICPolarity ;				//���벶���źż���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;						//����ͨ���Ͳ���ͨ��ӳ���ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;											//����ı������źŵķ�Ƶϵ��
	TIM_ICInitStructure.TIM_ICFilter = 0;																			//����ı������źŵ��˲�ϵ��
	TIM_ICInit(TIM2 , &TIM_ICInitStructure);														//��ʱ�����벶���ʼ��
		//TIM2CH3
	TIM_ICInitStructure.TIM_Channel = GENERAL_TIM_CHANNEL_3;									//�������벶���ͨ��
	TIM_ICInitStructure.TIM_ICPolarity = GENERAL_TIM_START_ICPolarity ;				//���벶���źż���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;						//����ͨ���Ͳ���ͨ��ӳ���ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;											//����ı������źŵķ�Ƶϵ��
	TIM_ICInitStructure.TIM_ICFilter = 0;																			//����ı������źŵ��˲�ϵ��
	TIM_ICInit(TIM2 , &TIM_ICInitStructure);														//��ʱ�����벶���ʼ��
		//TIM2CH4
	TIM_ICInitStructure.TIM_Channel = GENERAL_TIM_CHANNEL_4;									//�������벶���ͨ��
	TIM_ICInitStructure.TIM_ICPolarity = GENERAL_TIM_START_ICPolarity ;				//���벶���źż���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;						//����ͨ���Ͳ���ͨ��ӳ���ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;											//����ı������źŵķ�Ƶϵ��
	TIM_ICInitStructure.TIM_ICFilter = 0;																			//����ı������źŵ��˲�ϵ��
	TIM_ICInit(TIM2 , &TIM_ICInitStructure);														//��ʱ�����벶���ʼ��
		//TIM1CH1
	TIM_ICInitStructure.TIM_Channel = GENERAL_TIM_CHANNEL_1;									//�������벶���ͨ��
	TIM_ICInitStructure.TIM_ICPolarity = GENERAL_TIM_START_ICPolarity ;				//���벶���źż���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;						//����ͨ���Ͳ���ͨ��ӳ���ϵ
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;											//����ı������źŵķ�Ƶϵ��
	TIM_ICInitStructure.TIM_ICFilter = 0;																			//����ı������źŵ��˲�ϵ��
	TIM_ICInit(TIM1 , &TIM_ICInitStructure);														//��ʱ�����벶���ʼ��
	
	
	
	//������ºͲ����жϱ�־λ
			//TIM2
	TIM_ClearFlag(TIM2 , TIM_FLAG_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4 );					
			//TIM1
	TIM_ClearFlag(TIM1 , TIM_FLAG_Update|TIM_IT_CC1 );					
	//�������ºͲ����ж�
			//TIM2
	TIM_ITConfig(TIM2 ,TIM_IT_Update|TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4 ,ENABLE);			
			//TIM1
	TIM_ITConfig(TIM1 ,TIM_IT_Update|TIM_IT_CC1,ENABLE);			
	//ʹ�ܼ�����
			//TIM2
	TIM_Cmd(TIM2 ,ENABLE);																						
			//TIM1
	TIM_Cmd(TIM1 ,ENABLE);																							
}


void GENERAL_TIM_Init(void)
{
	GENERAL_TIM_GPIO_Config();
	GENERAL_TIM_Mode_Config();
}

void Remote_Data_ReceiveAnalysis(void)
{
	int16_t the1;
	int16_t the2;
	int16_t the3;
	int16_t the4;
	
	
			//if((TIM_ICUserValueStructure_1.Capture_CcrValue_b-TIM_ICUserValueStructure_1.Capture_CcrValue_a)<2100&&(TIM_ICUserValueStructure_1.Capture_CcrValue_b-TIM_ICUserValueStructure_1.Capture_CcrValue_a)>900)
			the1 = TIM_ICUserValueStructure_1.Capture_CcrValue_b - TIM_ICUserValueStructure_1.Capture_CcrValue_a;
			if(the1 < 0) the1 += 65536;
			if(the1 < 2100 && the1 > 900)
			{
				RC_Control.ROLL = Lowpass_Filter_ROLL(the1);
//			printf("%d,%d\n",the1,RC_Control.ROLL);
			}
			//if((TIM_ICUserValueStructure_2.Capture_CcrValue_b-TIM_ICUserValueStructure_2.Capture_CcrValue_a)<2100&&(TIM_ICUserValueStructure_2.Capture_CcrValue_b-TIM_ICUserValueStructure_2.Capture_CcrValue_a)>900)
			the2 = TIM_ICUserValueStructure_2.Capture_CcrValue_b - TIM_ICUserValueStructure_2.Capture_CcrValue_a;
			if(the2 < 0) the2 += 65536;
			if(the2 < 2100 && the2 > 900)
			{
				RC_Control.PITCH = Lowpass_Filter_PITCH(the2);
//			  printf("%d,%d\n",the2,RC_Control.PITCH);
			}
			//if((TIM_ICUserValueStructure_3.Capture_CcrValue_b-TIM_ICUserValueStructure_3.Capture_CcrValue_a)<2100&&(TIM_ICUserValueStructure_3.Capture_CcrValue_b-TIM_ICUserValueStructure_3.Capture_CcrValue_a)>900)
	   	the3 = TIM_ICUserValueStructure_3.Capture_CcrValue_b - TIM_ICUserValueStructure_3.Capture_CcrValue_a;
			if(the3 < 0) the3 += 65536;
			if(the3 < 2100 && the3 > 900)
			{	
				RC_Control.THROTTLE = Lowpass_Filter_THROTTLE(the3);
//			printf("%d,%d\n",the3,RC_Control.THROTTLE);
			}

			//if((TIM_ICUserValueStructure_4.Capture_CcrValue_b-TIM_ICUserValueStructure_4.Capture_CcrValue_a)<2100&& (TIM_ICUserValueStructure_4.Capture_CcrValue_b-TIM_ICUserValueStructure_4.Capture_CcrValue_a)>900)
			the4 = TIM_ICUserValueStructure_4.Capture_CcrValue_b - TIM_ICUserValueStructure_4.Capture_CcrValue_a;
			if(the4 < 0) the4 += 65536;
			if(the4 < 2100 && the4 > 900)
			{
				RC_Control.YAW = Lowpass_Filter_YAW(the4);
//		  printf("%d,%d\n",the4,RC_Control.YAW);
			}
}
