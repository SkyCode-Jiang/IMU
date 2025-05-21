/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/

#include "main.h"
extern uint8_t IMU_Init_Flag;

/************************************************************************************************
* ��  ����void System_Init(void)
* ��  �ܣ���ʼ������
* ��  ����    
* ����ֵ���� 
* ��  ע��������ΪSTM32�ɿ��������������
************************************************************************************************/
void System_Init(void)
{
	LED_Init();										//״ָ̬ʾ�Ƴ�ʼ��
	NvicConfig(); 								//ϵͳ�ж����ȼ����������жϳ�ʼ����		
	Delay_Init(); 								//ϵͳ��ʱ��ʼ��
	USART_init(115200);						//���Դ��ڳ�ʼ��      
	IIC_GPIO_Init(); 							//ģ��IIC��ʼ��
	TIM_Init();										//ϵͳʱ����ʼ����TIM4��Ϊϵͳʱ��
	MPU6050_Init();	 							//MPU6050��ʼ��
  MOTOR_Init(); 								//��������ʼ��													
	PidParameter_init(); 					//PID������ʼ��								
	GENERAL_TIM_Init();						//TIM2��TIM1��ʼ�������жϳ�ʼ�� ���ڶԽ��ջ��źŽ������벶��
//	OLED_Clear();
//	OLED_Init();
	
//��ʾIIC��⵽�ĵ�ַ.l
//	uint8_t ID;
//	ID = MPU6050_getDeviceID();
//	OLED_ShowString(6,6,"ID",16);		
//	OLED_Showdecimal(46,6,ID,6,4,16);

}

/************************************************************************************************
* ��  ����void Task_Schedule(void)
* ��  �ܣ�������
* ��  ����    
* ����ֵ���� 
* ��  ע��������ΪSTM32�ɿ��������������
************************************************************************************************/
void Task_Schedule(void)
{
	
		if(IMU_Scan) 																												//100Hz
		{
			IMU_Scan  = 0;																									//��־λ����
			Prepare_Data(); 																								//��ȡ��̬������������
			IMUupdate(&Gyr_filt,&Acc_filt,&Att_Angle); 											//��Ԫ����̬����
			Control(&Att_Angle,&Gyr_filt,&RC_Control,Airplane_Enable); 			//��̬����
			
		}
		if(LED_Scan) //10Hz
		{
			
			LED_Scan = 0;
			if( IMU_Init_Flag == 0 )//��ʼ��������R����˸
			{
				LEDR_H;
				LEDG_H;
				LEDB_H;
				if(loop)
				{
					loop=0;
					LEDR_H;
				}
				else
				{
					loop=1;
					LEDR_L;
				}
			}
			
			if( lock && IMU_Init_Flag == 1 )//��ʼ����ɣ��ɻ�������ң�����ź�����,G����˸
			{
				LEDR_H;
				LEDG_H;
				LEDB_H;
				if(loop)
				{
					loop=0;
					LEDG_H;
				}
				else
				{
					loop=1;
					LEDG_L;
				}
			}
			if( !lock && IMU_Init_Flag == 1)//��ʼ����ɣ��ɻ�����,B����˸
			{
				LEDR_H;
				LEDG_H;
				LEDB_H;
				if(loop)
				{
					loop=0;

					LEDB_H;
				}
				else
				{
					loop=1;

					LEDB_L;
				}

			}
			
			
		}
		
}


