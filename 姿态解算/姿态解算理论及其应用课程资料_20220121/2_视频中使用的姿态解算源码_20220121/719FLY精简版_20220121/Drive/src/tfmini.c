//#include "main.h"
////#include "tfmini.h"

///**ʹ�ô���1����ͨ��**/
//uint16_t TFminiStrength;
////uint8_t TFminiMode,TFminiCheckSum;
//uint8_t TFminiDATA[10],TFminiDistance;
////unsigned char TFminiDATA[Max_BUFF_Len];
////unsigned int RXTime=0;
//uint8_t RXTime=0;//����


///**ģ�⴮�ڣ������������ݼ��   ������   TX PA9**/

//void MUSART1_SendData(uint8_t data)
//{
//	uint8_t i = 0;
//	

//	//Delay_us(104);
//	
//	GPIOB->BRR  = GPIO_Pin_13;		//!<��ʼλ
//	Delay_us(8);
//	for(i = 0; i < 8; i++)
//	{
//		if(data & 0x01)
//			GPIOB->BSRR = GPIO_Pin_13;
//		else
//			GPIOB->BRR  = GPIO_Pin_13;
//		Delay_us(8);
//		data >>= 1;
//	}
//	GPIOB->BSRR = GPIO_Pin_13;		//!<ֹͣλ
//	Delay_us(8);
//}



//void USART1_IRQHandler(void)//����TFmini����
//{
//	 
//		if(USART_GetITStatus(USART1,USART_IT_RXNE) != RESET) //�жϲ��� 
//	{
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE); //����жϱ�־
//		TFminiDATA[RXTime] = USART_ReceiveData(USART1);
//		RXTime++;
//		if(TFminiDATA[0]!=0x59)	 
//			{
//			RXTime=0;
//			}
//			
//		if(TFminiDATA[1]==0x59&&RXTime>=8)
//			{
//		//MUSART1_SendData(TFminiDATA[2]);//dd���ǵ�ɾ����������
//		//		MUSART1_SendData(TFminiDATA[2]);//���ǵ�ɾ����������
//				TFminiDistance=TFminiDATA[2];
//			
//				//	MUSART1_SendData(RXTime);
//			RXTime=0;
//			}
//		
//		
//	}
//}

//			






//void TFmini_Init(void)
//{
//	//ȫ��ʹ��Ĭ������   ����������Ȳ����ٸĳ� �̶���λģʽ+�̾��뵵λ
//}
