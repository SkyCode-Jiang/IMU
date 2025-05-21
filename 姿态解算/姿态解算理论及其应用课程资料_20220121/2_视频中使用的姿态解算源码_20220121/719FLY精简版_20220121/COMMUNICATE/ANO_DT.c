/************************************************************************************************
* 程序版本：V3.0
* 程序日期：2022-11-3
* 程序作者：719飞行器实验室
************************************************************************************************/
#include "ANO_DT.h"
#include "usart.h"
#include "imu.h"
#include "mpu6050.h"
#include "tim.h"
#include "structconfig.h"
#include "paramsave.h"
#include "filter.h"


//数据拆分宏定义，在发送大于1字节的数据类型时，比如int16、float等，需要把数据拆分成单独字节进行发送
#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )
	
u8 BUFF[30];

void ANO_DT_Send_Data(float A,float B,float C)
{
	int i;
	u8 sumcheck = 0;
	u8 addcheck = 0;
	u8 _cnt=0;
	BUFF[_cnt++]=0xAA;//帧头
	BUFF[_cnt++]=0xFF;//目标地址
	BUFF[_cnt++]=0XF1;//功能码
	BUFF[_cnt++]=0x12;//数据长度
	//发送俯仰角
	BUFF[_cnt++]=BYTE0(A);//数据内容,小段模式，低位在前
	BUFF[_cnt++]=BYTE1(A);//需要将字节进行拆分，调用上面的宏定义即可。
	BUFF[_cnt++]=BYTE2(A);
	BUFF[_cnt++]=BYTE3(A);
	//发送横滚角
	BUFF[_cnt++]=BYTE0(B);
	BUFF[_cnt++]=BYTE1(B);
	BUFF[_cnt++]=BYTE2(B);
	BUFF[_cnt++]=BYTE3(B);
	//发送偏航角
	BUFF[_cnt++]=BYTE0(C);
	BUFF[_cnt++]=BYTE1(C);
	BUFF[_cnt++]=BYTE2(C);
	BUFF[_cnt++]=BYTE3(C);
	
	//SC和AC的校验直接抄最上面上面简介的即可
	for(i=0;i<BUFF[3]+12;i++) 
	{
		sumcheck+=BUFF[i];
		addcheck+=sumcheck;
	}
	BUFF[_cnt++]=sumcheck;	
	BUFF[_cnt++]=addcheck;	
	
	for(i=0;i<_cnt;i++) 
	{
		USART_SendData(USART1,BUFF[i]);//串口逐个发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TXE == RESET));
	}
	
	
}
