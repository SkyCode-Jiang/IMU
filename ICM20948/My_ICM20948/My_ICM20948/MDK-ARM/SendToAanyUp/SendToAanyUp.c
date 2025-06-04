#include "SendToAanyUp.h"
#include "stdio.h"
#include "usart.h"

extern  UART_HandleTypeDef huart1;
extern void SendToUp();


void SendToUpcomSensor(int16_t *value,size_t size)
{
	uint8_t sum = 0;
	uint8_t* nop =NULL;
	SendDataSensor mydata ={
		.FrameHead  = {0xAA ,0XAA},
		.FrameMagic = 0x02,
		.FrameLen   = 18,
	};
	mydata.ACC_X[0] = BYTE1(value[0]);
	mydata.ACC_X[1] = BYTE0(value[0]);
	mydata.ACC_Y[0] = BYTE1(value[1]);
	mydata.ACC_Y[1] = BYTE0(value[1]);	
	mydata.ACC_Z[0] = BYTE1(value[2]);
	mydata.ACC_Z[1] = BYTE0(value[2]);	
	
	mydata.GRYO_X[0] = BYTE1(value[3]);
	mydata.GRYO_X[1] = BYTE0(value[3]);
	mydata.GRYO_Y[0] = BYTE1(value[4]);
	mydata.GRYO_Y[1] = BYTE0(value[4]);
	mydata.GRYO_Z[0] = BYTE1(value[5]);
	mydata.GRYO_Z[1] = BYTE0(value[5]);
	
	mydata.MAG_X[0] = 0x00;
	mydata.MAG_X[1] = 0x00;
	mydata.MAG_Y[0] = 0x00;
	mydata.MAG_Y[1] = 0x00;
	mydata.MAG_Z[0] = 0x00;
	mydata.MAG_Z[1] = 0x00;	
	nop = (uint8_t*)&mydata;
	for(int i=0 ; i <22 ;i++)
	{
		sum += *(nop+i);
	}
 mydata.FrameSUM = sum;

 SendToUp((uint8_t *)&mydata,sizeof(mydata));
	
}

void SendToUpcomEuler(int16_t *value,size_t size)
{
	uint8_t sum = 0;
	uint8_t* nop =NULL;
  SendDataEural mydata ={
	.FrameHead  = {0xAA ,0XAA},
	.FrameMagic = 0x01,
	.FrameLen   = 11,
	.ALTUSE = {0x00,},
	.ARMED = 0XA1
};

	int16_t _temp;
	_temp = (int)(value[0]*100);
   mydata.ROL[0] =BYTE1(_temp);
   mydata.ROL[1] =BYTE0(_temp);

	_temp = (int)(value[1]*100);
   mydata.PIT[0] =BYTE1(_temp);
   mydata.PIT[1] =BYTE0(_temp);

	_temp = (int)(value[2]*100);
   mydata.YAW[0] =BYTE1(_temp);
   mydata.YAW[1] =BYTE0(_temp);
	nop = (uint8_t*)&mydata;
	for(int i=0 ; i <15 ;i++)
	{
		sum += *(nop+i);
	}
 mydata.FrameSUM = sum;
SendToUp((uint8_t *)&mydata,sizeof(mydata));
}



