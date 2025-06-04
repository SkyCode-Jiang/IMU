#ifndef SENDTOANNYUP_H
#define SENDTOANNYUP_H
#include "main.h"
//匿名上位机V1.3 通信协议

#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

typedef struct {
	uint8_t	FrameHead[2];
	uint8_t FrameMagic;
	uint8_t FrameLen;
	uint8_t ACC_X[2];
	uint8_t ACC_Y[2];
	uint8_t ACC_Z[2];
	uint8_t GRYO_X[2];
	uint8_t GRYO_Y[2];
	uint8_t GRYO_Z[2];
	uint8_t MAG_X[2];
	uint8_t MAG_Y[2];
	uint8_t MAG_Z[2];
	uint8_t FrameSUM;
}SendDataSensor;

typedef struct {
	uint8_t	FrameHead[2];
	uint8_t FrameMagic;
	uint8_t FrameLen;
	uint8_t ROL[2];
	uint8_t PIT[2];
	uint8_t YAW[2];
	uint8_t ALTUSE[4]; //飞机高度
	uint8_t ARMED;
	uint8_t FrameSUM;
}SendDataEural;
/**
  *@author       JCLStill
  *@brief        发送传感器数据
  *@param         X Y Z 加速度与角速度
  *@return        void
  */
void SendToUpcomSensor(int16_t *value,size_t size);
/**
  *@author       JCLStill
  *@brief        发送姿态数据
  *@param         ROL PIT YAW角度数据
  *@return        void
  */
void SendToUpcomEuler(int16_t *value,size_t size);

#endif 