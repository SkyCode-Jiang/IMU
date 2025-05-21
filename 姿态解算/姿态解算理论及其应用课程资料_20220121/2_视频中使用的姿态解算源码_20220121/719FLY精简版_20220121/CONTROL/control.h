/************************************************************************************************
* 程序版本：V3.0
* 程序日期：2022-11-3
* 程序作者：719飞行器实验室
************************************************************************************************/
#ifndef   _CONTROL_H
#define   _CONTROL_H


#include "main.h"
void Control(FLOAT_ANGLE *att_in,FLOAT_XYZ *gyr_in, RC_TYPE *rc_in, u8 armed);
int16_t Yaw_Control(float TARGET_YAW);
void Yaw_Carefree(FLOAT_ANGLE *Target_Angle, const FLOAT_ANGLE *Measure_Angle);
void Safety_Check(void);
extern uint8_t RC_middle;
#endif

