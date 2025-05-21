/************************************************************************************************
* 程序版本：V3.0
* 程序日期：2022-11-3
* 程序作者：719飞行器实验室
************************************************************************************************/
#ifndef   _PID_H_
#define   _PID_H_

#include "main.h"
void PidParameter_init(void);
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure);
void UnControl_Check(void);
#endif
