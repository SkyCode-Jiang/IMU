/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/
#ifndef   _PID_H_
#define   _PID_H_

#include "main.h"
void PidParameter_init(void);
void PID_Postion_Cal(PID_TYPE*PID,float target,float measure);
void UnControl_Check(void);
#endif
