/************************************************************************************************
* 程序版本：V3.0
* 程序日期：2022-11-3
* 程序作者：719飞行器实验室
************************************************************************************************/
#ifndef   _REMOTEDATA_H
#define   _REMOTEDATA_H

#include "stm32f10x.h"

void Remote_Data_ReceiveAnalysis(void);
void SI24R1_SingalCheck(void);
void WiFi_Data_Receive(uint8_t data);
void WiFi_Data_ReceiveAnalysis(uint8_t*buff,uint8_t cnt);

void SendToRemote(void);
#endif
