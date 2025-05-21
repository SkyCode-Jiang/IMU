#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//PB12 -- NCS
//PB13 -- SCK
//PB14 -- SDO
//PB15 -- SDI
#define	ICM20948_CS  PBout(12)   //片选信号

void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度
u8 SPI2_ReadWriteByte(u8 TxData,u8* RxData);//SPI总线读写一个字节

#endif

