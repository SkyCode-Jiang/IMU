#ifndef FUNCTION_H
#define FUNCTION_H

////////////////////////匿名上位机
//#define USEUPCOM

////////////////////////FIFO读取
//#define USEFIFO

///////////////////////APEX//////////////////////

////////////////////////敲击检测   	  ACC 200 500 1000Hz
//#define TAPDETE
//#define TAPDETE_INTNUM 1

////////////////////////倾斜检测  持续运动超过30°  ACC 50Hz
#define TILEDETE
#define TILEDETE_INTNUM  1	

////////////////////////显著运动检测  ACC 50Hz
//#define SIGNMOVE
//#define SIGNMOVE_INTNUM 1


////////////////////////运动唤醒检测  ACC 50Hz
//#define WAKEMOVE
//#define WAKEMOVE_INTNUM 2









#endif