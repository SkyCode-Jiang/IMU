/************************************************************************************************
* 程序版本：V3.0
* 程序日期：2022-11-3
* 程序作者：719飞行器实验室
************************************************************************************************/
#ifndef _KALMAN_H
#define _KALMAN_H

struct _1_ekf_filter
{
	float LastP;
	float	Now_P;
	float out;
	float Kg;
	float Q;
	float R;	
};

//void ekf_1(struct EKF *ekf,void *input);  //一维卡尔曼
extern void kalman_1(struct _1_ekf_filter *ekf,float input);  //一维卡尔曼

#endif


