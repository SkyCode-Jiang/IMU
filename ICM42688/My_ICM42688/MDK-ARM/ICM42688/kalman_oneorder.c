#include "kalman_oneorder.h"

void kalman_1(struct _1_ekf_filter *ekf,float input)  //一阶卡尔曼
{
	// 预测数据的协方差值
	ekf->Now_P = ekf->LastP + ekf->Q;
	// 更新
	// 计算新的加权系数  越大越信任测量值
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	// 根据加权系数 计算数据输出
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	// 更新模型的协方差
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}


