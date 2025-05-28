#ifndef KALMAN_ONEORDER_H
#define KALMAN_ONEORDER_H
struct _1_ekf_filter
{
	float LastP;//上一轮协方差
	float	Now_P;//模型预测数据的协方差值
	float out;  //输出数据
	float Kg;   //加权系数
	float Q;    //模型噪音协方差
	float R;		//测量噪音协方差 传感器噪音
};/**
  *@author       JCLStill
  *@brief         Des
  *@param         input :输入的传感器采集到的数据
  *@return        void
  */
void kalman_1(struct _1_ekf_filter *ekf,float input);

#endif