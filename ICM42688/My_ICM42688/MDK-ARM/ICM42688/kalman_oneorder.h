#ifndef KALMAN_ONEORDER_H
#define KALMAN_ONEORDER_H
struct _1_ekf_filter
{
	float LastP;//��һ��Э����
	float	Now_P;//ģ��Ԥ�����ݵ�Э����ֵ
	float out;  //�������
	float Kg;   //��Ȩϵ��
	float Q;    //ģ������Э����
	float R;		//��������Э���� ����������
};/**
  *@author       JCLStill
  *@brief         Des
  *@param         input :����Ĵ������ɼ���������
  *@return        void
  */
void kalman_1(struct _1_ekf_filter *ekf,float input);

#endif