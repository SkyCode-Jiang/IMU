#include "kalman_oneorder.h"

void kalman_1(struct _1_ekf_filter *ekf,float input)  //һ�׿�����
{
	// Ԥ�����ݵ�Э����ֵ
	ekf->Now_P = ekf->LastP + ekf->Q;
	// ����
	// �����µļ�Ȩϵ��  Խ��Խ���β���ֵ
	ekf->Kg = ekf->Now_P / (ekf->Now_P + ekf->R);
	// ���ݼ�Ȩϵ�� �����������
	ekf->out = ekf->out + ekf->Kg * (input - ekf->out);
	// ����ģ�͵�Э����
	ekf->LastP = (1-ekf->Kg) * ekf->Now_P ;
}


