#include "PoseCalc.h"
#include "icm42688.h"
#include <math.h>
#include <stdio.h>
#include "SendToAanyUp.h"
#include "kalman_oneorder.h"

	
INT16_XYZ	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;		 							 //��Ư����
uint8_t    	 SENSER_OFFSET_FLAG=1;  												 //������У׼��־λ

extern FLOAT_XYZ 	Acc_filt,Gyr_filt,Acc_filtold,Gyr_radold;	  //�˲��������
float   DCMgb[3][3];          
extern FLOAT_ANGLE Att_Angle;                        				 //ŷ����//�������Ҿ���

extern float ADChangeACC(float Filtdata) ;
extern float ADChangeGYRO(float Filtdata) ;
/**
  *@author       JCLStill
  *@brief        ���ٶȿ������˲�
  *@param         void
  *@return        void
  */

void IMU_AccRead(INT16_XYZ ACCint16,FLOAT_XYZ *Acc_filt)
{
	  int16_t accData[3];
	  uint8_t i;

    accData[0] = ACCint16.X;
    accData[1] = ACCint16.Y;
    accData[2] = ACCint16.Z;
	
	  for(i=0;i<3;i++)//�Լ��ٶ�ԭʼ���ݽ���һ�׿������˲�
		{ 
			static struct _1_ekf_filter ekf[3] = {{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543},{0.02,0,0,0,0.001,0.543}};	
			kalman_1(&ekf[i],accData[i]);
			accData[i] = (int16_t)ekf[i].out;
		}
	  Acc_filt->X  = (float)accData[0];
	  Acc_filt->Y  = (float)accData[1];
	  Acc_filt->Z  = (float)accData[2];
//		printf("%d,%f\r\n",MPU6050_ACC_RAW.X,Acc_filt->X);//�ɹ۲쵽�������˲�ǰ����ٶȼ���Ϣ�Ĳ���

}
/**
  *@author       JCLStill
  *@brief         ���ٶ�һ�׵�ͨ�˲�
  *@param         void
  *@return        void
  */

void IMU_GyroRead(INT16_XYZ GYROint16,FLOAT_XYZ *Gyr_filt)
{
	  int16_t gyroData[3];
	  uint8_t i;

    gyroData[0] = GYROint16.X ;
    gyroData[1] = GYROint16.Y ;
    gyroData[2] = GYROint16.Z ;
	
	  for(i=0;i<3;i++)//�Խ��ٶ�ԭʼ���ݽ���һ�׵�ͨ�˲�
		{  
       const float factor = 0.15f;  //�˲�����			
			 static float tBuff[3];
			 gyroData[i] = tBuff[i] = tBuff[i] * (1 - factor) + gyroData[i] * factor;
		}
		Gyr_filt->X  = (float)gyroData[0];
	  Gyr_filt->Y  = (float)gyroData[1];
	  Gyr_filt->Z  = (float)gyroData[2];

}


/******************************************************************************
* ��  �ܣ������Ǽ��ٶ�У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void IMU_CalOff(void)
{

	 SENSER_FLAG_SET(ACC_OFFSET);//���ٶ�У׼
	 SENSER_FLAG_SET(GYRO_OFFSET);//������У׼
}

/******************************************************************************
* ��  �ܣ����ٶȼ�У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void IMU_CalOff_Acc(void)
{
	 SENSER_FLAG_SET(ACC_OFFSET);//���ٶ�У׼
}

/******************************************************************************
* ��  �ܣ�������У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void IMU0_CalOff_Gyr(void)
{
	 SENSER_FLAG_SET(GYRO_OFFSET);//������У׼
}


/******************************************************************************
* ��  ����uint8_t IMU_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
* ��  �ܣ�IMU��ƫУ׼
* ��  ����value�� 	 ԭʼ����
*         offset��	 У׼�����ƫֵ
*         sensivity�����ٶȼƵ�������
* ����ֵ��1У׼��� 0У׼δ���
* ��  ע����
*******************************************************************************/
uint8_t IMU_OffSet_private(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
{
	static int32_t tempgx=0,tempgy=0,tempgz=0;
	static uint16_t cnt_a=0;//ʹ��static���εľֲ������������α������о�̬�洢���ڣ�Ҳ����˵�ú���ִ������ͷ��ڴ�
	if(cnt_a==0)
	{
		value.X=0;
		value.Y=0;
		value.Z=0;
		tempgx = 0;
		tempgy = 0;
		tempgz = 0;
		cnt_a = 1;
		sensivity = 0;
		offset->X = 0;
		offset->Y = 0;
		offset->Z = 0;
	}
	tempgx += value.X;
	tempgy += value.Y; 
	tempgz += value.Z-sensivity;			//���ٶȼ�У׼ sensivity ��ʼ��ʱ���õ�������ֵ��8196LSB/g��;������У׼ sensivity = 0��

	if(cnt_a==200)               //200����ֵ��ƽ��
	{
		offset->X=tempgx/cnt_a;
		offset->Y=tempgy/cnt_a;
		offset->Z=tempgz/cnt_a;
		cnt_a = 0;

		return 1;
	}
	cnt_a++;
	return 0;
}	

/******************************************************************************
* ��  �ܣ���IMU����ȥ��ƫ����
* ��  ������
* ����ֵ����
* ��  ע����

*******************************************************************************/
void IMU_Offset(INT16_XYZ ACCint16,INT16_XYZ GYROint16)
{
	
	if(GET_FLAG(GYRO_OFFSET)) //�����ǽ�����ƫУ׼
	{
		if(IMU_OffSet_private(GYROint16,&GYRO_OFFSET_RAW,0))
		{
			
			 SENSER_FLAG_RESET(GYRO_OFFSET);
		
			 
		   SENSER_FLAG_SET(ACC_OFFSET);//У׼���ٶ�
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //���ٶȼƽ�����ƫУ׼ 
	{
		if(IMU_OffSet_private(ACCint16,&ACC_OFFSET_RAW,8196))
		{
			 SENSER_FLAG_RESET(ACC_OFFSET);
			
		}
	}
}

void Prepare_Data(INT16_XYZ ACCint16,INT16_XYZ GYROint16)
{
 
	IMU_Offset(ACCint16,GYROint16);  //У׼ ��ȥ��ƫ

	IMU_AccRead(ACCint16,&Acc_filt);//�Լ���һ�׿������˲�
	IMU_GyroRead(GYROint16,&Gyr_filt);//���ٶȵ�ͨ�˲�
	
//	Aver_FilterXYZ(&Acc_filt,12);//���������˲� ����Ч�����Ǻܺ�
//  Aver_FilterXYZ(&Gyr_filt,12);//���������˲�
	
	//ADת��	
	Acc_filt.X = ADChangeACC(Acc_filt.X) * G;
	Acc_filt.Y = ADChangeACC(Acc_filt.Y) * G;
	Acc_filt.Z = ADChangeACC(Acc_filt.Z) * G;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",Acc_filt.X,Acc_filt.Y,Acc_filt.Z);

	Gyr_filt.X = ADChangeGYRO(Gyr_filt.X);  
	Gyr_filt.Y = ADChangeGYRO(Gyr_filt.Y); 
	Gyr_filt.Z = ADChangeGYRO(Gyr_filt.Z); 
//	printf("gx=%0.2f gy=%0.2f gz=%0.2f\r\n",Gyr_filt.X,Gyr_filt.Y,Gyr_filt.Z);
}

/****************************************************************************************************
* ��  ����float invSqrt(float x) 
* ������: ���ټ��� 1/Sqrt(x) 	
* ��  ����Ҫ�����ֵ
* ����ֵ������Ľ��
* ��  ע������ͨSqrt()����Ҫ���ı�See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
*****************************************************************************************************/
float invSqrt(float x) 
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f375a86 - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
//��̬����
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;     // quaternion elements representing the estimated orientation
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral erro

void IMUupdate(FLOAT_XYZ *Gyr_filt,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle)
{
	float ax = Acc_filt->X,ay = Acc_filt->Y,az = Acc_filt->Z;
	float gx = Gyr_filt->X,gy = Gyr_filt->Y,gz = Gyr_filt->Z;
	float vx, vy, vz;
	float ex, ey, ez;
	float norm;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;

	if(ax*ay*az==0)
	return;

	//���ٶȼ� ���� �������ٶ�����--��������ϵ 
	norm = invSqrt(ax*ax + ay*ay + az*az); 
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
//	printf("ax=%0.2f ay=%0.2f az=%0.2f\r\n",ax,ay,az);

	//�����ǻ��� ���� ��������--��������ϵ 
	vx = 2*(q1q3 - q0q2);	               //(3,1)������									
	vy = 2*(q0q1 + q2q3);                //(3,2)������
	vz = q0q0 - q1q1 - q2q2 + q3q3 ;     //(3,3)������
	
	// printf("vx=%0.2f vy=%0.2f vz=%0.2f\r\n",vx,vy,vz); 

	//�������
	ex = (ay*vz - az*vy);                     
	ey = (az*vx - ax*vz); 
	ez = (ax*vy - ay*vx);

	//��������л���
	exInt = exInt + ex * Ki;								 
	eyInt = eyInt + ey * Ki;
	ezInt = ezInt + ez * Ki;

	//�����PI�󲹳���������
	gx = gx + Kp*ex + exInt;					   		  	
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;//???gz????????????????,??????????????

	//��Ԫ��΢�ַ���
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

	//��һ����Ԫ��
	norm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;  
	q3 = q3 * norm;

//  ?????
//	matrix[0] = q0q0 + q1q1 - q2q2 - q3q3;	 // 11
//	matrix[1] = 2.f * (q1q2 + q0q3);	       // 12
//	matrix[2] = 2.f * (q1q3 - q0q2);	       // 13
//	matrix[3] = 2.f * (q1q2 - q0q3);	       // 21
//	matrix[4] = q0q0 - q1q1 + q2q2 - q3q3;	 // 22
//	matrix[5] = 2.f * (q2q3 + q0q1);	       // 23
//	matrix[6] = 2.f * (q1q3 + q0q2);	       // 31
//	matrix[7] = 2.f * (q2q3 - q0q1);	       // 32
//	matrix[8] = q0q0 - q1q1 - q2q2 + q3q3;	 // 33

	//��Ԫ�����ŷ����(Z->Y->X) 
	
	//ƫ����YAW
	if((Gyr_filt->Z *RadtoDeg > 1.0f) || (Gyr_filt->Z *RadtoDeg < -1.0f)) //����̫С��Ϊ�Ǹ���
	{
		Att_Angle->yaw += Gyr_filt->Z *RadtoDeg*0.01f;
	}   
//	printf("yaw: %f\r\n",Att_Angle->yaw);//
	
	//������ROLL
	Att_Angle->rol = -asin(2.f * (q1q3 - q0q2))* 57.3f;
//  printf("%f\r\n",Att_Angle->rol);//ROLL???

	//������PITCH
	Att_Angle->pit = -atan2(2.f * q2q3 + 2.f * q0q1, q0q0 - q1q1 - q2q2 + q3q3)* 57.3f ;
//	printf("pitch: %f\r\n",Att_Angle->pit);//PITCH???
//	 printf("\r\n");
//    printf("%f,%f,%f\r\n",Att_Angle->rol,Att_Angle->pit,Att_Angle->yaw);//PITCH???
//	int16_t value[3];
//	value[0]= Att_Angle->rol;
//	value[1]= Att_Angle->pit;
//	value[2]= Att_Angle->yaw;
//	SendToUpcomEuler(value,3);
}



void Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n)
{
	static int32_t bufax[N],bufay[N],bufaz[N];
	static uint8_t cnt =0,flag = 1;
	int32_t temp1=0,temp2=0,temp3=0,i;
	bufax[cnt] = Acc_filt->X;
	bufay[cnt] = Acc_filt->Y;
	bufaz[cnt] = Acc_filt->Z;
	cnt++;     														
	if(cnt<n && flag) 
		return; 														   
	else
		flag = 0;
	for(i=0;i<n;i++)
	{
		temp1 += bufax[i];
		temp2 += bufay[i];
		temp3 += bufaz[i];
	}
	 if(cnt>=n)  cnt = 0;
	 Acc_filt->X  = temp1/n;
	 Acc_filt->Y  = temp2/n;
	 Acc_filt->Z  = temp3/n;
}

