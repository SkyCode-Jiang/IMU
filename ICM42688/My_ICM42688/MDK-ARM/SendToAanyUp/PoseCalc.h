#ifndef POSECALC_H
#define POSECALC_H
#include "main.h"
#include "icm42688.h"
#define N 20                      //���������˲�����
#define G			9.80665f		      	// m/s^2	
#define RadtoDeg    57.324841f				//���ȵ��Ƕ� (���� * 180/3.1415)
#define DegtoRad    0.0174533f				//�Ƕȵ����� (�Ƕ� * 3.1415/180)

#define SENSER_FLAG_SET(FLAG)   SENSER_OFFSET_FLAG|=FLAG                //����״ֵ̬ 1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG&=~FLAG               //����״ֵ̬ 0
#define GET_FLAG(FLAG)         (SENSER_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //��ȡ״ֵ̬
#define GYRO_OFFSET 0x01 //������ ��ƯУ׼
#define ACC_OFFSET 0x02  //���ٶ� ��ƯУ׼

#define Kp_New      0.9f           				    //�����˲���ǰ���ݵ�Ȩ��
#define Kp_Old      0.1f           				    //�����˲����ʷ����ݵ�Ȩ��  
    

//��̬������ŷ����
typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE;

//kp=ki=0 ������ȫ����������
#define Kp 1.5f                          // proportional gain governs rate of convergence to accelerometer/magnetometer
                                          //�������� ���Ƽ��ٶȴ����Ƶ������ٶ�
#define Ki 0.005f                         // integral gain governs rate of convergence of gyroscope biases  
                                          //�������� ����������ƫ�������ٶ�
#define halfT 0.005f                      // half the sample period



void IMU_Offset(INT16_XYZ ACCint16,INT16_XYZ GYROint16);
void IMU_CalOff(void);
void IMU_CalOff_Acc(void);
void IMU_CalOff_Gyr(void);
void IMU_GyroRead(INT16_XYZ GYROint16,FLOAT_XYZ *Gyr_filt);
void IMU_AccRead(INT16_XYZ ACCint16,FLOAT_XYZ *Acc_filt);
void Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n);

//���ú���
void Prepare_Data(INT16_XYZ accint16,INT16_XYZ gyroint16);
void IMUupdate(FLOAT_XYZ *Gyr_rad,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle);

#endif