#ifndef POSECALC_H
#define POSECALC_H
#include "main.h"
#include "icm42688.h"
#define N 20                      //滑动窗口滤波数量
#define G			9.80665f		      	// m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)

#define SENSER_FLAG_SET(FLAG)   SENSER_OFFSET_FLAG|=FLAG                //设置状态值 1
#define SENSER_FLAG_RESET(FLAG) SENSER_OFFSET_FLAG&=~FLAG               //设置状态值 0
#define GET_FLAG(FLAG)         (SENSER_OFFSET_FLAG&FLAG)==FLAG ? 1 : 0  //获取状态值
#define GYRO_OFFSET 0x01 //陀螺仪 零漂校准
#define ACC_OFFSET 0x02  //加速度 零漂校准

#define Kp_New      0.9f           				    //互补滤波当前数据的权重
#define Kp_Old      0.1f           				    //互补滤波道钒数据的权重  
    

//姿态解算后的欧拉加
typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE;

//kp=ki=0 代表完全相信陀螺仪
#define Kp 1.5f                          // proportional gain governs rate of convergence to accelerometer/magnetometer
                                          //比例增益 控制加速度磁力计的收敛速度
#define Ki 0.005f                         // integral gain governs rate of convergence of gyroscope biases  
                                          //积分增益 控制陀螺仪偏差收敛速度
#define halfT 0.005f                      // half the sample period



void IMU_Offset(INT16_XYZ ACCint16,INT16_XYZ GYROint16);
void IMU_CalOff(void);
void IMU_CalOff_Acc(void);
void IMU_CalOff_Gyr(void);
void IMU_GyroRead(INT16_XYZ GYROint16,FLOAT_XYZ *Gyr_filt);
void IMU_AccRead(INT16_XYZ ACCint16,FLOAT_XYZ *Acc_filt);
void Aver_FilterXYZ(FLOAT_XYZ *Acc_filt,uint8_t n);

//调用函数
void Prepare_Data(INT16_XYZ accint16,INT16_XYZ gyroint16);
void IMUupdate(FLOAT_XYZ *Gyr_rad,FLOAT_XYZ *Acc_filt,FLOAT_ANGLE *Att_Angle);

#endif