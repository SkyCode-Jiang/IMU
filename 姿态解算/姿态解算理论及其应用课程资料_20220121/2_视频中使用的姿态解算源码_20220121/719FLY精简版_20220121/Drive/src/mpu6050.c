/************************************************************************************************
* ����汾��V3.0
* �������ڣ�2022-11-3
* �������ߣ�719������ʵ����
************************************************************************************************/

#include "MPU6050.h"

static uint8_t    MPU6050_buff[14];                  	//���ٶ� ������ �¶� ԭʼ����
INT16_XYZ	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;		 				//��Ư����
INT16_XYZ	 MPU6050_ACC_RAW,MPU6050_GYRO_RAW;	     		//��ȡֵԭʼ����
uint8_t    	 SENSER_OFFSET_FLAG=1;                    //������У׼��־λ

/*****************************************************************************
* ��  ����uint8_t MPU6050_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
* ��  �ܣ�дһ���ֽ����ݵ� MPU6050 �Ĵ���
* ��  ����reg�� �Ĵ�����ַ
*         data: Ҫд�������
* ����ֵ��0�ɹ� 1ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU6050Addr,reg,data))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* ��  ����uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
* ��  �ܣ���ָ��MPU6050�Ĵ�����ȡһ���ֽ�����
* ��  ����reg�� �Ĵ�����ַ
*         buf:  ��ȡ���ݴ�ŵĵ�ַ
* ����ֵ��1ʧ�� 0�ɹ�
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU6050Addr,reg,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* ��  ����uint8_t MPU6050_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* ��  �ܣ���ָ���Ĵ���д��ָ����������
* ��  ����reg���Ĵ�����ַ
*         len��д�����ݳ��� 
*         buf: д�����ݴ�ŵĵ�ַ
* ����ֵ��0�ɹ� 1ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_WriteMultByteToSlave(MPU6050Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* ��  ����uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* ��  �ܣ���ָ���Ĵ�����ȡָ����������
* ��  ����reg���Ĵ�����ַ
*         len����ȡ���ݳ��� 
*         buf: ��ȡ���ݴ�ŵĵ�ַ
* ����ֵ��0�ɹ� 0ʧ��
* ��  ע��MPU6050������ֲֻ���I2C�����޸ĳ��Լ��ļ���
*****************************************************************************/
uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU6050Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

/******************************************************************************
* ��  ����uint8_tMPU6050_getDeviceID(void)
* ��  �ܣ���ȡ  MPU6050 WHO_AM_I ��ʶ������ 0x68
* ��  ������
* ����ֵ�����ض�ȡ����
* ��  ע����
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
	uint8_t buf;
	MPU6050_ReadByte(MPU6050_RA_WHO_AM_I, &buf);
	return buf;
}

/******************************************************************************
* ��  ����uint8_tMPU6050_testConnection(void)
* ��  �ܣ����MPU6050 �Ƿ��Ѿ�����
* ��  ������
* ����ֵ��1������ 0δ����
* ��  ע����
*******************************************************************************/
uint8_t MPU6050_testConnection(void) 
{
	if(MPU6050_getDeviceID() == 0x68)  
		return 1;
	else 
		return 0;
}

/******************************************************************************
* ��  ����void MPU6050_Check()
* ��  �ܣ����IIC�����ϵ�MPU6050�Ƿ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_Check(void) 
{ 
	while(!MPU6050_testConnection())
	{
		printf("\rMPU6050 no connect...\r\n");
		
	}
}

/******************************************************************************
* ��  ����void MPU6050_AccRead(FLOAT_XYZ *Acc_filt)
* ��  �ܣ���ȡ���ٶȵ�ԭʼ���ݣ�����һ�׿������˲�
* ��  ����*accData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_AccRead(FLOAT_XYZ *Acc_filt)
{
	  int16_t accData[3];
	  uint8_t i;
    uint8_t buf[6];
   	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)MPU6050_ACC_RAW.X;
    accData[1] = (int16_t)MPU6050_ACC_RAW.Y;
    accData[2] = (int16_t)MPU6050_ACC_RAW.Z;
	
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

/******************************************************************************
* ��  ����void MPU6050_GyroRead(FLOAT_XYZ *Gyr_filt)
* ��  �ܣ���ȡ�����ǵ�ԭʼ���ݣ�����һ�׵�ͨ�˲�
* ��  ����*gyroData ԭʼ���ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_GyroRead(FLOAT_XYZ *Gyr_filt)
{
	  int16_t gyroData[3];
	  uint8_t i;
    uint8_t buf[6];
	  MPU6050_ReadMultBytes(MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)MPU6050_GYRO_RAW.X ;
    gyroData[1] = (int16_t)MPU6050_GYRO_RAW.Y ;
    gyroData[2] = (int16_t)MPU6050_GYRO_RAW.Z ;
	
	  for(i=0;i<3;i++)//�Խ��ٶ�ԭʼ���ݽ���һ�׵�ͨ�˲�
		{  
       const float factor = 0.15f;  //�˲�����			
			 static float tBuff[3];
			 gyroData[i] = tBuff[i] = tBuff[i] * (1 - factor) + gyroData[i] * factor;
		}
		Gyr_filt->X  = (float)gyroData[0];
	  Gyr_filt->Y  = (float)gyroData[1];
	  Gyr_filt->Z  = (float)gyroData[2];
//		printf("%d,%f\r\n",MPU6050_GYRO_RAW.X,Gyr_filt->X);//��ӡ���ٶȼ�ԭʼ������Ϣ
}

/******************************************************************************
* ��  ����void MPU6050_TempRead(float *tempdata)
* ��  �ܣ��¶�ֵ��ȡ
* ��  ����*tempdata �¶����ݵ�ָ��
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU6050_ReadMultBytes(MPU6050_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f);
}

/******************************************************************************
* ��  ����void MPU6050_Init(void)
* ��  �ܣ���ʼ��MPU6050���빤��״̬
* ��  ������
* ����ֵ����
* ��  ע��DLPF �����Ϊ����Ƶ�ʵ�һ�룡����
*******************************************************************************/
void MPU6050_Init(void)
{
	MPU6050_Check(); 																						//���MPU6050�Ƿ�����
	
	MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x80); 						//��λMPU6050
	Delay_ms(100);
	MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x01); 						//����MPU6050����ѡ��������x��PLLΪʱ��Դ
	MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x00); 						//��ֹ�ж�
	MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18); 						//������������+-2000��/�� (��ͷֱ��� = 2^15/2000 = 16.4LSB/��/�� 
	MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x08); 					//���ٶ�������+-4g   (��ͷֱ��� = 2^15/4g = 8196LSB/g )
	MPU6050_WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_42);		//�������ݵ����Ϊ1kHZ,DLPF=42Hz 
	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 0x00);  						//������Ƶ (����Ƶ�� = ���������Ƶ�� / (1+DIV)������Ƶ��1000hz��
	MPU6050_WriteByte(MPU6050_RA_INT_PIN_CFG, 0x02); 						//MPU ��ֱ�ӷ���MPU6050����I2C
	 
}

/******************************************************************************
* ��  ����void MPU6050_CalOff(void)
* ��  �ܣ������Ǽ��ٶ�У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_CalOff(void)
{

	 SENSER_FLAG_SET(ACC_OFFSET);//���ٶ�У׼
	 SENSER_FLAG_SET(GYRO_OFFSET);//������У׼
}

/******************************************************************************
* ��  ����void MPU6050_CalOff_Acc(void)
* ��  �ܣ����ٶȼ�У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_CalOff_Acc(void)
{
	 SENSER_FLAG_SET(ACC_OFFSET);//���ٶ�У׼
}

/******************************************************************************
* ��  ����void MPU6050_CalOff_Gyr(void)
* ��  �ܣ�������У׼
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_CalOff_Gyr(void)
{
	 SENSER_FLAG_SET(GYRO_OFFSET);//������У׼
}

/******************************************************************************
* ��  ����void MPU6050_Read(void)
* ��  �ܣ���ȡ�����Ǽ��ٶȼƵ�ԭʼ����
* ��  ������
* ����ֵ����
* ��  ע����
*******************************************************************************/
void MPU6050_Read(void)
{
	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H, 14, MPU6050_buff);// ��ѯ����ȡMPU6050��ԭʼ����
}

/******************************************************************************
* ��  ����uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
* ��  �ܣ�MPU6050��ƫУ׼
* ��  ����value�� 	 MPU6050ԭʼ����
*         offset��	 У׼�����ƫֵ
*         sensivity�����ٶȼƵ�������
* ����ֵ��1У׼��� 0У׼δ���
* ��  ע����
*******************************************************************************/
uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
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
	tempgz += value.Z-sensivity;			//���ٶȼ�У׼ sensivity ���� MPU6050��ʼ��ʱ���õ�������ֵ��8196LSB/g��;������У׼ sensivity = 0��

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
* ��  ����void MPU6050_DataProcess(void)
* ��  �ܣ���MPU6050����ȥ��ƫ����
* ��  ������
* ����ֵ����
* ��  ע����

*******************************************************************************/
void MPU6050_Offset(void)
{
	//���ٶ�ȥ��ƫADֵ 
	MPU6050_ACC_RAW.X =((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]) - ACC_OFFSET_RAW.X;
	MPU6050_ACC_RAW.Y =((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]) - ACC_OFFSET_RAW.Y;
	MPU6050_ACC_RAW.Z =((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]) - ACC_OFFSET_RAW.Z;
			
	//������ȥ��ƫADֵ 
	MPU6050_GYRO_RAW.X =((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9]) - GYRO_OFFSET_RAW.X;
	MPU6050_GYRO_RAW.Y =((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]) - GYRO_OFFSET_RAW.Y;
	MPU6050_GYRO_RAW.Z =((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]) - GYRO_OFFSET_RAW.Z;
	
	if(GET_FLAG(GYRO_OFFSET)) //�����ǽ�����ƫУ׼
	{
		if(MPU6050_OffSet(MPU6050_GYRO_RAW,&GYRO_OFFSET_RAW,0))
		{
			
			 SENSER_FLAG_RESET(GYRO_OFFSET);
		
			 
		   SENSER_FLAG_SET(ACC_OFFSET);//У׼���ٶ�
		}
	}
	if(GET_FLAG(ACC_OFFSET)) //���ٶȼƽ�����ƫУ׼ 
	{
		if(MPU6050_OffSet(MPU6050_ACC_RAW,&ACC_OFFSET_RAW,8196))
		{
			 SENSER_FLAG_RESET(ACC_OFFSET);
			
		}
	}
}


