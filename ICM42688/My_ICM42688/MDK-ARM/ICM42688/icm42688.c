#include "main.h"
#include "icm42688.h"
#include <stdio.h>


ICM42688Set User_set;

extern I2C_HandleTypeDef hi2c1;
void writeReg(uint8_t reg, void* pBuf, size_t size)
 {
	 HAL_I2C_Mem_Write(&hi2c1,ICM42688_I2C_WRITE_ADDR ,reg, I2C_MEMADD_SIZE_8BIT,pBuf,  size, 0xFFFF);
 }


uint8_t readReg(uint8_t reg, void* pBuf, size_t size)
{
	HAL_StatusTypeDef stuats;
	stuats = HAL_I2C_Mem_Read(&hi2c1, ICM42688_I2C_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pBuf, size, 0xFFFF);
	return stuats;
}


//读取Who_am_i寄存器
int ICM42688_Readme(void)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
	uint8_t id=0;
	readReg(ICM42688_WHO_AM_I,&id,1);

 
  if(readReg(ICM42688_WHO_AM_I,&id,1) != 0){
    return HAL_ERROR;
  }

  if(id != DFRobot_ICM42688_ID){
    return ERR_IC_VERSION;
  }
  uint8_t reset = 0;
  writeReg(ICM42688_DEVICE_CONFIG,&reset,1);
  HAL_Delay(2);
  return HAL_OK;
}





//设置加速度与陀螺仪的量程与采样速度
bool setODRAndFSR(uint8_t who,uint8_t ODR,uint8_t FSR)
{
  bool ret = true;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(who == GYRO){
    if(ODR > ODR_12_5KHZ || FSR > FSR_7){
      ret = false;
    }else{
      User_set.gyroConfig0.gyroODR = ODR;
      User_set.gyroConfig0.gyroFsSel = FSR;
      writeReg(ICM42688_GYRO_CONFIG0,&User_set.gyroConfig0,1);
      switch(FSR){
        case FSR_0:
          User_set._gyroRange = 4000/65535.0;
          break;
        case FSR_1:
          User_set._gyroRange = 2000/65535.0;
          break;
        case FSR_2:
          User_set._gyroRange = 1000/65535.0;
          break;
        case FSR_3:
          User_set._gyroRange = 500/65535.0;
          break;
        case FSR_4:
          User_set._gyroRange = 250/65535.0;
          break;
        case FSR_5:
          User_set._gyroRange = 125/65535.0;
          break;
        case FSR_6:
          User_set._gyroRange = 62.5/65535.0;
          break;
        case FSR_7:
          User_set._gyroRange = 31.25/65535.0;
          break;
      }
    }
  } else if(who == ACCEL){
    if(ODR > ODR_500HZ || FSR > FSR_3){
      ret = false;
    } else{
      User_set.accelConfig0.accelODR = ODR;
      User_set.accelConfig0.accelFsSel = FSR;
      writeReg(ICM42688_ACCEL_CONFIG0,&User_set.accelConfig0,1);
      switch(FSR){
        case FSR_0:
          User_set._accelRange = 0.488f;
          break;
        case FSR_1:
          User_set._accelRange = 0.244f;
          break;
        case FSR_2:
          User_set._accelRange = 0.122f;
          break;
        case FSR_3:
          User_set._accelRange = 0.061f;
          break;
      }
    }
  } 
  return ret;
}


//启动温度采集
void startTempMeasure()
{
  User_set.PWRMgmt0.tempDis = 0;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&User_set.PWRMgmt0,1);
  HAL_Delay(1);
}

//启动陀螺仪采集
void startGyroMeasure(uint8_t mode)
{
  User_set.PWRMgmt0.gyroMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&User_set.PWRMgmt0,1);
  HAL_Delay(1);
}
//启动加速度计采集
void startAccelMeasure(uint8_t mode)
{
  User_set.PWRMgmt0.accelMode = mode;
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  writeReg(ICM42688_PWR_MGMT0,&User_set.PWRMgmt0,1);
  HAL_Delay(10);
}


//获取温度 并转换
float getTemperature(void)
{
  float value;
  if(User_set.FIFOMode){
    value = (User_set._temp/2.07) + 25;
  } else{
    uint8_t data[2];
    int16_t value2;
    readReg(ICM42688_TEMP_DATA1, data, 2);
    value2 = ((uint16_t )data[0]<<8) | (uint16_t )data[1];
    value = value2/132.48 + 25;
  }
  return value;
}
//获取HEX值
void getHEXAccelData(uint8_t where, uint8_t *value, size_t size )
{
	uint8_t mid;
	switch(where)		{
			case X_:
				readReg(ICM42688_ACCEL_DATA_X1, value, 2);
				break;
			case Y_:
				readReg(ICM42688_ACCEL_DATA_Y1, value, 2);
				break;			
			case Z_:
				readReg(ICM42688_ACCEL_DATA_Z1, value, 2);
				break;		
	}
		
	mid = value[0];
	value[1] = 0;
	value[0] =mid;
}


void getHEXGyroData(uint8_t where, uint8_t *value, size_t size )
{
		uint8_t mid;
	 switch(where)		{
			case X_:
				readReg(ICM42688_GYRO_DATA_X1, value, 2);
				break;
			case Y_:
				readReg(ICM42688_GYRO_DATA_Y1, value, 2);
				break;			
			case Z_:
				readReg(ICM42688_GYRO_DATA_Z1, value, 2);
				break;		
	}
		
	mid = value[0];
	value[1] = 0;
	value[0] =mid;
}


//获取原始值 int16 *User_set._accelRange  float value;
int16_t getInt16AccelDataX(void)
{
  int16_t value;
  if(User_set.FIFOMode){
    value = User_set._accelX;
  } else{
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value;
}

int16_t getInt16AccelDataY(void)
{
  int16_t value;
  if(User_set.FIFOMode){
    value = User_set._accelY;
  } else{
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value;
}

int16_t getInt16AccelDataZ(void)
{
  int16_t value;
  if(User_set.FIFOMode){
    value = User_set._accelZ;
  } else{
    uint8_t data[2];
    readReg(ICM42688_ACCEL_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value;
}

int16_t getInt16GyroDataX(void)
{
  int16_t value;
  if(User_set.FIFOMode){
    value = User_set._gyroX;
  } else{
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_X1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value;
}

int16_t getInt16GyroDataY(void)
{
  int16_t value;
  if(User_set.FIFOMode){
    value = User_set._gyroY;
  } else{
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_Y1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value;
}

int16_t getInt16GyroDataZ(void)
{
  int16_t value;
  if(User_set.FIFOMode){
    value = User_set._gyroZ;
  } else{
    uint8_t data[2];
    readReg(ICM42688_GYRO_DATA_Z1, data, 2);
    int16_t value1 = ((uint16_t )data[0] << 8) | (uint16_t)data[1] ;
    value = value1;
  }
  return value;
}


//AD转换
float ADchange(int value,float range)
{
	return value*range;
}
float ADchangeGyro(int value)
{
	return value*User_set._gyroRange;
}
float getAccelDataX(void)
{
  float value;  
  int16_t value1 = getInt16AccelDataX() ;
  value = value1;
  return value*User_set._accelRange;
}

float getAccelDataY(void)
{
  float value;  
  int16_t value1 = getInt16AccelDataY() ;
  value = value1;
  return value*User_set._accelRange;
}


float getAccelDataZ(void)
{
  float value;  
  int16_t value1 = getInt16AccelDataZ() ;
  value = value1;
  return value*User_set._accelRange;
}

float getGyroDataX(void)
{
  float value;  
  int16_t value1 = getInt16GyroDataX() ;
  value = value1;
  return value*User_set._gyroRange;
}

float getGyroDataY(void)
{
  float value;  
  int16_t value1 = getInt16GyroDataY() ;
  value = value1;
  return value*User_set._gyroRange;
}

float getGyroDataZ(void)
{
  float value;  
  int16_t value1 = getInt16GyroDataZ() ;
  value = value1;
  return value*User_set._gyroRange;
}

void samepleGetTest(FLOAT_XYZ ACCFloat,FLOAT_XYZ GRYOFloat)
{	
	//getInt16AccelDataX、getAccelDataX重复读取	
	float temp = getTemperature();
  ACCFloat.X = getAccelDataX();
  ACCFloat.Y = getAccelDataY();
  ACCFloat.Z = getAccelDataZ();
  GRYOFloat.X= getGyroDataX();
  GRYOFloat.Y= getGyroDataY();
  GRYOFloat.Z= getGyroDataZ();		
  printf("Temperature: %f C",temp);
  printf("\r\n");

  printf("Accel_X: %f mg ",ACCFloat.X);
	printf("    Accel_Y: %f mg",ACCFloat.Y);
  printf("    Accel_Z: %f mg",ACCFloat.Z);
  printf("\r\n");
		
  printf("Gyro_X:  %f dps ",GRYOFloat.X);
	printf("    Gyro_y:  %f dps ",GRYOFloat.Y);
	printf("    Gyro_Z:  %f dps",GRYOFloat.Z);
  printf("\r\n");
}

	

void setFIFODataMode()
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  User_set.FIFOConfig1.FIFOHiresEn = 0;
  User_set.FIFOConfig1.FIFOAccelEn = 1;
  User_set.FIFOConfig1.FIFOGyroEn = 1;
  User_set.FIFOConfig1.FIFOTempEn = 1;
  User_set.FIFOConfig1.FIFOTmstFsyncEn = 0;
  writeReg(ICM42688_FIFO_CONFIG1,&(User_set.FIFOConfig1),1);

}
//Stream FIFI模式
void startFIFOMode()
{
  uint8_t bank = 0;
  User_set.FIFOMode = true;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  setFIFODataMode();
  uint8_t start = 1<<6;
  writeReg(ICM42688_FIFO_CONFIG,&start,1);
  getFIFOData();
}
void getFIFOData()
{
  uint8_t data[16];
  readReg(ICM42688_FIFO_DATA,data,16);
  User_set._accelX = (uint16_t)data[1]<<8 | (uint16_t)data[2];
  //DBG("_accelX");DBG(_accelX);
  User_set._accelY = (uint16_t)data[3]<<8 | (uint16_t)data[4];
  //DBG("_accelY");DBG(_accelY);
  User_set._accelZ = (uint16_t)data[5]<<8 | (uint16_t)data[6];
  //DBG("_accelZ");DBG(_accelZ);
  User_set._gyroX = (uint16_t)data[7]<<8 | (uint16_t)data[8];
  //DBG("_gyroX");DBG(_gyroX);
  User_set._gyroY = (uint16_t)data[9]<<8 | (uint16_t)data[10];
  //DBG("_gyroY");DBG(_gyroY);
  User_set._gyroZ = (uint16_t)data[11]<<8 | (uint16_t)data[12];
  //DBG("_gyroZ");DBG(_gyroZ);
  User_set._temp = (uint8_t)data[13];
  //DBG("_temp");DBG(data[13]);
}
void sotpFIFOMode()
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t start = 1<<7;
  writeReg(ICM42688_FIFO_CONFIG,&start,1);
}


void setINTMode(uint8_t INTPin,uint8_t INTmode,uint8_t INTPolarity,uint8_t INTDriveCircuit)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(INTPin == 1){
    User_set._INTPin = 1;
    User_set.INTConfig.INT1Mode = INTmode;
    User_set.INTConfig.INT1DriveCirCuit = INTDriveCircuit;
    User_set.INTConfig.INT1Polarity = INTPolarity;
  } else if(INTPin == 2){
    User_set._INTPin = 2;
    User_set.INTConfig.INT2Mode = INTmode;
    User_set.INTConfig.INT2DriveCirCuit = INTDriveCircuit;
    User_set.INTConfig.INT2Polarity = INTPolarity;
  }
  writeReg(ICM42688_INT_CONFIG,&(User_set.INTConfig),1);
}





uint8_t readInterruptStatus(uint8_t reg)
{
  uint8_t bank = 0;
  uint8_t status = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  readReg(reg,&status,1);
  return status;
}
void tapDetectionInit(uint8_t accelMode)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(accelMode == 0){                                                   //ACC LP模式
    User_set.accelConfig0.accelODR = 15;																//500Hz
    writeReg(ICM42688_ACCEL_CONFIG0,&(User_set.accelConfig0),1);
    User_set.PWRMgmt0.accelMode = 2;																	  //LP模式
    writeReg(ICM42688_PWR_MGMT0,&(User_set.PWRMgmt0),1);
    HAL_Delay(1000);
    User_set.INTFConfig1.accelLpClkSel = 0;															//只使用内部RC时钟	              
    writeReg(ICM42688_INTF_CONFIG1,&(User_set.INTFConfig1),1);
    User_set.accelConfig1.accelUIFiltORD = 2;														//UI 滤波器 3阶
    writeReg(ICM42688_ACCEL_CONFIG1,&User_set.accelConfig1,1);
    User_set.gyroAccelConfig0.accelUIFiltBW = 0;												
    writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&User_set.gyroAccelConfig0,1); // 未使用
  } else if(accelMode == 1){                                          //ACC LN模式
   User_set.accelConfig0.accelODR = 6;																//1000Hz
    writeReg(ICM42688_ACCEL_CONFIG0,&User_set.accelConfig0,1);
    User_set.PWRMgmt0.accelMode = 3;																	//LN模式
    writeReg(ICM42688_PWR_MGMT0,&User_set.PWRMgmt0,1);
    HAL_Delay(10);
    User_set.accelConfig1.accelUIFiltORD = 2;	                        //UI 滤波器 3阶
    writeReg(ICM42688_ACCEL_CONFIG1,&User_set.accelConfig1,1);
    User_set.gyroAccelConfig0.accelUIFiltBW = 0;									   	//ODR/2带宽
    writeReg(ICM42688_GYRO_ACCEL_CONFIG0,&User_set.gyroAccelConfig0,1);
  } else{
    return;
  }
  HAL_Delay(10);
  bank = 4;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  User_set.APEXConfig8.tapTmin = 5;													//单次敲击时间窗判断 时间窗--越大抗噪越好  延时越大
  User_set.APEXConfig8.tapTavg = 3;													//敲击平均能量判断   时间窗--越大抗噪越好
  User_set.APEXConfig8.tapTmax = 2;													//两次敲击判断 间隔最多3个采样点
  writeReg(ICM42688_APEX_CONFIG8,&User_set.APEXConfig8,1);
  User_set.APEXConfig7.tapMinJerkThr = 40;									// 敲击阈值0-63  越大抗噪越好 灵敏度较低
  User_set.APEXConfig7.tapMaxPeakTol = 1;										// 检测阈值峰值数量  0-2
  writeReg(ICM42688_APEX_CONFIG7,&User_set.APEXConfig7,1);
  HAL_Delay(10);
  User_set.INTSource.tapDetIntEn = 1;													//路由至中断
  if(User_set._INTPin==1){
    writeReg(ICM42688_INT_SOURCE6,&User_set.INTSource,1);
  } else {
    writeReg(ICM42688_INT_SOURCE7,&User_set.INTSource,1);
  }
  HAL_Delay(50);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  User_set.APEXConfig0.tapEnable = 1;
  writeReg(ICM42688_APEX_CONFIG0,&User_set.APEXConfig0,1); //使能 敲击检测
}
void getTapInformation()
{
  uint8_t data;
  readReg(ICM42688_APEX_DATA4, &data, 1);
  User_set._tapNum = data & 0x18;
  User_set._tapAxis = data & 0x06;
  User_set._tapDir = data & 0x01;
}
uint8_t numberOfTap()
{
  return User_set._tapNum;
}
uint8_t  axisOfTap()
{
  return User_set._tapAxis;
}




void  wakeOnMotionInit()
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  User_set.accelConfig0.accelODR = 9;													// ACC 50Hz			
  writeReg(ICM42688_ACCEL_CONFIG0,&User_set.accelConfig0,1);
  User_set.PWRMgmt0.accelMode = 2;														//LP 模式
  writeReg(ICM42688_PWR_MGMT0,&User_set.PWRMgmt0,1);
  HAL_Delay(1);
  User_set.INTFConfig1.accelLpClkSel = 0;											//RC时钟
  writeReg(ICM42688_INTF_CONFIG1,&User_set.INTFConfig1,1);
  HAL_Delay(1);
}


void setWOMTh(uint8_t axis,uint8_t threshold)
{
  uint8_t bank = 4;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t womValue = threshold;
  if(axis == X_AXIS){
    writeReg(ICM42688_ACCEL_WOM_X_THR,&womValue,1);
  } else if(axis == Y_AXIS){
    writeReg(ICM42688_ACCEL_WOM_Y_THR,&womValue,1);
  } else if(axis == Z_AXIS){
    writeReg(ICM42688_ACCEL_WOM_Z_THR,&womValue,1);
  } else if(axis == ALL){																		//X Y Z 设置检测阈值  mg单位
    writeReg(ICM42688_ACCEL_WOM_X_THR,&womValue,1);
    writeReg(ICM42688_ACCEL_WOM_Y_THR,&womValue,1);
    writeReg(ICM42688_ACCEL_WOM_Z_THR,&womValue,1);
  }
  HAL_Delay(1);
  bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
}

void enableSMDInterrupt(uint8_t mode)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t INT = 1<<3 ;
  if(mode != 0){
    if(User_set._INTPin == 1){
      writeReg(ICM42688_INT_SOURCE1,&INT,1);
    } else {
      writeReg(ICM42688_INT_SOURCE4,&INT,1);
    }
  }
  HAL_Delay(50);
  User_set.SMDConfig.SMDMode = mode;
  User_set.SMDConfig.WOMMode = 1;
  User_set.SMDConfig.WOMIntMode = 0;
  writeReg(ICM42688_SMD_CONFIG,&User_set.SMDConfig,1);	
}

void  setWOMInterrupt(uint8_t axis)
{
  uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  if(User_set._INTPin == 1){
    writeReg(ICM42688_INT_SOURCE1,&axis,1);
  } else {
    writeReg(ICM42688_INT_SOURCE4,&axis,1);
  }
  HAL_Delay(50);
	
//  User_set.SMDConfig.SMDMode = 1;
  User_set.SMDConfig.WOMMode = 1;
  User_set.SMDConfig.WOMIntMode = 0;
  writeReg(ICM42688_SMD_CONFIG,&User_set.SMDConfig,1);
}

void tiltDetectionInit()
{
	uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  User_set.accelConfig0.accelODR = 9;													// ACC 50Hz			
  writeReg(ICM42688_ACCEL_CONFIG0,&User_set.accelConfig0,1);
  User_set.PWRMgmt0.accelMode = 2;														//LP 模式
  writeReg(ICM42688_PWR_MGMT0,&User_set.PWRMgmt0,1);
  HAL_Delay(1);
  User_set.INTFConfig1.accelLpClkSel = 0;											//RC时钟
  writeReg(ICM42688_INTF_CONFIG1,&User_set.INTFConfig1,1);
	User_set.APEXConfig0.dmpODR = 2;
	writeReg(ICM42688_APEX_CONFIG0,&User_set.INTFConfig1,1);		//DMP ODR 50Hz
  HAL_Delay(1);
}

void setTiltDetection()
{
	uint8_t bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);	
  User_set.SignalPathReset_t.DMPMemResetEn = 1;											
  writeReg(ICM42688_SIGNAL_PATH_RESET,&User_set.SignalPathReset_t,1);
	HAL_Delay(1);
	
	bank = 4;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);	
  User_set.APEXConfig4.tiltWaitTimeSel=1;												//持续运动时间			
  writeReg(ICM42688_APEX_CONFIG4,&User_set.APEXConfig4,1);							
	HAL_Delay(1);
	
	bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);	
  User_set.SignalPathReset_t.DMPInitEn = 1;											
  writeReg(ICM42688_SIGNAL_PATH_RESET,&User_set.SignalPathReset_t,1);
	HAL_Delay(1);
}

void enableTILEInterrupt()
{
	uint8_t bank = 4;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  uint8_t INT = 1<<3 ;
	if(User_set._INTPin == 1){
		writeReg(ICM42688_INT_SOURCE6,&INT,1);
	} else {
		writeReg(ICM42688_INT_SOURCE7,&INT,1);
	}
  HAL_Delay(50);
	bank = 0;
  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
  User_set.APEXConfig0.tiltEnable = 1;
  writeReg(ICM42688_APEX_CONFIG0,&User_set.APEXConfig0,1);	
}
