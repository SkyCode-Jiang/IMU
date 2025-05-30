/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "function.h"
#include "icm42688.h"
#include "SendToAanyUp.h"
#include "PoseCalc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern ICM42688Set User_set;                           //icm42688参数
//原始数据
INT16_XYZ ACCInt,GYROInt;
//AD转换后的浮点数据
FLOAT_XYZ ACCFloat,GYROFloat;
float tempData;

FLOAT_ANGLE Att_Angle;                                 //欧拉角
FLOAT_XYZ 	Acc_filt,Acc_filtold,Gyr_radold,Gyr_filt;	  //滤波后的数据

uint8_t GETDATAFLAG =0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
///////////根据设备不同改写//////////////////
///////writeReg///////////MCU////////////////
///////readReg////////////MCU////////////////
///////SendToUp///////////MCU////////////////
///////ADChangeACC////////IMU////////////////
///////ADChangeGYRO///////IMU////////////////
/////////////////////////////////////////////

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi1;
uint8_t SPI_ReadWriteByte(uint8_t TxData)
{
	uint8_t Rxdata;
	HAL_SPI_TransmitReceive(&hspi1, &TxData, &Rxdata, 1, 1000);       
 	return Rxdata;          		    
}

void SPI_Read(uint8_t* pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead)   
{ 
	uint16_t i; 
	
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port,ICM_CS_Pin,GPIO_PIN_RESET);  
	SPI_ReadWriteByte((uint8_t)ReadAddr);   
	for (i = 0; i < NumByteToRead; i++)
	{ 
		pBuffer[i]=SPI_ReadWriteByte(0XFF);    
	}	
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port,ICM_CS_Pin,GPIO_PIN_SET); 
} 


void SPI_Write (uint8_t* pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite)
{
 	uint16_t i; 
 
    HAL_GPIO_WritePin(ICM_CS_GPIO_Port,ICM_CS_Pin,GPIO_PIN_RESET);   
   SPI_ReadWriteByte((uint8_t)WriteAddr);   
    for (i = 0; i < NumByteToWrite; i++)
	{
		SPI_ReadWriteByte(pBuffer[i]);
	}
	
	HAL_GPIO_WritePin(ICM_CS_GPIO_Port,ICM_CS_Pin,GPIO_PIN_SET); 
}



void writeReg(uint8_t reg, void* pBuf, size_t size)
 { 
	 #ifdef  IIC_USE
	 
		HAL_I2C_Mem_Write(&hi2c1,ICM42688_I2C_WRITE_ADDR ,reg, I2C_MEMADD_SIZE_8BIT,pBuf,  size, 0xFFFF);
	 
	 #endif
	 
	#ifdef  SPI_USE
	 
	 SPI_Write(pBuf,reg&0x7F,size);
	
	 #endif
	
 }


uint8_t readReg(uint8_t reg, void* pBuf, size_t size)
{
	
	#ifdef  IIC_USE
	
	HAL_StatusTypeDef stuats;
	stuats = HAL_I2C_Mem_Read(&hi2c1, ICM42688_I2C_READ_ADDR, reg, I2C_MEMADD_SIZE_8BIT, pBuf, size, 0xFFFF);
	return stuats;	
	
	#endif
	
	#ifdef  SPI_USE	
		 
	SPI_Read(pBuf,reg|0x80,size);
////////////////////////////////////////test	
//		HAL_GPIO_WritePin(ICM_CS_GPIO_Port,ICM_CS_Pin,GPIO_PIN_RESET); 
//		uint8_t reg2 = (reg | 0x80);
//		HAL_SPI_Transmit( &hspi1, &reg2, 1,  0xFFFF);
//		HAL_SPI_Receive(&hspi1, pBuf, size, 0xffff);
//		HAL_GPIO_WritePin(ICM_CS_GPIO_Port,ICM_CS_Pin,GPIO_PIN_SET); 
	#endif

}


/**
  *@author       JCLStill
  *@brief        SendToAanyUp 通信函数，根据不同设备改写
  *@param         void
  *@return        void
  */
void SendToUp(uint8_t *data,size_t size)
{
	 HAL_UART_Transmit(&huart1,(uint8_t *)data,size,0xff);
}

/**
  *@author       JCLStill
  *@brief        PoseCalc
  *@param         void
  *@return        void
  */
float ADChangeACC(float Filtdata ) 
{
	return Filtdata*User_set._accelRange/1000;
}
/**
  *@author       JCLStill
  *@brief        PoseCalc 角速度转换为弧度
  *@param         void
  *@return        void
  */
float ADChangeGYRO(float Filtdata )
{
	return Filtdata*User_set._gyroRange*3.1415/180;
}
 
 

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t result;
	int16_t AllData[6];
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
	printf("Start init\r\n");
	
	result = ICM42688_Readme();
	if(result == HAL_OK){
		printf("This is ICM42688\r\n");
	}else{
		printf("SomeThings error \r\n");
		return HAL_ERROR;
	}
	bool ret = true;
//  uint8_t bank = 0;
//	uint8_t value_ = 0x01;
//  writeReg(ICM42688_REG_BANK_SEL,&bank,1);
//	writeReg(ICM42688_DEVICE_CONFIG,&value_,1);
//	HAL_Delay(5);
//	value_ = 0x27;
//	writeReg(ICM42688_DRIVE_CONFIG,&value_,1);
	
	//SPI 设置 DEVICE_CONFIG DRIVE_CONFIG  INTF_CONFIG4
	
	
	////////////ACC GYRO 初始化//////////////////////////////		
	////////////如果使用APEX APEX会重新初始化某些位///////////////	
	setODRAndFSR(/* who= */GYRO,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);
  setODRAndFSR(/* who= */ACCEL,/* ODR= */ODR_1KHZ, /* FSR = */FSR_0);
	
	startTempMeasure();
  startGyroMeasure(/* mode= */LN_MODE);
  startAccelMeasure(/* mode= */LN_MODE);
	////////////FIFO使能////////////////
	#ifdef USEFIFO
	startFIFOMode();
	#endif
	
	#ifdef INTGET
// INTGET_INTNUM号中断 脉冲模式  高电平有效  推挽模式 
	setINTMode(/*INTPin=*/INTGET_INTNUM, /*INTmode=*/0, /*INTPolarity=*/1, /*INTDriveCircuit=*/1);
  enableGETDATAInterrupt();
	#endif
	
	
	#ifdef TILTDETE	 //倾斜检测
	// TILEDETE_INTNUM号中断 脉冲模式  高电平有效  推挽模式 
	setINTMode(/*INTPin=*/TILEDETE_INTNUM, /*INTmode=*/0, /*INTPolarity=*/1, /*INTDriveCircuit=*/1);
	tiltDetectionInit();	
	setTiltDetection();
	enableTILEInterrupt();	
	#endif
	
	
	#ifdef TAPDETE	 //敲击检测
	// TAPDETE_INTNUM号中断 脉冲模式  高电平有效  推挽模式 
	setINTMode(/*INTPin=*/TAPDETE_INTNUM, /*INTmode=*/0, /*INTPolarity=*/1, /*INTDriveCircuit=*/1);
	tapDetectionInit(/* accelMode= */1);
	#endif
	
	  
	#ifdef SIGNMOVE  //显著运动检测
	//1号中断 脉冲模式  高电平有效  推挽模式 
	setINTMode(/*INTPin=*/SIGNMOVE_INTNUM, /*INTmode=*/0, /*INTPolarity=*/1, /*INTDriveCircuit=*/1);
	wakeOnMotionInit();
	setWOMTh(/*axis=*/ALL,/*threshold=*/98);
	enableSMDInterrupt(/*mode=*/2);
	#endif
	
	#ifdef WAKEMOVE  //运动唤醒检测
	//1号中断 脉冲模式  高电平有效  推挽模式 
	setINTMode(/*INTPin=*/WAKEMOVE_INTNUM, /*INTmode=*/0, /*INTPolarity=*/1, /*INTDriveCircuit=*/1);
	wakeOnMotionInit();
	setWOMTh(/*axis=*/ALL,/*threshold=*/98);
	setWOMInterrupt(/* axis = */X_AXIS_WOM|Y_AXIS_WOM|Z_AXIS_WOM);
	#endif	
	
	
	
	
	printf("Init finsh ,start");
		
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {		
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//////////////////////////////////////////////////////////////////////////////////////////////
	//IMU读取简单测试，不要和其它函数一起调用/////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//samepleGetTest(ACCFloat,GYROFloat);
	////////////获取FIFO数据////////////////
	#ifdef INTGET
		if(GETDATAFLAG == 1)
		{
			GETDATAFLAG = 0 ;
	#endif
		
		
	#ifdef USEFIFO
		getFIFOData();
	#endif		
	
	tempData=    getTemperature();
	ACCInt.X	= getInt16AccelDataX();
	ACCInt.Y	= getInt16AccelDataY();	
	ACCInt.Z	= getInt16AccelDataZ();
		
	GYROInt.X	= getInt16GyroDataX();
	GYROInt.Y	= getInt16GyroDataY();	
	GYROInt.Z	= getInt16GyroDataZ();	
	
  ACCFloat.X = ADchange(ACCInt.X,User_set._accelRange);
	ACCFloat.Y = ADchange(ACCInt.Y,User_set._accelRange);
	ACCFloat.Z = ADchange(ACCInt.Z,User_set._accelRange);
		
	GYROFloat.X= ADchange(GYROInt.X,User_set._gyroRange);
  GYROFloat.Y= ADchange(GYROInt.Y,User_set._gyroRange);
  GYROFloat.Z= ADchange(GYROInt.Z,User_set._gyroRange);						

	#ifdef COMPRINTF_FLOAT
	printf("Temperature: %.2f C",tempData);
  printf("\r\n");

  printf("Accel_X: %.2f mg ",ACCFloat.X);
	printf("   Accel_Y: %.2f mg",ACCFloat.Y);
  printf("    Accel_Z: %.2f mg",ACCFloat.Z);
  printf("\r\n");
		
  printf("Gyro_X:  %.2f dps ",GYROFloat.X);
	printf("    Gyro_y:  %.2f dps ",GYROFloat.Y);
	printf("    Gyro_Z:  %.2f dps",GYROFloat.Z);
  printf("\r\n");	
	#endif
		
////////////////////////////////////////////////////

////////////////////////////////////////////////////////
	
	#ifdef USEUPCOM
	int16_t AllData[6];	
	AllData[0]=ACCFloat.X;
	AllData[1]=ACCFloat.Y;
	AllData[2]=ACCFloat.Z;
	AllData[3]=GYROFloat.X;	
	AllData[4]=GYROFloat.Y;	
	AllData[5]=GYROFloat.Z;	
	SendToUpcomSensor(AllData,0);
	#endif
	
	//////////////////////////////////////////////////////////////////////////////////////////////
	//姿态解算////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////
	//送入IMU获取的 ACCInt,GYROInt
	#ifdef POSEEN
	Prepare_Data(ACCInt,GYROInt); 												//获取姿态解算所需数据	
	IMUupdate(&Gyr_filt,&Acc_filt,&Att_Angle); 						//四元数姿态解算
	#endif

	#ifdef USEUPCOM
	int16_t Eulervalue[3];	
	Eulervalue[0]=Att_Angle.rol;
	Eulervalue[1]=Att_Angle.pit;
	Eulervalue[2]=Att_Angle.yaw;
	SendToUpcomEuler(Eulervalue,3);
	#endif
	////////////////////////////////////////////////////////////////////////////////////////////
	#ifdef POLLGET
		HAL_Delay(1);
	#endif
	
	#ifdef INTGET	
		}
	#endif
	}

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint8_t status;
	#ifdef INTGET
		GETDATAFLAG = 1;
	status= readInterruptStatus(/* reg= */ICM42688_INT_STATUS);
	#endif

	
	#ifdef TILEDETE
	if(GPIO_Pin== (0x0040 >>TILEDETE_INTNUM) )
	 {
		status= readInterruptStatus(/* reg= */ICM42688_INT_STATUS3);
		 printf("status %x\r\n",status);
    if(status & ICM42688_TILT_DET_INT){
      printf("TILE HAPPEN \r\n");
    }
	 }
	 printf("TILE \r\n");
	 printf("\r\n");
	#endif
	 
	 
	#ifdef TAPDETE	
  if(GPIO_Pin== (0x0040 >>TAPDETE_INTNUM) )
  {		
		uint32_t tapNum;
		uint8_t tapAxis;
    status= readInterruptStatus(/* reg= */ICM42688_INT_STATUS3);
//		printf("status %x\r\n",status);
    if(status & ICM42688_TAP_DET_INT){
      getTapInformation();  //Get tap information
      tapNum = numberOfTap();  //Get the number of tap: single-tap TAP_SINGLE or double tap TAP_DOUBLE
      tapAxis = axisOfTap();  //Get the axis on which tap occurred: X_AXIS, Y_AXIS or Z_AXIS
      if(tapAxis == X_AXIS){
        printf("X axis: ");
      } else if(tapAxis == Y_AXIS){
        printf("Y axis: ");
      } else if(tapAxis == Z_AXIS){
        printf("Z axis: ");
      }
      if(tapNum == TAP_SINGLE){
        printf("Single\r\n");
      } else if(tapNum == TAP_DOUBLE){
        printf("Double\r\n");
		
      }
    }

	 }
		#endif	 
	 
	 
	#ifdef SIGNMOVE
  if(GPIO_Pin== (0x0040 >>SIGNMOVE_INTNUM) )
  {
		status= readInterruptStatus(/*reg=*/ ICM42688_INT_STATUS2);
    if(status & ICM42688_SMD_INT){
      printf("SMD_INT\r\n");
    }
	 }
	#endif
	 	 
	#ifdef WAKEMOVE
	if(GPIO_Pin== (0x0040 >>WAKEMOVE_INTNUM) )
	 {
	 status= readInterruptStatus(/*reg=*/ ICM42688_INT_STATUS2);
    if(status & ICM42688_WOM_X_INT){
      printf("X_AXIS_WOM ");
    }
    if(status & ICM42688_WOM_Y_INT){
      printf("Y_AXIS_WOM ");
    }
    if(status & ICM42688_WOM_Z_INT){
     printf("Z_AXIS_WOM ");
    }
	 printf("\r\n");
	 }

	#endif
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
