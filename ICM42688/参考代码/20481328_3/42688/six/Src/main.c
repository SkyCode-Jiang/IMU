/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "bno055.h"
#include "usart.h"
#include "string.h"
#include "lsm6dsr_reg.h"


//博世的六轴
//#define bno055
//st的六轴
//#define st

//#define test

#if defined sensor_42688
float aa,ab,ac,ga,gb,gc;
uint16_t accx,accy,accz,gryox,gryoy,gryoz;
#endif


#if defined bno055_1
struct bno055_t myBNO;		//全局变量
#endif
#if defined st

float aa,ab,ac,ga,gb,gc;
struct data {
    char x_a[10];
    char y_b[10];
    char z_c[10];
    char g_a[10];
    char g_b[10];
    char g_c[10];
};

struct data gaxyz;
stmdev_ctx_t dev_ctx;
#endif


I2C_HandleTypeDef hi2c1;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

#if defined bno055_1

s8 BNO_read(u8 dev_addr,u8 reg_addr,u8*reg_data,u8 wr_len)
{
    HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr, sizeof(reg_addr), reg_data, wr_len, 0xFF);
    return 0;

}

s8 BNO_write(u8 dev_addr,u8 reg_addr,u8*reg_data,u8 wr_len)
{
    HAL_I2C_Mem_Write(&hi2c1, dev_addr << 1, reg_addr, sizeof(reg_addr), reg_data, wr_len, 0xFF);
    return 0;
}
#endif

#if defined st
#define SENSOR_BUS hi2c1
void LSM_WriteOneByte(u8 Add,u8 WriteAddr,u8 DataToWrite)     //IIC从指定地址写入一个字节
{
    HAL_I2C_Mem_Write(&hi2c1, Add , WriteAddr, sizeof(WriteAddr),&DataToWrite, 8, 0xFF);
}
u8 LSM_ReadOneByte(u8 Add,u8 ReadAddr)                  //IIC从指定地址读取一个字节
{
    u8 data = 0;
    HAL_I2C_Mem_Read(&hi2c1, Add , ReadAddr, sizeof(ReadAddr), &data, 8, 0xFF);
    return data;
}

static int32_t platform_write(void *handle, uint8_t reg,
                              uint8_t *bufp,
                              uint16_t len)
{
    HAL_I2C_Mem_Write(handle, LSM6DSR_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    HAL_I2C_Mem_Read(handle, LSM6DSR_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);

    return 0;
}

void lsm6dsm_init(void)
{
#if 1
    uint8_t rst;

    lsm6dsr_pin_int1_route_t int1_route;
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &SENSOR_BUS;
    /* Init test platform */


    /* Restore default configuration */
    lsm6dsr_reset_set(&dev_ctx, PROPERTY_ENABLE);

//  do {
//    lsm6dsr_reset_get(&dev_ctx, &rst);
//  } while (rst);

    /* Disable I3C interface */
    lsm6dsr_i3c_disable_set(&dev_ctx, LSM6DSR_I3C_DISABLE);
    /* Set XL and Gyro Output Data Rate */
    lsm6dsr_xl_data_rate_set(&dev_ctx, LSM6DSR_XL_ODR_208Hz);
    lsm6dsr_gy_data_rate_set(&dev_ctx, LSM6DSR_GY_ODR_104Hz);
    /* Set 2g full XL scale and 250 dps full Gyro */
    lsm6dsr_xl_full_scale_set(&dev_ctx, LSM6DSR_2g);
    lsm6dsr_gy_full_scale_set(&dev_ctx, LSM6DSR_250dps);
    /* Set duration for Activity detection to 9.62 ms (= 2 * 1 / ODR_XL) */
    lsm6dsr_wkup_dur_set(&dev_ctx, 0x02);
    /* Set duration for Inactivity detection to 4.92 s (= 2 * 512 / ODR_XL) */
    lsm6dsr_act_sleep_dur_set(&dev_ctx, 0x02);
    /* Set Activity/Inactivity threshold to 62.5 mg */
    lsm6dsr_wkup_threshold_set(&dev_ctx, 0x02);
    /* Inactivity configuration: XL to 12.5 in LP, gyro to Power-Down */
    lsm6dsr_act_mode_set(&dev_ctx, LSM6DSR_XL_12Hz5_GY_PD);
    /* Enable interrupt generation on Inactivity INT1 pin */
    lsm6dsr_pin_int1_route_get(&dev_ctx, &int1_route);
    int1_route.md1_cfg.int1_sleep_change = PROPERTY_ENABLE;
    lsm6dsr_pin_int1_route_set(&dev_ctx, &int1_route);

//  /* Wait Events */
//  while (1) {
//    lsm6dsr_all_sources_t all_source;
//    /* Check if Activity/Inactivity events */
//    lsm6dsr_all_sources_get(&dev_ctx, &all_source);

//    if (all_source.wake_up_src.sleep_state) {
//      sprintf((char *)tx_buffer, "Inactivity Detected\r\n");
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }

//    if (all_source.wake_up_src.wu_ia) {
//      sprintf((char *)tx_buffer, "Activity Detected\r\n");
//      tx_com(tx_buffer, strlen((char const *)tx_buffer));
//    }
//  }

#endif
#if 0
    if(LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_WHO_AM_I) != 0x6B)
        return ;



    LSM_WriteOneByte(ACC_GYRO_ADDRESS,0x18,0x02); //disable i3c

    LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_CTRL2_G,0x40);	//  104HZ 250
    LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_CTRL1_XL,0x50);  		 // 208hz 2g
    // LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_CTRL10_C,0x30);

    //唤醒中断寄存器配置
    LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_TAP_CFG,0x90);
    LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_WAKE_UP_DUR,0x00);
    LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_WAKE_UP_THS,0x02);
    LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_MD1_CFG,0x20);
//		//6D Orientation Configuration   根据需求自行添加  （正反面值有差异）
//		LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_TAP_THS_6D,0x40);
//		LSM_WriteOneByte(ACC_GYRO_ADDRESS,LSM6DSL_CTRL8_XL,0x01);

#endif
}

//获取加速度值
void Lsm_Get_Rawacc(void)
{
    s16 ax,ay,az;


    volatile u8 buf[6];

    if((LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_STATUS_REG)&0x01)!=0)  //有数据生成
    {
        buf[0]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTX_H_XL);
        buf[1]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTX_L_XL);
        buf[2]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTY_H_XL);
        buf[3]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTY_L_XL);
        buf[4]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTZ_H_XL);
        buf[5]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTZ_L_XL);

        ax=((buf[0]<<8)|buf[1]);
        ay=((buf[2]<<8)|buf[3]);
        az=((buf[4]<<8)|buf[5]);
        //printf(" %.1f, %.1f, %.1f\n",(double)(ax*122)/1000000,(double)(ay*122)/1000000,(double)(az*122)/1000000);
        aa = ax;
        ab = ay,ac = az;

    }
}

//获取陀螺仪值
void Lsm_Get_Rawgryo(void)                                                                            //IIC协议
{
    volatile s16 gx,gy,gz;
    volatile float a,b,c;
    int16_t buf[6];
    if((LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_STATUS_REG)&0x02)!=0)                              //有数据生成
    {
        buf[0]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTX_H_G);
        buf[1]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTX_L_G);
        buf[2]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTY_H_G);
        buf[3]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTY_L_G);
        buf[4]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTZ_H_G);
        buf[5]= LSM_ReadOneByte(ACC_GYRO_ADDRESS,LSM6DSL_OUTZ_L_G);
        gx=(buf[0]<<8)|buf[1];
        gy=(buf[2]<<8)|buf[3];
        gz=(buf[4]<<8)|buf[5];
        ga = gx*70/1000,gb = gy*70/1000,gc = gz*70/1000;

    }
}


#endif
#if defined sensor_42688

u8 read(u8 Add,u8 ReadAddr)
{
    u8 data = 0;
    HAL_I2C_Mem_Read(&hi2c1, Add , ReadAddr, sizeof(ReadAddr), &data, 8, 0xFF);
    return data;
}

void write(u8 Add,u8 WriteAddr,u8 DataToWrite)
{
    HAL_I2C_Mem_Write(&hi2c1, Add , WriteAddr, sizeof(WriteAddr),&DataToWrite, 8, 0xFF);

}
void acc_data()
{
    int i;
    volatile u8 dat[6];
    volatile s16 ax,ay,az;
    volatile float a,b,c;
    write(ADDRESS,REG_BANK_SEL, 0x00);
    dat[0] = read(ADDRESS,ACCEL_DATA_X0);
    for(i=0; i<200; i++);
    dat[1] = read(ADDRESS,ACCEL_DATA_X1);
    for(i=0; i<200; i++);
    dat[2] = read(ADDRESS,ACCEL_DATA_Y0);
    for(i=0; i<200; i++);
    dat[3] = read(ADDRESS,ACCEL_DATA_Y1);
    for(i=0; i<200; i++);
    dat[4] = read(ADDRESS,ACCEL_DATA_Z0);
    for(i=0; i<200; i++);
    dat[5] = read(ADDRESS,ACCEL_DATA_Z1);
    for(i=0; i<200; i++);

    ax = ((dat[1] << 8) |dat[0]);
    ay = ((dat[3] << 8) |dat[2]);
    az = ((dat[5] << 8) |dat[4]);


    accx = ((dat[1] << 8) |dat[0]);
    accy = ((dat[3] << 8) |dat[2]);
    accz = ((dat[5] << 8) |dat[4]);
    //printf("%x,%x,%x ",accx,accy,accz);
    a = ax;
    b = ay;
    c = az;
    aa = a;
    ab = b;
    ac = c;
    //printf(".....%.3f,%.3f,%.3f\n",a*5/10000,b*5/10000,c*5/10000);

}
void gryo_data()
{
    volatile u8 dat[6];
    volatile s16 gx,gy,gz;
    volatile float a,b,c;
    write(ADDRESS,REG_BANK_SEL, 0x00);
    //if(read(ADDRESS,0x2D)&0x08)
    //{
    dat[0] = read(ADDRESS,GYRO_DATA_X0);
    dat[1] = read(ADDRESS,GYRO_DATA_X1);

    dat[4] = read(ADDRESS,GYRO_DATA_Z0);
    dat[5] = read(ADDRESS,GYRO_DATA_Z1);

    dat[2] = read(ADDRESS,GYRO_DATA_Y0);
    dat[3] = read(ADDRESS,GYRO_DATA_Y1);
    //}
    gx = ((dat[1] << 8) |dat[0]);
    gy = ((dat[3] << 8) |dat[2]);
    gz = ((dat[5] << 8) |dat[4]);

    gryox = ((dat[1] << 8) |dat[0]);
    gryoy = ((dat[3] << 8) |dat[2]);
    gryoz = ((dat[5] << 8) |dat[4]);

    a = gx,b = gy,c = gz;
    ga = a,gb = b,gc = c;

    //printf("%x,%x,%x",gryox,gryoy,gryoz);
    //printf("........%.3f, %.3f, %.3f\n",ga,gb,gc);

}
u8 id = 0;
void sensor_42688_init()
{
    uint8_t data;
    write(ADDRESS,REG_BANK_SEL,0x00);
    HAL_Delay(5);
    write(ADDRESS,0x11,0x01);
    HAL_Delay(5);
    id = read(ADDRESS,WHOAMI);
    data = read(ADDRESS,PWR_MGMT0);
    HAL_Delay(5);
    write(ADDRESS,PWR_MGMT0,data|0x0F);
    HAL_Delay(5);

    data = read(ADDRESS,GYRO_CONFIG0);
    HAL_Delay(5);
    write(ADDRESS,GYRO_CONFIG0,data|0x06);
    HAL_Delay(5);
    data = read(ADDRESS,ACCEL_CONFIG0);
    HAL_Delay(5);
    write(ADDRESS,ACCEL_CONFIG0,data|0x06);
    HAL_Delay(5);

}


#endif
int main(void)
{
    u16 buff[6];
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */
#if defined test
    uint8_t reg_data = 0x03;

#endif

#if defined bno055_1
    volatile struct bno055_euler_float_t a;
    uint8_t stat;
    volatile signed char retval,ret;
    struct bno055_euler_float_t buff;

    struct bno055_gravity_double_t d_gravity_xyz;
    struct bno055_euler_double_t d_euler_hpr;

    s16 gyro_datax = BNO055_INIT_VALUE;

    /* variable used to read the gyro y data */
    s16 gyro_datay = BNO055_INIT_VALUE;

    /* variable used to read the gyro z data */
    s16 gyro_dataz = BNO055_INIT_VALUE;

#endif

    HAL_Init();


    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    uart_init(1000000);

#if defined sensor_42688
    sensor_42688_init();
    HAL_Delay(1);
#endif

#if defined st
    HAL_Delay(10);
    lsm6dsm_init();

#endif
#if defined test
    HAL_I2C_Mem_Write(&hi2c1, 0x4c, 0x01, 8, &reg_data, 8, 0xFF);
#endif

#if defined bno055_1
    myBNO.bus_read = BNO_read;
    myBNO.bus_write = BNO_write;
    myBNO.delay_msec = HAL_Delay;

    myBNO.dev_addr= BNO055_I2C_ADDR1;
    ret = bno055_init(&myBNO);
    bno055_set_sys_rst(BNO055_BIT_ENABLE);
    HAL_Delay(700);

    //bno055_set_operation_mode(OPERATION_MODE_NDOF);
    //bno055_convert_float_euler_hpr_deg(&a);
    bno055_set_operation_mode(BNO055_OPERATION_MODE_IMUPLUS);

    while(1)
    {
        bno055_get_gyro_calib_stat(&stat);
        if (stat == 3) {
            break;
        }
        HAL_Delay(500);
    }

#endif
    while (1)
    {


#if defined bno055_1
        bno055_convert_double_gravity_xyz_msq(&d_gravity_xyz);
        bno055_convert_double_euler_hpr_deg(&d_euler_hpr);

        bno055_read_gyro_x(&gyro_datax);
        bno055_read_gyro_y(&gyro_datay);
        bno055_read_gyro_z(&gyro_dataz);



        printf("%f, %f, %f, %d, %d, %d\r\n",d_gravity_xyz.x,d_gravity_xyz.y,d_gravity_xyz.z,gyro_datax,gyro_datay,gyro_dataz);
#endif

#if defined st

        Lsm_Get_Rawacc();
        Lsm_Get_Rawgryo();
        printf("%.1f, %.1f, %.1f, %.1f ,%.1f, %.1f\n",(aa*122)/1000000,(ab*122)/1000000,(ac*122)/1000000,ga,gb,gc);
#endif

#if defined sensor_42688
        if(read(ADDRESS,0X2d)&0x08 != 0)
            acc_data();
        gryo_data();
        printf("%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n",aa/2048,ab/2048,ac/2048,ga,gb,gc);
        //printf("%x, %x, %x, %x, %x, %x\n",accx,accy,accz,gryox,gryoy,gryoz);


#endif

        //HAL_Delay(1);
    }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 168;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

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
