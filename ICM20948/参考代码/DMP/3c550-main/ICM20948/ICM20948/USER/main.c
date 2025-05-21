#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"
#include "icm20948.h"

icm20948_data_t icm20948_data;

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    delay_init();
	
    uart_init(921600);
    SPI2_Init();
    GPIO_Config();
    while(ICM_20948_Init());

    while(1)
    {
        if (hal.new_gyro == 1)
        {
            hal.new_gyro = 0;
            //fifo_handler();//处理函数可放于中断
			ICM20948_Get_Data(&icm20948_data);
			printf("Accel Data\t %8.5f, %8.5f, %8.5f\r\n", icm20948_data.accel_float[0], icm20948_data.accel_float[1], icm20948_data.accel_float[2]);
			printf("Gyro Data\t %7.5f, %7.5f, %7.5f\r\n", icm20948_data.gyro_float[0], icm20948_data.gyro_float[1], icm20948_data.gyro_float[2]);
			printf("Compass Data\t %7.5f, %7.5f, %7.5f\r\n", icm20948_data.compass_float[0], icm20948_data.compass_float[1], icm20948_data.compass_float[2]);
			printf("Orientation\t %7.5f, %7.5f, %7.5f\r\n", icm20948_data.orientation[0], icm20948_data.orientation[1],icm20948_data.orientation[2]);
			printf("Temperature\t %.3f\r\n",icm20948_data.temperature);
        }
    }
}






