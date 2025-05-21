/*********************************************************************
File    : gpio.h
Purpose : 
**********************************************************************/

#ifndef __gpio_H__
#define __gpio_H__

/************************** Includes ***********************************/
#include "sys.h"  


/****************************** Defines *******************************/
//开启/关闭外部中断
#define ENABLE_INV_INTERRUPTS  EnableInvInterrupt()
#define DISABLE_INV_INTERRUPTS DisableInvInterrupt()

//外部中断配置
#define INVEN_INT_PIN                         GPIO_Pin_9
#define INVEN_INT_GPIO_PORT                   GPIOB
#define INVEN_INT_GPIO_CLK                    RCC_APB2Periph_GPIOB
#define INVEN_INT_EXTI_PORT                   GPIO_PortSourceGPIOB
#define INVEN_INT_EXTI_PIN                    GPIO_PinSource9
#define INVEN_INT_EXTI_LINE                   EXTI_Line9
#define INVEN_INT_EXTI_IRQ                    EXTI9_5_IRQn     


/***************************Globals *******************************************/


/***************************** Prototypes *****************************/
void GPIO_Config(void);
void EnableInvInterrupt(void);
void DisableInvInterrupt(void);


#endif // __gpio_H__

