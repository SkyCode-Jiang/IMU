/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "timer.h"
#include "gpio.h"

// system drivers
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_syscfg.h"

void timer_configure_fsync_pwm_30Hz(unsigned timer_num)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	/* Compute the prescaler value to get TIM1 counter clock at 100MHz 
	   Prescaler = (TIM1CLK / TIM1 counter clock) - 1
	   Prescaler = (SystemCoreClock /100 MHz) - 1

	   To get TIM1 output clock at 30 Hz, the period (ARR)) is computed as follows:
	   ARR = (TIM1 counter clock / TIM1 output clock) - 1
		   = 3332
	*/
	TIM_TimeBaseStructure.TIM_Period = 3332;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 100000) - 1;;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = (uint16_t) (3332 / 2) /* TIM Channel2 duty cycle : 50% */;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
		
	if(timer_num == TIMER1) {
			
		/* TIM clock enable */
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

		gpio_init_fsync_pin(GPIO_PA9);

		/* Time base configuration */
		TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
		
		/* PWM1 Mode configuration: Channel2 */
		TIM_OC2Init(TIM1, &TIM_OCInitStructure);

		/* TIM1 enable counter */
		TIM_Cmd(TIM1, ENABLE);

		/* Main Output Enable */
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
		
	} else if(timer_num == TIMER2) {
		/* TIM clock enable */
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

		gpio_init_fsync_pin(GPIO_PB3);
		
		/* Time base configuration */
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

		/* PWM1 Mode configuration: Channel2 */
		TIM_OC2Init(TIM2, &TIM_OCInitStructure);

		/* TIM2 enable counter */
		TIM_Cmd(TIM2, ENABLE);

		/* Main Output Enable */
		TIM_CtrlPWMOutputs(TIM2, ENABLE);
	}
}
