/*
 * servo.c
 *
 *  Created on: 11/01/2013
 *      Author: Florian
 */

#include "servo.h"
#include "stm32f4xx_conf.h"
#include "gpio.h"

// running on timer4

static uint16_t channel_pulseLength[SERVO_NUM_CHANNELS];


#define SERVO_PRESC ((uint16_t)(PCLK1*2/0xffff/SERVO_UPDATE_RATE+1))
#define SECS_PER_TICK     (1.0 /(PCLK1*2/(SERVO_PRESC+1)))
#define US_PER_TICK (1000000.0 /(PCLK1*2/(SERVO_PRESC+1)))
#define SECS_PER_UPDATE (1.0 / SERVO_UPDATE_RATE)
#define SERVO_OVERF ((uint16_t)(SECS_PER_UPDATE / SECS_PER_TICK))


void servo_init()
{
	for(uint8_t i = 0; i < SERVO_NUM_CHANNELS; ++i)
		channel_pulseLength[i] = 12000;


	// output pins
	gpio_setToOutput(SERVO_CHAN0_PIN, GPIO_Speed_50MHz, GPIO_OType_PP);
	gpio_setToOutput(SERVO_CHAN1_PIN, GPIO_Speed_50MHz, GPIO_OType_PP);
	gpio_setToOutput(SERVO_CHAN2_PIN, GPIO_Speed_50MHz, GPIO_OType_PP);
	gpio_setToOutput(SERVO_CHAN3_PIN, GPIO_Speed_50MHz, GPIO_OType_PP);



	// timers

	/* TIM4 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* Enable the TIM4 gloabal Interrupt */
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);


	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;


	TIM_TimeBaseStructure.TIM_Period = SERVO_OVERF;//0xffff;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;//prescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	TIM_PrescalerConfig(TIM4, SERVO_PRESC, TIM_PSCReloadMode_Immediate);

	/* Output Compare Timing Mode configuration: Channel1 */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0x00ff;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Disable);


	/* TIM Interrupts enable */
	TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update, ENABLE);

	/* TIM3 enable counter */
	TIM_Cmd(TIM4, ENABLE);

}




// a serco channel is centered around 0 with an amplitude of [-100,100]
// this is a loose definition, some receivers can over-drive a servo channel and values of [-150,150] can be received
// normal deflection [-100,100] => [1000us,2000us]
// 150% => [-150.150] =? [750us,2250]
void servo_setS(uint8_t chan, int16_t val)
{
	val = clamp(-150, 150, val);

	// map [-150,150]  => [750,2250]
	uint32_t value = val * 5 + 1500;

	// convert to timer ticks
	value = (value * 10000) / ((int32_t)(US_PER_TICK * 10000));

	channel_pulseLength[chan] = value;
}

// for escs [0,100] => [1000us,2000us]
void servo_setU(uint8_t chan, uint16_t val)
{
	val = clamp(0, 100, val);

	// map [0,100]  => [1000,2000]
	uint32_t value = val * 10 + 1000;

	// convert to timer ticks
	value = (value * 10000) / ((int32_t)(US_PER_TICK * 10000));

	channel_pulseLength[chan] = value;
}


/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		TIM_SetCompare1(TIM4, channel_pulseLength[0]);
		TIM_SetCompare2(TIM4, channel_pulseLength[1]);
		TIM_SetCompare3(TIM4, channel_pulseLength[2]);
		TIM_SetCompare4(TIM4, channel_pulseLength[3]);

		GPIO_SetBits(SERVO_CHAN0_PIN->Port, SERVO_CHAN0_PIN->Pin);
		GPIO_SetBits(SERVO_CHAN1_PIN->Port, SERVO_CHAN1_PIN->Pin);
		GPIO_SetBits(SERVO_CHAN2_PIN->Port, SERVO_CHAN2_PIN->Pin);
		GPIO_SetBits(SERVO_CHAN3_PIN->Port, SERVO_CHAN3_PIN->Pin);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC1))
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);

		GPIO_ResetBits(SERVO_CHAN0_PIN->Port, SERVO_CHAN0_PIN->Pin);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC2))
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);

		GPIO_ResetBits(SERVO_CHAN1_PIN->Port, SERVO_CHAN1_PIN->Pin);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC3))
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);

		GPIO_ResetBits(SERVO_CHAN2_PIN->Port, SERVO_CHAN2_PIN->Pin);
	}
	else if(TIM_GetITStatus(TIM4, TIM_IT_CC4))
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);

		GPIO_ResetBits(SERVO_CHAN3_PIN->Port, SERVO_CHAN3_PIN->Pin);
	}
}


