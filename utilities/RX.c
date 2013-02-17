/*
 * RX.c
 *
 *  Created on: 11/01/2013
 *      Author: Florian
 */

#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "RX.h"

// Rx channeld or'ed together on Pin B4
// uses Timer3


static uint8_t currentChan = 0;
__IO uint16_t RXchan[RX_NUM_CHANNELS];
__IO uint8_t RXnumChannelsSampled = 0;


void init_Rx()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* GPIOB clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* TIM3 chennel1 configuration : PB.04 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN ;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect TIM pin to AF2 */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);



	/* TIM3 configuration: PWM Input mode ------------------------
	The external signal is connected to TIM3 CH1 pin (PB.04),
	The Rising edge is used as active edge,
	------------------------------------------------------------ */
	TIM_ICInitTypeDef  TIM_ICInitStructure;
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_PWMIConfig(TIM3, &TIM_ICInitStructure);

	/* Select the TIM3 Input Trigger: TI2FP2 */
	TIM_SelectInputTrigger(TIM3, TIM_TS_TI1FP1);

	/* Select the slave Mode: Reset Mode */
	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
	TIM_TimeBaseInitStruct.TIM_Prescaler = 32;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);

	/* TIM enable counter */
	TIM_Cmd(TIM3, ENABLE);

	/* Enable the CC2 Interrupt Request */
	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

}





/**
  * @brief  This function handles TIM3 global interrupt request.
  * @param  None
  * @retval None
  */
void TIM3_IRQHandler(void)
{
	/* Clear TIM3 Capture compare interrupt pending bit */
	//TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
	/* Get the Input Capture value */
	uint16_t IC1Value = TIM_GetCapture1(TIM3);
	uint16_t IC2Value = TIM_GetCapture2(TIM3);


	if(IC1Value > RX_RECYCLE_TIMEOUT || currentChan >= RX_NUM_CHANNELS) {
	  RXnumChannelsSampled = currentChan;
	  currentChan = 0;
  	  //GPIO_ToggleBits(GPIOC, GPIO_Pin_2);
	}

	RXchan[currentChan] = IC2Value;
	currentChan++;

}



