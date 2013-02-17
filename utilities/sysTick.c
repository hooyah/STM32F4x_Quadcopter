/*
 * sysTick.c
 *
 *  Created on: 16/01/2013
 *      Author: Florian
 */
#include <stdint.h>
#include <stm32f4xx_conf.h>
#include "sysTick.h"

static volatile uint32_t systime = 0;

void sysTick_init()
{
	// configure systick for 100microsecond resolution
	SysTick_Config(SystemCoreClock / (1000000 / SYSTICK_RESOLUTION_US));

}

uint32_t getTime_us()
{
	return systime * SYSTICK_RESOLUTION_US;
}

uint32_t timeElapsed_us(uint32_t since)
{
uint32_t now = getTime_us();

	if(since <= now)
		return (now - since);
	else
		return (0xffffffff - since + now);;
}


extern void CPAL_Timeout_Handler(void);

void SysTick_Handler(void)
{
static uint8_t msCounter = 10;

	systime++;


	if(msCounter)
		--msCounter;
	else {
		msCounter=10;
		CPAL_Timeout_Handler();
	}
}
