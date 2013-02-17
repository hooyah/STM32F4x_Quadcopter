/*
 * delay.c
 *
 *  Created on: 5/01/2013
 *      Author: Florian
 */
#include "delay.h"
#include "sysTick.h"



void delay_ms(volatile uint32_t nCount)
{
	//while(nCount--)
	//	delay_us(1000);
	nCount *= 1000;
	volatile uint32_t time = getTime_us();
	while(timeElapsed_us(time) < nCount);
}
