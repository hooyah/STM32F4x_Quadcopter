/*
 * sysTick.h
 *
 *  Created on: 16/01/2013
 *      Author: Florian
 */

#ifndef SYSTICK_H_
#define SYSTICK_H_

#define SYSTICK_RESOLUTION_US 100

void sysTick_init();
uint32_t getTime_us();
uint32_t timeElapsed_us();


#endif /* SYSTICK_H_ */
