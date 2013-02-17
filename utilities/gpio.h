/*
 * gpio.h
 *
 * brief: pin definitions for STM32F4xx
 *
 *  Created on: 12/01/2013
 *      Author: Florian
 */

#ifndef GPIO_H_
#define GPIO_H_

#ifdef STM32F4XX
	#include "stm32f4xx_conf.h"
#else
	#error "this file is only meant for STM32F4xx devices!"
#endif
#define _cat_nx(A,B) A##B
#define _cat2(A,B) _cat_nx(A,B)
#define _cat3(A,B,C) _cat2(A,_cat2(B,C))
#define _cat4(A,B,C,D) _cat2(_cat2(A,B),_cat2(C,D))



#define GPIO_PERIPH AHB1
#define GPIO_CLK_CMD _cat3(RCC_,GPIO_PERIPH,PeriphClockCmd)

#define gpio_pin_a0  (&GPIO_PORTA[0])
#define gpio_pin_a1  (&GPIO_PORTA[1])
#define gpio_pin_a2  (&GPIO_PORTA[2])
#define gpio_pin_a3  (&GPIO_PORTA[3])
#define gpio_pin_a4  (&GPIO_PORTA[4])
#define gpio_pin_a5  (&GPIO_PORTA[5])
#define gpio_pin_a6  (&GPIO_PORTA[6])
#define gpio_pin_a7  (&GPIO_PORTA[7])
#define gpio_pin_a8  (&GPIO_PORTA[8])
#define gpio_pin_a9  (&GPIO_PORTA[9])
#define gpio_pin_a10 (&GPIO_PORTA[10])
#define gpio_pin_a11 (&GPIO_PORTA[11])
#define gpio_pin_a12 (&GPIO_PORTA[12])
#define gpio_pin_a13 (&GPIO_PORTA[13])
#define gpio_pin_a14 (&GPIO_PORTA[14])
#define gpio_pin_a15 (&GPIO_PORTA[15])

#define gpio_pin_b0  (&GPIO_PORTB[0])
#define gpio_pin_b1  (&GPIO_PORTB[1])
#define gpio_pin_b2  (&GPIO_PORTB[2])
#define gpio_pin_b3  (&GPIO_PORTB[3])
#define gpio_pin_b4  (&GPIO_PORTB[4])
#define gpio_pin_b5  (&GPIO_PORTB[5])
#define gpio_pin_b6  (&GPIO_PORTB[6])
#define gpio_pin_b7  (&GPIO_PORTB[7])
#define gpio_pin_b8  (&GPIO_PORTB[8])
#define gpio_pin_b9  (&GPIO_PORTB[9])
#define gpio_pin_b10 (&GPIO_PORTB[10])
#define gpio_pin_b11 (&GPIO_PORTB[11])
#define gpio_pin_b12 (&GPIO_PORTB[12])
#define gpio_pin_b13 (&GPIO_PORTB[13])
#define gpio_pin_b14 (&GPIO_PORTB[14])
#define gpio_pin_b15 (&GPIO_PORTB[15])

#define gpio_pin_c0  (&GPIO_PORTC[0])
#define gpio_pin_c1  (&GPIO_PORTC[1])
#define gpio_pin_c2  (&GPIO_PORTC[2])
#define gpio_pin_c3  (&GPIO_PORTC[3])
#define gpio_pin_c4  (&GPIO_PORTC[4])
#define gpio_pin_c5  (&GPIO_PORTC[5])
#define gpio_pin_c6  (&GPIO_PORTC[6])
#define gpio_pin_c7  (&GPIO_PORTC[7])
#define gpio_pin_c8  (&GPIO_PORTC[8])
#define gpio_pin_c9  (&GPIO_PORTC[9])
#define gpio_pin_c10 (&GPIO_PORTC[10])
#define gpio_pin_c11 (&GPIO_PORTC[11])
#define gpio_pin_c12 (&GPIO_PORTC[12])
#define gpio_pin_c13 (&GPIO_PORTC[13])
#define gpio_pin_c14 (&GPIO_PORTC[14])
#define gpio_pin_c15 (&GPIO_PORTC[15])

#define gpio_pin_d0  (&GPIO_PORTD[0])
#define gpio_pin_d1  (&GPIO_PORTD[1])
#define gpio_pin_d2  (&GPIO_PORTD[2])
#define gpio_pin_d3  (&GPIO_PORTD[3])
#define gpio_pin_d4  (&GPIO_PORTD[4])
#define gpio_pin_d5  (&GPIO_PORTD[5])
#define gpio_pin_d6  (&GPIO_PORTD[6])
#define gpio_pin_d7  (&GPIO_PORTD[7])
#define gpio_pin_d8  (&GPIO_PORTD[8])
#define gpio_pin_d9  (&GPIO_PORTD[9])
#define gpio_pin_d10 (&GPIO_PORTD[10])
#define gpio_pin_d11 (&GPIO_PORTD[11])
#define gpio_pin_d12 (&GPIO_PORTD[12])
#define gpio_pin_d13 (&GPIO_PORTD[13])
#define gpio_pin_d14 (&GPIO_PORTD[14])
#define gpio_pin_d15 (&GPIO_PORTD[15])


typedef struct
{
	uint32_t 		RRC_Periph;
	uint32_t 		Pin;
	GPIO_TypeDef* 	Port;

}gpio_PinStruct;

typedef gpio_PinStruct* gpio_Pin;

extern gpio_PinStruct GPIO_PORTA[16];
extern gpio_PinStruct GPIO_PORTB[16];
extern gpio_PinStruct GPIO_PORTC[16];
extern gpio_PinStruct GPIO_PORTD[16];


void gpio_setToOutput(gpio_Pin pin, GPIOSpeed_TypeDef speed, GPIOOType_TypeDef PPtype);



#endif /* GPIO_H_ */
