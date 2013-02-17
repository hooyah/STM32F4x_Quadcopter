/*
 * gpio.c
 *
 *  Created on: 12/01/2013
 *      Author: Florian
 */

#include "gpio.h"

gpio_PinStruct GPIO_PORTA[16] = {
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_0,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_1,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_2,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_3,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_4,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_5,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_6,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_7,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_8,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_9,  GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_10, GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_11, GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_12, GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_13, GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_14, GPIOA },
		{ RCC_AHB1Periph_GPIOA, GPIO_Pin_15, GPIOA }
};
gpio_PinStruct GPIO_PORTB[16] = {
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_0,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_1,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_2,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_3,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_4,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_5,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_6,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_7,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_8,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_9,  GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_10, GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_11, GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_12, GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_13, GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_14, GPIOB },
		{ RCC_AHB1Periph_GPIOB, GPIO_Pin_15, GPIOB }
};
gpio_PinStruct GPIO_PORTC[16] = {
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_0,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_1,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_2,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_3,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_4,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_5,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_6,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_7,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_8,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_9,  GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_10, GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_11, GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_12, GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_13, GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_14, GPIOC },
		{ RCC_AHB1Periph_GPIOC, GPIO_Pin_15, GPIOC }
};
gpio_PinStruct GPIO_PORTD[16] = {
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_0,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_1,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_2,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_3,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_4,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_5,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_6,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_7,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_8,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_9,  GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_10, GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_11, GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_12, GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_13, GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_14, GPIOD },
		{ RCC_AHB1Periph_GPIOD, GPIO_Pin_15, GPIOD }
};




void gpio_setToOutput(gpio_Pin pin, GPIOSpeed_TypeDef speed, GPIOOType_TypeDef PPtype)
{
	/* GPIOD Periph clock enable */
	GPIO_CLK_CMD(pin->RRC_Periph, ENABLE);

	/* Configure in output pushpull mode */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pin->Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = PPtype;
	GPIO_InitStructure.GPIO_Speed = speed;
	GPIO_Init(pin->Port, &GPIO_InitStructure);
}
