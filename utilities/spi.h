/*
 * spi.h
 *
 *  Created on: 4/01/2013
 *      Author: Florian
 */

#ifndef SPI_H_
#define SPI_H_

#include "stdint.h"
#include "stdbool.h"

#define SPI_GPIO_CLOCK_CMD RCC_AHB1Periph_SPI

#define SPI_CS_GPIO_PORT			GPIOB
#define SPI_CS_GPIO_CLK				RCC_AHB1Periph_GPIOB
#define SPI_CS_GPIO_PIN				GPIO_Pin_12
#define SPI_CS_GPIO_PINSOURCE		GPIO_PinSource12

#define SPI_MISO_GPIO_PORT			GPIOB
#define SPI_MISO_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define SPI_MISO_GPIO_PIN			GPIO_Pin_14
#define SPI_MISO_GPIO_PINSOURCE		GPIO_PinSource14

#define SPI_MOSI_GPIO_PORT			GPIOB
#define SPI_MOSI_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define SPI_MOSI_GPIO_PIN			GPIO_Pin_15
#define SPI_MOSI_GPIO_PINSOURCE		GPIO_PinSource15

#define SPI_SCK_GPIO_PORT			GPIOB
#define SPI_SCK_GPIO_CLK			RCC_AHB1Periph_GPIOB
#define SPI_SCK_GPIO_PIN			GPIO_Pin_13
#define SPI_SCK_GPIO_PINSOURCE		GPIO_PinSource13


#define SPI_BUSY_TIMEOUT 100000







bool spi_init();
uint8_t spi_sendByte(uint8_t c);
void spi_CS_low();
void spi_CS_high();



#endif /* SPI_H_ */
