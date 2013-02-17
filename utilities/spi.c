/*
 * spi.c
 *
 *  Created on: 4/01/2013
 *      Author: Florian
 */

#include "spi.h"
#include "stm32f4xx_spi.h"
#include "stm32f4xx_gpio.h"

/* select the peripheral: Chip Select pin low */
void spi_CS_low()
{
	GPIO_ResetBits(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN);
}

/* deselect peripheral: Chip Select pin high */
void spi_CS_high()
{
	GPIO_SetBits(SPI_CS_GPIO_PORT, SPI_CS_GPIO_PIN);
}


bool spi_init()
{
	//  --- clocks ----

	GPIO_InitTypeDef GPIO_InitStructure;

	/*!< Enable the SPI clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/*!< Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(SPI_SCK_GPIO_CLK | SPI_MISO_GPIO_CLK |
						 SPI_MOSI_GPIO_CLK | SPI_CS_GPIO_CLK, ENABLE);

	//  --- GPIOs ----

	/*!< Connect SPI pins to AF */
	GPIO_PinAFConfig(SPI_SCK_GPIO_PORT,  SPI_SCK_GPIO_PINSOURCE,  GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_MISO_GPIO_PORT, SPI_MISO_GPIO_PINSOURCE, GPIO_AF_SPI2);
	GPIO_PinAFConfig(SPI_MOSI_GPIO_PORT, SPI_MOSI_GPIO_PINSOURCE, GPIO_AF_SPI2);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;

	/*!< SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = SPI_SCK_GPIO_PIN;
	GPIO_Init(SPI_SCK_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MOSI pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI_MOSI_GPIO_PIN;
	GPIO_Init(SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);

	/*!< SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin =  SPI_MISO_GPIO_PIN;
	GPIO_Init(SPI_MISO_GPIO_PORT, &GPIO_InitStructure);

	/*!< Configure CS pin in output pushpull mode ********************/
	GPIO_InitStructure.GPIO_Pin = SPI_CS_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPI_CS_GPIO_PORT, &GPIO_InitStructure);



	// ---- SPI ----

	SPI_InitTypeDef  SPI_InitStructure;

	/*!< Deselect bus: Chip Select high */
	spi_CS_high();

	/*!< SPI configuration */
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // lets go slow first

	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	/*!< Enable the sFLASH_SPI  */
	SPI_Cmd(SPI2, ENABLE);

	return true;
}


uint8_t spi_sendByte(uint8_t c)
{
	/*!< Loop while DR register is not empty */
	uint32_t timeout = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET )
		if(timeout++ >= SPI_BUSY_TIMEOUT)
			return 0xFF;

	/*!< Send byte through the SPI1 peripheral */
	SPI_I2S_SendData(SPI2, c);

	/*!< Wait to receive a byte */
	timeout = 0;
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
		if(timeout++ >= SPI_BUSY_TIMEOUT)
			return 0xFF;


	/*!< Return the byte read from the SPI bus */
	return SPI_I2S_ReceiveData(SPI2);
}



