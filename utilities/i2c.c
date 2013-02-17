/*
 * i2c.c
 *
 *  Created on: 4/01/2013
 *      Author: Florian
 */

#include "i2c.h"
#include "cpal_i2c.h"

CPAL_InitTypeDef   I2C1_DevStructure;
CPAL_TransferTypeDef    TX_Transfer , RX_Transfer;

bool i2c_init()
{
    CPAL_I2C_StructInit(&I2C1_DevStructure);
    I2C1_DevStructure.pCPAL_I2C_Struct->I2C_ClockSpeed = 100000;
    I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
    //I2C1_DevStructure.wCPAL_Options = CPAL_OPT_DMATX_TCIT | CPAL_OPT_DMATX_HTIT;

    TX_Transfer.pbBuffer = 0;
    TX_Transfer.wNumData = 0;
    RX_Transfer.pbBuffer = 0;
    RX_Transfer.wNumData = 0;
    I2C1_DevStructure.pCPAL_TransferTx = &TX_Transfer ;
    I2C1_DevStructure.pCPAL_TransferRx = &RX_Transfer ;

    return (CPAL_I2C_Init(&I2C1_DevStructure) == CPAL_PASS) ? true : false;
}



bool i2c_readRegister(uint8_t dev, uint8_t reg, uint8_t* data)
{

	RX_Transfer.pbBuffer = data;
	RX_Transfer.wNumData = 1;
	RX_Transfer.wAddr1 = dev<<1;
	RX_Transfer.wAddr2 = reg;

	uint32_t reading = CPAL_I2C_Read(&I2C1_DevStructure);
	if(reading == CPAL_PASS)
	{
		//while(RX_Transfer.wNumData)
		while(I2C1_DevStructure.CPAL_State != CPAL_STATE_READY)
			if(I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
			{
				//__CPAL_I2C_HAL_SWRST(I2C1_DevStructure);
				return false;
			}


		return true;
	}
	else
		return false;
}

bool i2c_readBulk(uint8_t dev, uint8_t reg, uint8_t num, uint8_t* data)
{

	RX_Transfer.pbBuffer = data;
	RX_Transfer.wNumData = num;
	RX_Transfer.wAddr1 = dev<<1;
	RX_Transfer.wAddr2 = reg | (1<<7); // multi byte read

	uint32_t reading = CPAL_I2C_Read(&I2C1_DevStructure);
	if(reading == CPAL_PASS)
	{
		//while(RX_Transfer.wNumData)
		while(I2C1_DevStructure.CPAL_State != CPAL_STATE_READY)
			if(I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
			{
				//__CPAL_I2C_HAL_SWRST(I2C1_DevStructure);
				return false;
			}


		return true;
	}
	else
		return false;
}


bool i2c_writeRegister(uint8_t dev, uint8_t reg, uint8_t data)
{

	TX_Transfer.pbBuffer = &data;
	TX_Transfer.wNumData = 1;
	TX_Transfer.wAddr1 = dev<<1;
	TX_Transfer.wAddr2 = reg;

	uint32_t written = CPAL_I2C_Write(&I2C1_DevStructure);
	if(written == CPAL_PASS)
	{
		while(I2C1_DevStructure.CPAL_State != CPAL_STATE_READY)
			if(I2C1_DevStructure.CPAL_State == CPAL_STATE_ERROR)
			{
				//__CPAL_I2C_HAL_SWRST(I2C1_DevStructure);
				return false;
			}


		return true;
	}
	else
		return false;
}


