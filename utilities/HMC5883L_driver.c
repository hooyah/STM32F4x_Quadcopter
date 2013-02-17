/*
 * HMC5883L_driver.c
 *
 *  Created on: Sep 17, 2012
 *      Author: Florian
 */

#ifndef HMC5883L_DRIVER_C_
#define HMC5883L_DRIVER_C_

#include "HMC5883L_driver.h"
#include "i2c.h"




uint8_t hmc5883l_ReadReg(uint8_t Reg, uint8_t* Data)
{
	return i2c_readRegister(HMC5883L_DEV_ADDR, Reg, Data);
}

uint8_t hmc5883l_WriteReg(uint8_t Reg, uint8_t Data)
{
	return i2c_writeRegister(HMC5883L_DEV_ADDR, Reg, Data);
}

uint8_t hmc5883l_ReadRegs(uint8_t StartReg, uint32_t num, uint8_t* Data)
{
	return i2c_readBulk(HMC5883L_DEV_ADDR, StartReg, num, Data);
}



HMC5883L_status_t hmc5883l_detect()
{
uint8_t whoami[3] = {0,0,0};

	if(!hmc5883l_ReadRegs(HMC5883L_ID_A, 3, whoami))
	    return HMC5883L_ERROR;
	if(whoami[0] != I_AM_HMC5883L_A || whoami[1] != I_AM_HMC5883L_B || whoami[2] != I_AM_HMC5883L_C)
	    return HMC5883L_ERROR;

	return HMC5883L_SUCCESS;
}


HMC5883L_status_t hmc5883l_setMode(HMC5883L_mode_t mode)
{
uint8_t reg = mode & 0xf;

	if(!hmc5883l_WriteReg(HMC5883L_MODE, reg))
		return HMC5883L_ERROR;

	return HMC5883L_SUCCESS;
}








#endif /* HMC5883L_DRIVER_C_ */
