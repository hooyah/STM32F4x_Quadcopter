/*
 * i2c.h
 *
 *  Created on: 4/01/2013
 *      Author: Florian
 */

#ifndef I2C_H_
#define I2C_H_

#include <stdbool.h>
#include <stdint.h>

bool i2c_init();

bool i2c_readRegister(uint8_t dev, uint8_t reg, uint8_t* data);
bool i2c_writeRegister(uint8_t dev, uint8_t reg, uint8_t data);
bool i2c_readBulk(uint8_t dev, uint8_t reg, uint8_t num, uint8_t* data);


#endif /* I2C_H_ */
