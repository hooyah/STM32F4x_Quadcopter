/*
 * MPU6000.c
 *
 *  Created on: 4/01/2013
 *      Author: Florian
 */

#include "MPU6000.h"
#include "spi.h"
#include "delay.h"


void mpu6k_cs_wait()
{
	delay_us(10);
}

// Note: Make sure there is an adequate time between successive calls to readRegister/writeRegister for the chip
// to recognise the CS deassert! If in doubt, insert a mpu6k_cs_wait() call in between.
uint8_t mpu6k_readRegister(uint8_t addr)
{
	uint8_t ret;

	spi_CS_low();
	delay_us(1);
	spi_sendByte(addr | (1<<7));
	ret = spi_sendByte(0x00);
	spi_CS_high();

	return ret;
}

void mpu6k_readRegisters(uint8_t addr, uint8_t num, uint8_t* data)
{
	spi_CS_low();
	spi_sendByte(addr | (1<<7));

	for(;num > 0;--num, ++data)
		*data = spi_sendByte(0x00);

	spi_CS_high();
}

void mpu6k_writeRegister(uint8_t addr, uint8_t data)
{
	spi_CS_low();
	spi_sendByte(addr & ~(1<<7));
	spi_sendByte(data);
	spi_CS_high();
}

bool mpu6k_init()
{
uint8_t reg, rega;

	if(mpu6k_readRegister(MPU6K_REG_WHOAMI) != MPU6K_I_AM_MPU6K)
		return false;

	// reset
	mpu6k_writeRegister(MPU6K_REG_PWR_MGMT_1, MPU6K_BIT_DEVICE_RESET);
    delay_ms(100);  // Startup time delay

    // Disable I2C bus / FIFO
    mpu6k_writeRegister(MPU6K_REG_USER_CTRL, MPU6K_BIT_I2C_I_DIS);
    mpu6k_cs_wait();

    // Wake Up device and select GyroX clock (better performance)
    mpu6k_writeRegister(MPU6K_REG_PWR_MGMT_1, MPU6K_BIT_CLKSEL_PLLGX);
    mpu6k_cs_wait();
    // Enable all axis
    mpu6k_writeRegister(MPU6K_REG_PWR_MGMT_2, 0);
    mpu6k_cs_wait();

    // SAMPLE RATE
    mpu6k_writeRegister(MPU6K_REG_SMPLRT_DIV,0x00);     // Sample rate = 1kHz
    mpu6k_cs_wait();

    // DLPF = 42Hz (low pass filter)
    mpu6k_writeRegister(MPU6K_REG_CONFIG, MPU6K_BIT_DLPF_CFG_42HZ);
    mpu6k_cs_wait();

    // Gyro scale 1000º/s
    //    mpu6k_writeRegister(MPU6K_REG_GYRO_CONFIG, MPU6K_BIT_GYRO_CONFIG_FS_SEL_1000DEG);
    mpu6k_writeRegister(MPU6K_REG_GYRO_CONFIG, MPU6K_BIT_GYRO_CONFIG_FS_SEL_1000DEG);
    mpu6k_cs_wait();

    // Accel scale +-4g (4096LSB/g)
    mpu6k_writeRegister(MPU6K_REG_ACCEL_CONFIG, MPU6K_BIT_ACCEL_CONFIG_AFS_SEL_4G);

    // INT CFG => Interrupt on Data Ready
    //mpu6k_writeRegister(MPU6K_INT_ENABLE,BIT_RAW_RDY_EN);        // INT: Raw data ready
    //mpu6k_writeRegister(MPU6K_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read


//    mpu6k_cs_wait();
//    reg = mpu6k_readRegister(MPU6K_REG_PWR_MGMT_1);
	//printf("after PWR_MGMT_1 = %d\n", reg);

//    mpu6k_cs_wait();
//    reg = mpu6k_readRegister(MPU6K_REG_PWR_MGMT_2);
	//("after PWR_MGMT_2 = %d\n", reg);

//    mpu6k_cs_wait();
//    reg = mpu6k_readRegister(MPU6K_REG_USER_CTRL);
	//printf("after USER_CTRL = %d\n", reg);

//    mpu6k_cs_wait();
//    reg = mpu6k_readRegister(MPU6K_REG_SMPLRT_DIV);
	//printf("after SMPLRT_DIV = %d\n", reg);

//    mpu6k_cs_wait();
//    reg = mpu6k_readRegister(MPU6K_REG_CONFIG);
	//printf("after CONFIG = %d\n", reg);

//    mpu6k_cs_wait();
//    reg = mpu6k_readRegister(MPU6K_REG_GYRO_CONFIG);
	//printf("after GYRO_CONFIG = %d\n", reg);

    //mpu6k_cs_wait();
    //reg = mpu6k_readRegister(MPU6K_REG_ACCEL_CONFIG);
	//printf("after ACCEL_CONFIG = %d\n", reg);

	//printf("-----\n");

    return true;

}
