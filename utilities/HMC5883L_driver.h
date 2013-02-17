/*
 * HMC5883L_driver.h
 *
 *  Created on: Sep 17, 2012
 *      Author: Florian
 */

#ifndef HMC5883L_DRIVER_H_
#define HMC5883L_DRIVER_H_

#include <stdint.h>

typedef enum {
	HMC5883L_SUCCESS	=		1,
	HMC5883L_ERROR		=		0
} HMC5883L_status_t;

typedef enum {
	HMC5883L_MODE_CONTINUOUS	=		0,
	HMC5883L_MODE_SINGLE		=		1,
	HMC5883L_MODE_IDLE			=		3
} HMC5883L_mode_t;





#define HMC5883L_DEV_ADDR           0x1E // this device only has one address
#define I_AM_HMC5883L_A 'H'
#define I_AM_HMC5883L_B '4'
#define I_AM_HMC5883L_C '3'
// ---- registers -----

#define HMC5883L_CONFIG_A        0x00
#define HMC5883L_CONFIG_B        0x01
#define HMC5883L_MODE            0x02
#define HMC5883L_DATAX_H         0x03
#define HMC5883L_DATAX_L         0x04
#define HMC5883L_DATAZ_H         0x05
#define HMC5883L_DATAZ_L         0x06
#define HMC5883L_DATAY_H         0x07
#define HMC5883L_DATAY_L         0x08
#define HMC5883L_STATUS          0x09
#define HMC5883L_ID_A            0x0A
#define HMC5883L_ID_B            0x0B
#define HMC5883L_ID_C            0x0C

#define HMC5883L_CRA_AVERAGE_BIT    6
#define HMC5883L_CRA_AVERAGE_LENGTH 2
#define HMC5883L_CRA_RATE_BIT       4
#define HMC5883L_CRA_RATE_LENGTH    3
#define HMC5883L_CRA_BIAS_BIT       1
#define HMC5883L_CRA_BIAS_LENGTH    2

#define HMC5883L_AVERAGING_1        0x00
#define HMC5883L_AVERAGING_2        0x01
#define HMC5883L_AVERAGING_4        0x02
#define HMC5883L_AVERAGING_8        0x03

#define HMC5883L_RATE_0P75          0x00
#define HMC5883L_RATE_1P5           0x01
#define HMC5883L_RATE_3             0x02
#define HMC5883L_RATE_7P5           0x03
#define HMC5883L_RATE_15            0x04
#define HMC5883L_RATE_30            0x05
#define HMC5883L_RATE_75            0x06

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_CRB_GAIN_BIT       7
#define HMC5883L_CRB_GAIN_LENGTH    3

#define HMC5883L_GAIN_1370          0x00
#define HMC5883L_GAIN_1090          0x01
#define HMC5883L_GAIN_820           0x02
#define HMC5883L_GAIN_660           0x03
#define HMC5883L_GAIN_440           0x04
#define HMC5883L_GAIN_390           0x05
#define HMC5883L_GAIN_330           0x06
#define HMC5883L_GAIN_220           0x07

#define HMC5883L_MODEREG_BIT        1
#define HMC5883L_MODEREG_LENGTH     2

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0



uint8_t hmc5883l_ReadReg(uint8_t Reg, uint8_t* Data);
uint8_t hmc5883l_WriteReg(uint8_t Reg, uint8_t Data);
uint8_t hmc5883l_ReadRegs(uint8_t StartReg, uint32_t num, uint8_t* Data);

HMC5883L_status_t hmc5883l_detect();
HMC5883L_status_t hmc5883l_setMode(HMC5883L_mode_t);



#endif /* HMC5883L_DRIVER_H_ */
