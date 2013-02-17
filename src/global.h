/*
 * global.h
 *
 *  Created on: 26/01/2013
 *      Author: Florian
 */

#ifndef GLOBAL_H_
#define GLOBAL_H_

#include <stdint.h>

#define RXCHAN_THUST 4
#define RXCHAN_ROLL  0
#define RXCHAN_PITCH 3
#define RXCHAN_YAW   5
#define RXCHAN_AUX   1
#define RXCHAN_GEAR  2

#define LEDR GPIO_Pin_2
#define LEDG GPIO_Pin_1
#define LEDB GPIO_Pin_0
#define LedsOn(LEDBITS) GPIO_SetBits(GPIOC, LEDBITS)
#define LedsOff(LEDBITS) GPIO_ResetBits(GPIOC, LEDBITS)
#define LedsToggle(LEDBITS) GPIO_ToggleBits(GPIOC, LEDBITS)


#define NB_OF_VAR             ((uint8_t) 48 )
#define EE_PID_PITCH_KP_ABS 0
#define EE_PID_PITCH_KI_ABS 2
#define EE_PID_PITCH_KD_ABS 4
#define EE_PID_ROLL_KP_ABS 6
#define EE_PID_ROLL_KI_ABS 8
#define EE_PID_ROLL_KD_ABS 10
#define EE_PID_YAW_KP_ABS 12
#define EE_PID_YAW_KI_ABS 14
#define EE_PID_YAW_KD_ABS 16
#define EE_CTRL_PITCH_ABS 18
#define EE_CTRL_ROLL_ABS  20
#define EE_CTRL_YAW_ABS   22

#define EE_PID_PITCH_KP_REL 24
#define EE_PID_PITCH_KI_REL 26
#define EE_PID_PITCH_KD_REL 28
#define EE_PID_ROLL_KP_REL 30
#define EE_PID_ROLL_KI_REL 32
#define EE_PID_ROLL_KD_REL 34
#define EE_PID_YAW_KP_REL 36
#define EE_PID_YAW_KI_REL 38
#define EE_PID_YAW_KD_REL 40
#define EE_CTRL_PITCH_REL 42
#define EE_CTRL_ROLL_REL  44
#define EE_CTRL_YAW_REL   46

#define FM_ABSOLUTE 0
#define FM_RELATIVE 1

//extern uint16_t VirtAddVarTab[NB_OF_VAR];


#endif /* GLOBAL_H_ */
