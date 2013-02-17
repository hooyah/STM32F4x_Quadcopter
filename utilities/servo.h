/*
 * servos.h
 *
 *  Created on: 11/01/2013
 *      Author: Florian
 */

#ifndef SERVOS_H_
#define SERVOS_H_

#include <stdint.h>

#define SERVO_NUM_CHANNELS 4
#define SERVO_UPDATE_RATE 100 // Hz

#define HCLK 168000000 // system clock
#define PCLK1 (HCLK/4)

// pins
#define SERVO_CHAN0_PIN gpio_pin_b1
#define SERVO_CHAN1_PIN gpio_pin_b0
#define SERVO_CHAN2_PIN gpio_pin_c5
#define SERVO_CHAN3_PIN gpio_pin_c4



void servo_init();

// a serco channel is centered around 0 with an amplitude of [-100,100]
// this is a loose definition, some receivers can over-drive a servo channel and values of [-150,150] can be received
void servo_setS(uint8_t chan, int16_t value);
// [0,100]
void servo_setU(uint8_t chan, uint16_t value);

#define _min(a,b) (((a)<(b))?(a):(b))
#define _max(a,b) (((a)>(b))?(a):(b))
#define clamp(MIN, MAX, VAL) _max(MIN, _min(MAX,VAL))

#define linstep(MIN, MAX, VAL) (((VAL) <= (MIN))?0.0f:( ((VAL) >= (MAX))?1.0f:( ((VAL)-(MIN))/((MAX)-(MIN)) ) ) )


#endif /* SERVOS_H_ */
