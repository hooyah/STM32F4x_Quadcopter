/*
 * delay.h
 *
 *  Created on: 5/01/2013
 *      Author: Florian
 */

#ifndef DELAY_H_
#define DELAY_H_

#include <stdint.h>


#if __STDC_VERSION__>=199901L
#define asm __asm__
#endif

#ifndef FCPU
#error "FCPU not not found. Please specify the cpu speed by defining FCPU"
#endif

#define CYCLES_PER_US_LOOP 3
#define DELAY_US_MULT (FCPU/1000000/CYCLES_PER_US_LOOP)


void delay_ms(volatile uint32_t nCount);

static inline void delay_us(uint32_t us)
{
    us *= DELAY_US_MULT;

    asm volatile("   mov r0, %[us]          \n\t"
                 "1: subs r0, #1            \n\t"
                 "   bhi 1b                 \n\t"
                 :
                 : [us] "r" (us)
                 : "r0");
}

#endif /* DELAY_H_ */
