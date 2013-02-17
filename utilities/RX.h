/*
 * RX.h
 *
 *  Created on: 11/01/2013
 *      Author: Florian
 */

#ifndef RX_H_
#define RX_H_


#define RX_NUM_CHANNELS 6
#define RX_RECYCLE_TIMEOUT 8000


void init_Rx();

extern __IO uint16_t RXchan[RX_NUM_CHANNELS];
extern __IO uint8_t RXnumChannelsSampled;

#endif /* RX_H_ */
