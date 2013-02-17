/*
 * flightControl.h
 *
 *  Created on: 20/01/2013
 *      Author: Florian
 */

#ifndef FLIGHTCONTROL_H_
#define FLIGHTCONTROL_H_

void init_flightController();

void reset_absoluteFlightController(float heading_pitch, float heading_roll, float heading_yaw, float dt);
void update_absoluteFlightController(int16_t thrust, int16_t pitch, int16_t roll, int16_t yaw,
								int16_t aux,
								float heading_pitch, float heading_roll, float heading_yaw, float dt);

void reset_relativeFlightController(float gyro_pitch, float gyro_roll, float gyro_yaw, float dt);
void update_relativeFlightController(int16_t thrust, int16_t pitch, int16_t roll, int16_t yaw,
								int16_t aux,
								float gyro_pitch, float gyro_roll, float gyro_yaw, float dt);


#endif /* FLIGHTCONTROL_H_ */
