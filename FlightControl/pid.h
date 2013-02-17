/*
 * pid.h
 *
 *  Created on: 20/01/2013
 *      Author: Florian
 */

#ifndef PID_H_
#define PID_H_

typedef struct
{
	float Kp;
	float Ki;
	float Kd;

	float integErr;
	float lastErr;
	float Ki_clamp;
}PID;


void pid_init(PID* pid_struct, float p, float i, float d);
float pid_update(PID* pid_struct, float error, float dt);
void pid_resetInteg(PID* pid_struct);

/*
inline void pid_init(PID* pid_struct, float p, float i, float d)
{
	pid_struct->Kp = p;
	pid_struct->Ki = i;
	pid_struct->Kd = d;
	pid_struct->integErr = 0;
	pid_struct->lastErr = 0;
	pid_struct->Ki_clamp = 100000;
}

inline float pid_update(PID* pid_struct, float error, float dt)
{
	float deriv = (error - pid_struct->lastErr) / dt;
	pid_struct->integErr += error * dt;

	pid_struct->lastErr = error;

	return error * pid_struct->Kp + pid_struct->integErr * pid_struct->Ki + deriv * pid_struct->Kd;
}

inline void pid_resetInteg(PID* pid_struct)
{
	pid_struct->integErr = 0;
}
*/

#endif /* PID_H_ */
