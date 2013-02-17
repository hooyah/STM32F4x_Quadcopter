/*
 * pid.c
 *
 *  Created on: 26/01/2013
 *      Author: Florian
 */

#include "pid.h"

 void pid_init(PID* pid_struct, float p, float i, float d)
{
	pid_struct->Kp = p;
	pid_struct->Ki = i;
	pid_struct->Kd = d;
	pid_struct->integErr = 0;
	pid_struct->lastErr = 0;
	pid_struct->Ki_clamp = 100000;
}

 float pid_update(PID* pid_struct, float error, float dt)
{
	float deriv = (error - pid_struct->lastErr) / dt;
	pid_struct->integErr += error * dt;

	pid_struct->lastErr = error;

	return error * pid_struct->Kp + pid_struct->integErr * pid_struct->Ki + deriv * pid_struct->Kd;
}

 void pid_resetInteg(PID* pid_struct)
{
	pid_struct->integErr = 0;
}
