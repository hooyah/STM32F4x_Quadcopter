/*
 * flightControl.c
 *
 *  Created on: 30/01/2013
 *      Author: Florian
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pid.h"
#include "servo.h"
#include "global.h"
#include "eeprom.h"
#include "terminal.h"

#define M_PI 3.1415926535

PID pid_pitchAbsolute, pid_pitchRelative;
PID pid_rollAbsolute, pid_rollRelative;
PID pid_yawAbsolute, pid_yawRelative;

int16_t ctrl_deadPoint;
float ctrl_gain_pitchAbsolute, ctrl_gain_rollAbsolute, ctrl_gain_yawAbsolute;
float ctrl_gain_pitchRelative, ctrl_gain_rollRelative, ctrl_gain_yawRelative;
float freeFlightAim_pitch, freeFlightAim_roll, freeFlightAim_yaw;






void init_flightController()
{
//	pid_init( &pid_pitch, 15.5, 0.0001, 4 );
//	pid_init( &pid_roll , 15.5, 0.0001, 4 );
//	pid_init( &pid_yaw,   25.0, 0.0001, 3 );

	ctrl_deadPoint = 6;

	freeFlightAim_pitch = 0;
	freeFlightAim_roll  = 0;
	freeFlightAim_yaw   = 0;

	//ctrl_gain_pitch = M_PI / 300;
	//ctrl_gain_roll  = M_PI / 300;
	//ctrl_gain_yaw   = M_PI / 150;


	read_params();
}


//          m0(ccw)
//          ^
// (cw)m1 --|-- m2 (cw)
//          |
//          m3(ccw)
float _motor[4];
void setMotorResponsePlus(float pitchForce, float rollForce, float yawForce, float thrust)
{
	_motor[0] = _motor[1] = _motor[2] = _motor[3] = thrust;

	float ctrlWeight = linstep(20, 40, thrust);
	pitchForce *= ctrlWeight;
	rollForce *= ctrlWeight;
	yawForce *= ctrlWeight;

	_motor[0] -= pitchForce;
	_motor[3] += pitchForce;
	// in case of maxing out a motor, reduce the opposite motor instead
//	if(abs(pitchForce)+thrust > 100) {
//		float offs = abs(pitchForce)+thrust-100;
//		_motor[0] -= offs;
//		_motor[3] -= offs;
//	}

	_motor[1] -= rollForce;
	_motor[2] += rollForce;
	// in case of maxing out a motor, reduce the opposite motor instead
//	if(abs(rollForce)+thrust > 100) {
//		float offs = abs(rollForce)+thrust-100;
//		_motor[1] -= offs;
//		_motor[2] -= offs;
//	}

	_motor[0] += yawForce;
	_motor[3] += yawForce;
	_motor[1] -= yawForce;
	_motor[2] -= yawForce;

	float offs = 0;
	if(_motor[0] > 100)
		offs = _max(offs, _motor[0] - 100);
	if(_motor[1] > 100)
		offs = _max(offs, _motor[1] - 100);
	if(_motor[2] > 100)
		offs = _max(offs, _motor[2] - 100);
	if(_motor[3] > 100)
		offs = _max(offs, _motor[3] - 100);
	if(offs > 0) {
		_motor[0] -= offs;
		_motor[1] -= offs;
		_motor[2] -= offs;
		_motor[3] -= offs;
		LedsOn(LEDR);
	}else
		LedsOff(LEDR);


	servo_setU(0, _motor[0] = clamp(0, 100, _motor[0]));
	servo_setU(1, _motor[1] = clamp(0, 100, _motor[1]));
	servo_setU(2, _motor[2] = clamp(0, 100, _motor[2]));
	servo_setU(3, _motor[3] = clamp(0, 100, _motor[3]));

//	if( thrust > 0 &&
//	   (_motor[0] >= 100 || _motor[0] < 0 ||
//	   _motor[1] >= 100 || _motor[1] < 0 ||
//	   _motor[2] >= 100 || _motor[2] < 0 ||
//	   _motor[3] >= 100 || _motor[3] < 0)  )
//		LedsOn(LEDR);
//	else
//		LedsOff(LEDR);
}



//inline
float circleConstrain(float angle)
{
	if(angle > M_PI)
		angle = -2.0 * M_PI + angle;
	else if(angle < -M_PI)
		angle =  2.0 * M_PI + angle;

	return angle;
}


void reset_absoluteFlightController(float heading_pitch, float heading_roll, float heading_yaw, float dt)
{
	freeFlightAim_pitch = 0;
	freeFlightAim_roll = 0;
	freeFlightAim_yaw = heading_yaw;
	pid_pitchAbsolute.integErr = 0;
	pid_pitchAbsolute.lastErr = heading_pitch;
	pid_rollAbsolute.integErr = 0;
	pid_rollAbsolute.lastErr = heading_roll;
	pid_yawAbsolute.integErr = 0;
	pid_yawAbsolute.lastErr = 0;
}




void update_absoluteFlightController(int16_t thrust, int16_t pitch, int16_t roll, int16_t yaw,
								int16_t aux,
								float heading_pitch, float heading_roll, float heading_yaw, float dt)
{
	LedsOn(LEDB);

	// update the goal orientation
	if( abs(pitch) > ctrl_deadPoint )
		freeFlightAim_pitch += (float)pitch * ctrl_gain_pitchAbsolute * dt;
	if( abs(roll) > ctrl_deadPoint )
		freeFlightAim_roll += roll * ctrl_gain_rollAbsolute * dt;
	if( abs(yaw) > ctrl_deadPoint)
		freeFlightAim_yaw += yaw * ctrl_gain_yawAbsolute * dt;


	float aux_bias = (float)clamp(-100, 100, aux) / 100.0 * 4;

	// ignore control when on the ground
	if(thrust < 20) {
		freeFlightAim_pitch = 0;
		freeFlightAim_roll = 0;
		freeFlightAim_yaw = heading_yaw;
		pid_resetInteg(&pid_pitchAbsolute);
		pid_resetInteg(&pid_rollAbsolute);
		pid_resetInteg(&pid_yawAbsolute);
	}

	// constrain goals to one circle
	freeFlightAim_pitch = circleConstrain(freeFlightAim_pitch);
	freeFlightAim_roll  = circleConstrain(freeFlightAim_roll);
	freeFlightAim_yaw   = circleConstrain(freeFlightAim_yaw);


	float p_d = pid_pitchAbsolute.Kd; pid_pitchAbsolute.Kd = p_d + aux_bias;
	float r_d = pid_rollAbsolute.Kd;  pid_rollAbsolute.Kd  = r_d + aux_bias;

	float pitchErr = circleConstrain( heading_pitch - freeFlightAim_pitch ); // check for quatflips
	//float pitchErr = heading_pitch - freeFlightAim_pitch ; // check for quatflips
	float pitchForce = pid_update( &pid_pitchAbsolute, pitchErr, dt);

	float rollErr = circleConstrain( heading_roll - freeFlightAim_roll );
	//float rollErr = heading_roll - freeFlightAim_roll;
	float rollForce = pid_update( &pid_rollAbsolute, rollErr, dt);

	float yawErr = circleConstrain( heading_yaw - freeFlightAim_yaw );
	//float yawErr = heading_yaw - freeFlightAim_yaw;
	float yawForce = pid_update( &pid_yawAbsolute, yawErr, dt);

	pid_pitchAbsolute.Kd = p_d; pid_rollAbsolute.Kd = r_d;

	setMotorResponsePlus(pitchForce, rollForce, yawForce, thrust);
}




int8_t printFloat_grr(float number, uint8_t numDecimals)
{
	int8_t len = 1;
	int32_t base = 1;

	if(number < 0) {
		len++;
		number = fabs(number);
		printf("-%d.", (int)number);

	}
	else
		printf("%d.", (int)number);

	base = 1;
	for(uint8_t i = 0; i < numDecimals; ++i) {
		base *= 10;
		len++;
	}

	uint32_t num = (number - (int32_t)number)*base;
	if(num > 0) {

		// remove trailing zeros
		for(; num % 10 == 0; ) {
			num /= 10;
			len--;
			base /= 10;
			numDecimals--;
		}

		for(uint32_t tbase = 10; tbase <= base; tbase *= 10) {

			num = (uint32_t)(number*tbase) % 10;
			char chr =  num;
			printf("%d", chr);
		}
	}
	else {
		len -= numDecimals - 1;
		printf("%d",0);
	}

	return len+1;
}


static uint8_t i = 0;
static int av_pos = 0;
static float p_av[8] = {0,0,0,0,0,0,0,0};
static float r_av[8] = {0,0,0,0,0,0,0,0};

void reset_relativeFlightController(float gyro_pitch, float gyro_roll, float gyro_yaw, float dt)
{
	pid_pitchRelative.integErr = 0;
	pid_pitchRelative.lastErr = 0;
	pid_rollRelative.integErr = 0;
	pid_rollRelative.lastErr = 0;
	pid_yawRelative.integErr = 0;
	pid_yawRelative.lastErr = 0;

	for(int j = 0; j < 8; j++) {
		p_av[j] = gyro_pitch;
		r_av[j] = gyro_roll;
	}
}

void update_relativeFlightController(int16_t thrust, int16_t pitch, int16_t roll, int16_t yaw,
								int16_t aux,
								float gyro_pitch, float gyro_roll, float gyro_yaw, float dt)
{
	LedsOff(LEDB);


	// update the goal orientation
	if( abs(pitch) < ctrl_deadPoint )
		pitch = 0;
	if( abs(roll) < ctrl_deadPoint )
		roll = 0;
	if( abs(yaw) < ctrl_deadPoint)
		yaw = 0;


	p_av[av_pos] = gyro_pitch;
	r_av[av_pos] = gyro_roll;

	gyro_pitch = gyro_roll = 0;
	for(int j = 0; j < 8; j++) {
		gyro_pitch += p_av[j];
		gyro_roll  += r_av[j];
	}
	gyro_pitch /= 8;
	gyro_roll /= 8;
	av_pos++;
	if(av_pos >= 8)
		av_pos = 0;



	float aux_bias = (float)clamp(-100, 100, aux) / 100.0 * 4;

	// ignore control when on the ground
	if(thrust < 20) {
		pitch = 0;
		roll = 0;
		yaw = 0;
	}
	pid_resetInteg(&pid_pitchRelative);
	pid_resetInteg(&pid_rollRelative);
	pid_resetInteg(&pid_yawRelative);



	//float p_d = pid_pitch.Kd; pid_pitch.Kd = p_d + aux_bias;
	//float r_d = pid_roll.Kd;  pid_roll.Kd  = r_d + aux_bias;

	//float pitchErr = (gyro_pitch - pitch * ctrl_gain_pitch);
	float pitchErr = gyro_roll - pitch * ctrl_gain_pitchRelative;
	float pitchForce = pid_update( &pid_pitchRelative, pitchErr, dt);

	//float pitchErr = (gyro_pitch - pitch * ctrl_gain_pitch);
	float rollErr = gyro_pitch - roll * ctrl_gain_rollRelative;
	float rollForce = pid_update( &pid_rollRelative, rollErr, dt);

	//float yawErr = (gyro_yaw - yaw * ctrl_gain_yaw);
	float yawErr = gyro_yaw - yaw * ctrl_gain_yawRelative;
	float yawForce = pid_update( &pid_yawRelative, yawErr, dt);

	//pid_pitch.Kd = p_d; pid_roll.Kd = r_d;


	//if(++i >= 200)
	{
		//uint8_t foo = printFloat_grr(pitchErr, 5);
		//printf("\r\n", foo);
		setMotorResponsePlus(pitchForce, rollForce, yawForce, thrust);
		i = 0;
	}
}
