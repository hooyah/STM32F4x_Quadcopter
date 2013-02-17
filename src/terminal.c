/*
 * terminal.c
 *
 *  Created on: 26/01/2013
 *      Author: Florian
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_conf.h"
#include "pid.h"
#include "eeprom.h"
#include "global.h"

typedef struct
{
	char name[16];
	float *valA;
	float *valB;
}TParam;

#define RXBUFSIZE 100
bool	buffer_overrun = false;
uint8_t bufpos = 0;
uint8_t buffer[RXBUFSIZE+1];

extern PID pid_pitchAbsolute, pid_rollAbsolute, pid_yawAbsolute;
extern PID pid_pitchRelative, pid_rollRelative, pid_yawRelative;
extern float ctrl_gain_pitchAbsolute, ctrl_gain_rollAbsolute, ctrl_gain_yawAbsolute;
extern float ctrl_gain_pitchRelative, ctrl_gain_rollRelative, ctrl_gain_yawRelative;
TParam parameters[] = {
		{"pitch_Kp=", &pid_pitchAbsolute.Kp, &pid_pitchRelative.Kp},
		{"pitch_Ki=", &pid_pitchAbsolute.Ki, &pid_pitchRelative.Ki},
		{"pitch_Kd=", &pid_pitchAbsolute.Kd, &pid_pitchRelative.Kd},
		{"roll_Kp=",  &pid_rollAbsolute.Kp, &pid_rollRelative.Kp},
		{"roll_Ki=",  &pid_rollAbsolute.Ki, &pid_rollRelative.Ki},
		{"roll_Kd=",  &pid_rollAbsolute.Kd, &pid_rollRelative.Kd},
		{"yaw_Kp=",   &pid_yawAbsolute.Kp, &pid_yawRelative.Kp},
		{"yaw_Ki=",   &pid_yawAbsolute.Ki, &pid_yawRelative.Ki},
		{"yaw_Kd=",   &pid_yawAbsolute.Kd, &pid_yawRelative.Kd},
		{"ctrl_pitch=",   &ctrl_gain_pitchAbsolute, &ctrl_gain_pitchRelative},
		{"ctrl_roll=",   &ctrl_gain_rollAbsolute, &ctrl_gain_rollRelative},
		{"ctrl_yaw=",   &ctrl_gain_yawAbsolute, &ctrl_gain_yawRelative},
};



bool readln()
{
	if( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) )
	{
		buffer[bufpos] = USART_ReceiveData(USART1);
		while ( !USART_GetFlagStatus(USART1, USART_FLAG_TC ));
		USART_SendData(USART1, buffer[bufpos]);
		if(bufpos < RXBUFSIZE) {
			if(buffer[bufpos] == 27) { // ESC
				bufpos = 0;
				buffer[0] = 0;
				return true;
			}
			else if(buffer[bufpos] == 13 || buffer[bufpos] == 10) {
				buffer[bufpos] = 0;
				bufpos++;
				return true;
			}
			bufpos++;
		}
		else
		{
			buffer_overrun = true;
		}
	}
	return false;
}


void padWith(int16_t num, uint8_t chr)
{
	for(;--num > 0;) {
		while ( !USART_GetFlagStatus(USART1, USART_FLAG_TC ));
		USART_SendData(USART1, chr);
	}
}

int8_t printFloat_doodledoo(float number, uint8_t numDecimals)
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

float myatof(const char* string)
{
	int32_t ltz = atoi(string);
	while(*string != '.') {
		if(!*string) {
			return ltz;
		}
		string++;
	}
	string++;
	float remainder = 0.0f;
	float base = 0.1f;
	while(*string >= '0' && *string <= '9') {

		remainder += (float)(*string - '0') * base;
		base /= 10;
		string++;
	}
	if(ltz >= 0)
		return (float)ltz + remainder;
	else
		return (float)ltz - remainder;
}


float ee_read_float_param(uint16_t p_num)
{
	uint16_t l = 0, h = 0;
	EE_ReadVariable(p_num, &l);
	EE_ReadVariable(p_num+1, &h);
	uint32_t f = h << 16 | l;
	return *((float*)&f);
}

void ee_write_float_param(uint16_t p_num, float val)
{
	float oldval = ee_read_float_param(p_num);
	if(oldval == val)
		return;

	uint32_t *ui = (uint32_t*)&val;
	uint16_t l = (*ui)&0xffff , h = (*ui)>>16;

	EE_WriteVariable(p_num, l);
	EE_WriteVariable(p_num+1, h);
}

void read_params()
{
	FLASH_Unlock();

	float p, i, d;
	p = ee_read_float_param(EE_PID_PITCH_KP_ABS);
	i = ee_read_float_param(EE_PID_PITCH_KI_ABS);
	d = ee_read_float_param(EE_PID_PITCH_KD_ABS);
	pid_init( &pid_pitchAbsolute, p, i, d );

	p = ee_read_float_param(EE_PID_ROLL_KP_ABS);
	i = ee_read_float_param(EE_PID_ROLL_KI_ABS);
	d = ee_read_float_param(EE_PID_ROLL_KD_ABS);
	pid_init( &pid_rollAbsolute, p, i, d );

	p = ee_read_float_param(EE_PID_YAW_KP_ABS);
	i = ee_read_float_param(EE_PID_YAW_KI_ABS);
	d = ee_read_float_param(EE_PID_YAW_KD_ABS);
	pid_init( &pid_yawAbsolute, p, i, d );

	p = ee_read_float_param(EE_CTRL_PITCH_ABS);
	ctrl_gain_pitchAbsolute = p;
	p = ee_read_float_param(EE_CTRL_ROLL_ABS);
	ctrl_gain_rollAbsolute = p;
	p = ee_read_float_param(EE_CTRL_YAW_ABS);
	ctrl_gain_yawAbsolute = p;

	p = ee_read_float_param(EE_PID_PITCH_KP_REL);
	i = ee_read_float_param(EE_PID_PITCH_KI_REL);
	d = ee_read_float_param(EE_PID_PITCH_KD_REL);
	pid_init( &pid_pitchRelative, p, i, d );

	p = ee_read_float_param(EE_PID_ROLL_KP_REL);
	i = ee_read_float_param(EE_PID_ROLL_KI_REL);
	d = ee_read_float_param(EE_PID_ROLL_KD_REL);
	pid_init( &pid_rollRelative, p, i, d );

	p = ee_read_float_param(EE_PID_YAW_KP_REL);
	i = ee_read_float_param(EE_PID_YAW_KI_REL);
	d = ee_read_float_param(EE_PID_YAW_KD_REL);
	pid_init( &pid_yawRelative, p, i, d );

	p = ee_read_float_param(EE_CTRL_PITCH_REL);
	ctrl_gain_pitchRelative = p;
	p = ee_read_float_param(EE_CTRL_ROLL_REL);
	ctrl_gain_rollRelative = p;
	p = ee_read_float_param(EE_CTRL_YAW_REL);
	ctrl_gain_yawRelative = p;

	FLASH_Lock();
}

void write_params()
{
	FLASH_Unlock();

	ee_write_float_param(EE_PID_PITCH_KP_ABS, pid_pitchAbsolute.Kp);
	ee_write_float_param(EE_PID_PITCH_KI_ABS, pid_pitchAbsolute.Ki);
	ee_write_float_param(EE_PID_PITCH_KD_ABS, pid_pitchAbsolute.Kd);

	ee_write_float_param(EE_PID_ROLL_KP_ABS, pid_rollAbsolute.Kp);
	ee_write_float_param(EE_PID_ROLL_KI_ABS, pid_rollAbsolute.Ki);
	ee_write_float_param(EE_PID_ROLL_KD_ABS, pid_rollAbsolute.Kd);

	ee_write_float_param(EE_PID_YAW_KP_ABS, pid_yawAbsolute.Kp);
	ee_write_float_param(EE_PID_YAW_KI_ABS, pid_yawAbsolute.Ki);
	ee_write_float_param(EE_PID_YAW_KD_ABS, pid_yawAbsolute.Kd);

	ee_write_float_param(EE_CTRL_PITCH_ABS, ctrl_gain_pitchAbsolute);
	ee_write_float_param(EE_CTRL_ROLL_ABS, ctrl_gain_rollAbsolute);
	ee_write_float_param(EE_CTRL_YAW_ABS, ctrl_gain_yawAbsolute);


	ee_write_float_param(EE_PID_PITCH_KP_REL, pid_pitchRelative.Kp);
	ee_write_float_param(EE_PID_PITCH_KI_REL, pid_pitchRelative.Ki);
	ee_write_float_param(EE_PID_PITCH_KD_REL, pid_pitchRelative.Kd);

	ee_write_float_param(EE_PID_ROLL_KP_REL, pid_rollRelative.Kp);
	ee_write_float_param(EE_PID_ROLL_KI_REL, pid_rollRelative.Ki);
	ee_write_float_param(EE_PID_ROLL_KD_REL, pid_rollRelative.Kd);

	ee_write_float_param(EE_PID_YAW_KP_REL, pid_yawRelative.Kp);
	ee_write_float_param(EE_PID_YAW_KI_REL, pid_yawRelative.Ki);
	ee_write_float_param(EE_PID_YAW_KD_REL, pid_yawRelative.Kd);

	ee_write_float_param(EE_CTRL_PITCH_REL, ctrl_gain_pitchRelative);
	ee_write_float_param(EE_CTRL_ROLL_REL, ctrl_gain_rollRelative);
	ee_write_float_param(EE_CTRL_YAW_REL, ctrl_gain_yawRelative);

	FLASH_Lock();
}



void print_params(uint8_t mode)
{
	uint8_t num = sizeof(parameters)/ sizeof(TParam);

	printf("\r\nminimaBL 1.0 flight control parameters\r\n");
	for(uint8_t i = 0; i < num; ++i) {

		int16_t len = strlen(parameters[i].name);
		printf("%s", parameters[i].name);
		padWith(12-len, ' ');
		len = printFloat_doodledoo((mode==FM_ABSOLUTE)?(*parameters[i].valA):(*parameters[i].valB), 5);
		padWith(15-len, ' ');
		if((i+1)%3==0)
			printf("\r\n");
	}
	printf("\r\n");
}

void terminal(uint8_t mode)
{
	if(readln()) {

		if(!buffer_overrun) {

			uint8_t num = sizeof(parameters)/ sizeof(TParam);

			if(strncmp(buffer, "save", 4) == 0)
			{
				write_params();
				bufpos = 0;
				printf("parameters saved to EEprom\r\n");
				return;
			}
			for(uint8_t i = 0; i < num; ++i) {

				uint8_t len = strlen(parameters[i].name);
				if(strncmp(buffer, parameters[i].name, len) == 0) {

					float val = myatof(&buffer[len]);
					printf("setting %s to ", parameters[i].name);
					printFloat_doodledoo(val, 5);
					if(mode==FM_ABSOLUTE)
						*parameters[i].valA = val;
					else
						*parameters[i].valB = val;
					printf("\r\n");
					break;
				}
			}

			print_params(mode);

		}else
			buffer_overrun = false;
		bufpos = 0;
	}
}
