#include "global.h"

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include "sysTick.h"
#include "i2c.h"
#include "spi.h"
#include "delay.h"
#include "HMC5883L_driver.h"
#include "MPU6000.h"
#include "RX.h"
#include "servo.h"
//#include "Kinematics_DCM.h"
#include "Kinematics_ARG.h"
#include "flightControl.h"
#include "eeprom.h"
#include "terminal.h"


typedef struct Sensor3Struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Sensor3;
static Sensor3 mag, acc, gyr;
static Sensor3 mag0, acc0, gyr0;
float oneG = 263;


GPIO_InitTypeDef  GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;








void init_usart()
{

	// --	RCC	---------

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* Enable UART1 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


	// --	GPIO	---------

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	/* Configure USART Tx as alternate function push-pull */
	// Tx = B6
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Connect B6 to USART1_Tx */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1);

	/* Configure USART Rx as alternate function push-pull */
	// Rx = B7
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function push-pull */
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);


	// --	USART     ---------

	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;//38400;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);
}





void init_essentials()
{
	sysTick_init();

	// LEDs C0, C1, C2
	/* GPIOD Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	/* Configure in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_SetBits(GPIOC, GPIO_Pin_0);


	// servo output
	servo_init();
    servo_setU(0, 0);
	servo_setU(1, 0);
	servo_setU(2, 0);
	servo_setU(3, 0);

}

void init_sensorFusion()
{
	initializeKinematics(1.0f, 0.0f);
}


void init()
{
	FLASH_Unlock();
	EE_Init();
	FLASH_Lock();

	// uart
	init_usart();

	i2c_init();

	spi_init();

	init_Rx();


	//delay_ms(100); // wait for the board to settle from power up before starting to talk to peripherals
	if(!hmc5883l_detect())
		printf("magneto not found\n");
	else {
		printf("magneto found\n");
		hmc5883l_setMode(HMC5883L_MODE_CONTINUOUS);
	}

	if(!mpu6k_init())
		printf("mpu6k not found\n");
	else
		printf("mpu6k found\n");



	init_sensorFusion();
	init_flightController();
}





void readSensors()
{
	uint8_t buff[14];

	if(hmc5883l_ReadRegs(HMC5883L_DATAX_H, 6, buff) ) {
		mag.x = (uint16_t)buff[0] << 8 | buff[1];
		mag.y = (uint16_t)buff[2] << 8 | buff[3];
		mag.z = (uint16_t)buff[4] << 8 | buff[5];
	}

	mpu6k_readRegisters(MPU6K_REG_ACCEL_XOUT_H, 14, buff);
	acc.x = -(((int16_t)buff[0] << 8) | buff[1]);
	acc.y = -(((int16_t)buff[2] << 8) | buff[3]);
	acc.z = -(((int16_t)buff[4] << 8) | buff[5]);
	gyr.x = buff[8] << 8 | buff[9];
	gyr.y = buff[10] << 8 | buff[11];
	gyr.z = buff[12] << 8 | buff[13];
//	temp = buff[6] << 8 | buff[7];
}

void cancelSensors()
{
	// todo: wait for a calm period (user set down the quad)

	int32_t a0x = 0, a0y = 0, a0z = 0;
	int32_t g0x = 0, g0y = 0, g0z = 0;
	for(uint8_t i = 0; i < 255; ++i) {
		readSensors();
		a0x += acc.x;
		a0y += acc.y;
		a0z += acc.z;
		g0x += gyr.x;
		g0y += gyr.y;
		g0z += gyr.z;
		delay_ms(5);
		if(i % 10 == 0)
			LedsToggle(LEDR);
	}

	LedsOff(LEDR);
	acc0.x = a0x / 255;
	acc0.y = a0y / 255;
	acc0.z = a0z / 255;
	gyr0.x = g0x / 255;
	gyr0.y = g0y / 255;
	gyr0.z = g0z / 255;
}




#define radians(A) ((((float)(A))/180.0f)*M_PI)
#define degrees(A) ((((float)(A))/M_PI)*180.0f)

float rollRate;
float pitchRate;
float yawRate;
float longitudinalAccel;
float lateralAccel;
float verticalAccel;
// test
float gyro_drift_pitch=0, gyro_drift_roll=0, gyro_drift_yaw=0;

void update_sensorFusion(float dt)
{
	//	  float oneG = fabs(acc0.z);
	  float oneG = sqrt(acc0.x*acc0.x+acc0.y*acc0.y+acc0.z*acc0.z);

	  rollRate  = radians( (float)(gyr.x - gyr0.x) / 32.8f ) - gyro_drift_roll;
	  pitchRate = radians( (float)(gyr.y - gyr0.y) / 32.8f ) - gyro_drift_pitch;
	  yawRate   = radians( (float)(gyr.z - gyr0.z) / 32.8f );
	  longitudinalAccel = (float)(acc.x - acc0.x) / oneG;
	  lateralAccel 	  = (float)(acc.y - acc0.y) / oneG;
	  verticalAccel	  = (float)(acc.z) / oneG;
	  //oneG = 1.0;
	  float magX = 0;
	  float magY = 0;


	  // test: high pass filter for the gyro
	  float macc = sqrt(acc.x*acc.x+acc.y*acc.y+acc.z*acc.z) / oneG;
	  if(macc > 0.8 && macc < 1.2) // reasonably quite
	  {
		  const float f = 0.9999f, i_f = 1.0f - f;
		  gyro_drift_pitch = gyro_drift_pitch * f + pitchRate * i_f;
		  gyro_drift_roll  = gyro_drift_roll * f + rollRate * i_f;
	  }


	  calculateKinematics( rollRate,             pitchRate,       yawRate,
	                       longitudinalAccel,    lateralAccel,    verticalAccel,
	                       /*oneG*/1.0f,         magX,            magY,
	  				       dt);
}


void wait_for_settle_down()
{
	int32_t a0x = 0, a0y = 0, a0z = 0, i=0;
	int8_t activity = 10;

	while(activity) {
		readSensors();

		if(fabs(a0x - acc.x) < 100 &&
		   fabs(a0y - acc.y) < 100 &&
		   fabs(a0z - acc.z) < 100 )
			activity--;
		else
			activity = 10;

		a0x = acc.x;
		a0y = acc.y;
		a0z = acc.z;
		delay_ms(100);
		if(i % 2 == 0)
			LedsToggle(LEDR);
		i++;
	}

	LedsOff(LEDR);
}

void wait_for_signal()
{
	LedsOn(LEDR);
	while(RXnumChannelsSampled < RX_NUM_CHANNELS) {
		delay_ms(100);
	}
	LedsOff(LEDR);
}

void wait_for_throttle_cut()
{
	int16_t i = 0, thrust;
	do
	{
		thrust = (RXchan[RXCHAN_THUST] - 2815)/20;
		delay_ms(100);
		if(i % 10 == 0)
			LedsToggle(LEDR);
		i++;
	}while(thrust > 5);
	LedsOff(LEDR);
}




int8_t lastFlightMode = 0, absolute_override = 0;
int8_t printFloat_doodledoo(float number, uint8_t numDecimals);

int main(void)
{
const uint loop_period = 1000000 / 500; // in us (400 Hz update)


	init_essentials();
	GPIO_SetBits(GPIOC, GPIO_Pin_1);
	init();
	GPIO_SetBits(GPIOC, GPIO_Pin_2);



	LedsOff(LEDR|LEDG|LEDB);
	wait_for_settle_down();
	cancelSensors();
	wait_for_signal();
	wait_for_throttle_cut();


	//	control loop
	//

	int16_t count = 0;
	uint32_t lastTime = getTime_us();
	int16_t rx_thrust, rx_pitch, rx_roll, rx_yaw, rx_aux, rx_gear;
	GPIO_ResetBits(GPIOC, GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2);
	while (1)
	{
		// wait for the next update
		uint32_t t_e;
		while((t_e=timeElapsed_us(lastTime)) < loop_period);

		float dt = ((float)(t_e)) * 1e-6f;
		lastTime = getTime_us();

		if(RXnumChannelsSampled < RX_NUM_CHANNELS) {
			rx_thrust = 0;
			rx_pitch = 0;
			rx_roll = 0;
			rx_yaw = 0;
			rx_aux = 0;
			rx_gear = 0;
		}else{
			rx_thrust = (RXchan[RXCHAN_THUST] - 2815)/20;
			rx_pitch  = (RXchan[RXCHAN_PITCH] - 2815)/10 - 100;
			rx_roll   = (RXchan[RXCHAN_ROLL] - 2815)/10 - 100;
			rx_yaw    = (RXchan[RXCHAN_YAW]  - 2815)/10 - 100;
			rx_aux    = (RXchan[RXCHAN_AUX]  - 2815)/10 - 100;
			rx_gear   = (RXchan[RXCHAN_GEAR] - 2815)/10 - 100;
		}


		uint8_t flightMode = rx_gear > 50 ? 1:0;
		LedsOn(LEDG);
		readSensors();
		update_sensorFusion(dt);


		// auto zero at rest pos
		if(flightMode==FM_RELATIVE) {
			if(!absolute_override) {
				if(abs(rx_pitch) < 10 && abs(rx_roll) < 10)
					absolute_override = 1;
			}else{
				if(abs(rx_pitch) > 20 || abs(rx_roll) > 20)
					absolute_override = 0;
			}
		}

		//if(absolute_override)
		//	flightMode = FM_ABSOLUTE;

		if(lastFlightMode != flightMode) {// toggle
			if(flightMode==FM_ABSOLUTE)
				reset_absoluteFlightController(kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], dt);
			else
				reset_relativeFlightController(pitchRate, rollRate, yawRate, dt);
		}


		if(flightMode==FM_ABSOLUTE) {
			update_absoluteFlightController(rx_thrust, rx_pitch, rx_roll, rx_yaw,
											rx_aux,
											kinematicsAngle[XAXIS], kinematicsAngle[YAXIS], kinematicsAngle[ZAXIS], dt);
		}else{
			update_relativeFlightController(rx_thrust, rx_pitch, rx_roll, rx_yaw,
											rx_aux,
											pitchRate, rollRate, yawRate, dt);
		}
		lastFlightMode = flightMode;

		LedsOff(LEDG);

		if(rx_thrust < 10)
		{
			terminal(rx_gear > 50 ? 1:0);
		}

#if 0
		if(count%128==0) {
			printf("gyr: %d, %d\r\n", gyr.x-gyr0.x, gyr.y-gyr0.y);
			printFloat_doodledoo(gyro_drift_pitch, 6);
			printf("  ");
			printFloat_doodledoo(gyro_drift_roll, 6);
			printf("\r\n");
		}
#endif
    count++;
  }
}



#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
