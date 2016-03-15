/**
 ******************************************************************************
 * @file    SysTick_metoder.c
 * @author  Sivert Sliper and Stian Sørensen
 * @version V1.0
 * @date    08-February-2016
 * @brief   This file contains all the functions prototypes for the SysTick
 *          timer.
 *
 ******************************************************************************
 */

/* Include---- ------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "core_cm4.h"
#include "stm32f30x_dma.h"
#include "rov2016_Accelerometer.h"
#include "rov2016_canbus.h"
#include "rov2016_ADC.h"
#include "rov2016_UART.h"
#include "rov2016_Gyroscope.h"
#include "rov2016_SPI.h"
#include "MadgwickAHRS.h"
//#include "MahonyAHRS.h"
#include "stm32f3_discovery_lsm303dlhc.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Private variables -------------------------------------------------------------------*/
static float gx=0.0f, gy=0.0f, gz=0.0f, ax=0.0f, ay=0.0f, az=0.0f, mx=0.0f, my=0.0f, mz=0.0f;
static uint8_t kjor = 0, timestamp=0;

/* Private function declarations ---------------------------------------------------------------*/

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Configures the SysTick timer for 100 Hz interrupt frequency.
 * @param  None
 * @retval None
 */
void SysTick_init(void) {
	NVIC_SetPriority(SysTick_IRQn, 1);
	SysTick->CTRL = 0; /* Disable SysTick */
	SysTick->LOAD = 72000000/100;  // 10 msek avbruddsintervall.
	SysTick->VAL = 0;
	SysTick->CTRL = (SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk
			| SysTick_CTRL_CLKSOURCE_Msk);
} // end Systick_init()


/**
 * @brief  Configures the CAN-Controller peripheral for 500 kbps communication.
 * 		   Also configures Rx filters according to ID's specified in "can_metoder.h"
 * @param  None
 * @retval None
 */

uint8_t teller = 0;
uint16_t val = 0;
uint32_t valVoltage = 0;
uint8_t timeStamp = 0;
uint8_t olav = 1;

void SysTick_Handler(void){
	teller++;

	/* Check for new message on CAN and update LEDs */
	if(CAN_getRxMessages()>0){
		GPIOE->ODR ^= (1u << CAN_getByteFromMessage(2,0)) << 8;
	} // end if

	/* Check for USART messages, start if 'k' */
	if (USART_getNewBytes()>0){
		uint8_t melding = USART_getRxMessage();
		if (melding == 'k') {
			kjor = 1;
			USART_transmit(USART2, 0x02); // STX
		}
		if (melding == 's'){
			kjor = 0;
			USART_transmit(USART2, 0x03); //ETX
		}
	}

	accelerometer_updateValue();
	magnetometer_updateValue();
	gyroscope_updateValue();


	/* 10 Hz loop. */
	if((teller>100) && kjor){
		GPIOE->ODR ^= SYSTICK_LED << 8;
		teller = 0;
//		MS5803_updateDigital(MS5803_CONVERT_PRESSURE);
		CAN_transmitByte(POWR_COOLING_FAN_SWITCH,olav);
		printf("Sent %d to address %d", olav, POWR_COOLING_FAN_SWITCH);
		if(olav) olav = 0;
		else olav = 1;

//		CAN_transmitQuaternions((int16_t)(q0*1000), (int16_t)(q1*1000), (int16_t)(q2*1000), (int16_t)(q3*1000));

//		USART_matlab_visualizer_transmit((int16_t)(q0*1000), (int16_t)(q1*1000), (int16_t)(q2*1000), (int16_t)(q3*1000));
//		USART_matlab_visualizer_transmit((int16_t)(ax), (int16_t)(ay), (int16_t)(az), (int16_t)(gz));
//		USART_matlab_visualizer_transmit((int16_t)MS5803_getPressure(), 0,0,0);

	if(kjor){
		ax = (float)accelerometer_getRawData(ACCELEROMETER_X_AXIS);
		ay = (float)accelerometer_getRawData(ACCELEROMETER_Y_AXIS);
		az = (float)accelerometer_getRawData(ACCELEROMETER_Z_AXIS);

		mx = ((float)magnetometer_getRawData(MAGNETOMETER_X_AXIS));
		my = ((float)magnetometer_getRawData(MAGNETOMETER_Y_AXIS));
		mz = ((float)magnetometer_getRawData(MAGNETOMETER_Z_AXIS));

		/* Compensate for sensitivity difference between magnetometer axes. */
		mx /= 9.8f;
		my /= 9.8f;
		mz /= 11.0f;

		gx = gyroscope_getRPS(GYROSCOPE_X_AXIS);
		gy = gyroscope_getRPS(GYROSCOPE_Y_AXIS);
		gz = gyroscope_getRPS(GYROSCOPE_Z_AXIS);

		/* Update AHRS (Attitude Heading Reference System. */
//		MadgwickAHRSupdate(gx,gy,gz,ax,ay,az,mx,my,mz);

		/* From testing. */
		MadgwickAHRSupdate(gy, -gx, gz, -ax, -ay, az, -mx, -my, mz);
//		MahonyAHRSupdate(gy, -gx, gz, -ax, -ay, az, -mx, -my, mz);

		/* From MCD application team. */
//		MadgwickAHRSupdate(-gy, gx, gz, ax, ay, az, mx, my, mz);
		//myFusion(-gy, gx, gz, ax, ay, az, mx, my, mz);

//		MadgwickAHRSupdateIMU(gy, -gx, gz, -ax, -ay, az);
		//MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
	} // end if



	if((teller>10) && kjor){
		GPIOE->ODR ^= SYSTICK_LED << 8;
		teller = 0;

		/* Transmit quaternions over CAN-bus. */
		//		CAN_transmitQuaternions((int16_t)(q0*1000), (int16_t)(q1*1000), (int16_t)(q2*1000), (int16_t)(q3*1000));

		/* Transmit q0...q3 to matlab. */
//		USART_matlab_visualizer_transmit((int16_t)(q0*10000), (int16_t)(q1*10000), (int16_t)(q2*10000), (int16_t)(q3*10000));

		/* Transmit a_x, a_y, a_z to matlab. */
//		USART_matlab_visualizer_transmit((int16_t)(ax*10), (int16_t)(ay*10), (int16_t)(az*10), (int16_t)(0u));

		/* Transmit g_x, g_y, g_z x10000 to matlab. */
//		USART_matlab_visualizer_transmit((int16_t)(gx*10000), (int16_t)(gy*10000), (int16_t)(gz*10000), (int16_t)(0u));

		/* Transmit m_x, m_y, m_z x100 to matlab.*/
//		USART_matlab_visualizer_transmit((int16_t)(mx*100), (int16_t)(my*100), (int16_t)(mz*100), (int16_t)(0u));

		/* Transmit -2, -1, 0, 1 x10000 to matlab. */
//		USART_matlab_visualizer_transmit((int16_t)(-2*10000), (int16_t)(-1*10000), (int16_t)(0*10000), (int16_t)(1*10000));
	} // end if

} // end Systick_Handler()
