/**
 **************************************************************************************
 * @file    CAN_metoder.c
 * @author  Sivert Sliper, Stian Soerensen
 * @version V1.0
 * @date    3-February-2016
 * @brief   This file contains all the functions for the CAN peripheral.
 **************************************************************************************
 */

/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "rov2016_canbus.h"
#include "rov2016_Interface.h"
#include "stm32f30x_can.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_misc.h"
/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Private variables -------------------------------------------------------------------*/
static uint8_t dataBuffer[8] = {0};
static uint8_t counter_alive = 0;

typedef enum {
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS
} CAN_PACKET_ID;

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Transmit SENSOR_AN_RAW package.
 * @param  None
 * @retval None
 */
extern void CAN_transmit_AN_RAW(void){
	dataBuffer[0] = (uint8_t) (ADC4_getChannel(0) & 0xFF);
	dataBuffer[1] = (uint8_t) (ADC4_getChannel(0) >> 8);
	dataBuffer[2] = (uint8_t) (ADC1_getChannel(1) & 0xFF);
	dataBuffer[3] = (uint8_t) (ADC1_getChannel(1) >> 8);
	dataBuffer[4] = (uint8_t) (ADC1_getChannel(2) & 0xFF);
	dataBuffer[5] = (uint8_t) (ADC1_getChannel(2) >> 8);
	dataBuffer[6] = (uint8_t) (ADC1_getChannel(3) & 0xFF);
	dataBuffer[7] = (uint8_t) (ADC1_getChannel(3) >> 8);

	CAN_transmitBuffer(SENSOR_AN_RAW, dataBuffer, 8, CAN_ID_STD);
}

/**
 * @brief  Transmit acceleration measurements in all 3 axes.
 * @param  Array of dimension 6 containing measurements.
 * @retval None
 */
extern void CAN_transmitAcceleration(uint8_t* acc_array){
	CAN_transmitBuffer(SENSOR_ACCELERATION, acc_array, 6, CAN_ID_STD);
}

/**
 * @brief  Transmit 4 quaternions as 4 int16_t split into 8 bytes.
 * @param  int16_t q0, q1, q2, q3
 * @retval None
 */
extern void CAN_transmitQuaternions(int16_t q0, int16_t q1, int16_t q2, int16_t q3){
	dataBuffer[0] = (uint8_t)(q0 >> 8u);
	dataBuffer[1] = (uint8_t)(q0 & 0xFF);
	dataBuffer[2] = (uint8_t)(q1 >> 8u);
	dataBuffer[3] = (uint8_t)(q1 & 0xFF);
	dataBuffer[4] = (uint8_t)(q2 >> 8u);
	dataBuffer[5] = (uint8_t)(q2 & 0xFF);
	dataBuffer[6] = (uint8_t)(q3 >> 8u);
	dataBuffer[7] = (uint8_t)(q3 & 0xFF);

	CAN_transmitBuffer(SENSOR_AHRS_QUATERNIONS, dataBuffer, 8, CAN_ID_STD);
}

/**
 * @brief  Send alive-message over CAN-bus to topside. Should be called every ~1 second.
 * @param  None
 * @retval None
 */
extern void CAN_transmitAlive(void){
	CAN_transmitByte(SENSOR_ALIVE, ++counter_alive);
	if(counter_alive == 255) counter_alive = 0;
}

/**
 * @brief  Send pitch, roll, yaw and heading to topside system.
 * @param  Pitch, roll, yaw, heading in 0.1 degrees.
 * @retval None
 */
extern void CAN_transmitAHRS(int16_t pitch, int16_t roll, int16_t yaw, uint16_t heading){
	dataBuffer[0] = (uint8_t)(pitch >> 8u);
	dataBuffer[1] = (uint8_t)(pitch & 0xFF);
	dataBuffer[2] = (uint8_t)(roll >> 8u);
	dataBuffer[3] = (uint8_t)(roll & 0xFF);
	dataBuffer[4] = (uint8_t)(yaw >> 8u);
	dataBuffer[5] = (uint8_t)(yaw & 0xFF);
	dataBuffer[6] = (uint8_t)(heading >> 8u);
	dataBuffer[7] = (uint8_t)(heading & 0xFF);

	CAN_transmitBuffer(SENSOR_AHRS, dataBuffer, 8, CAN_ID_STD);
}

/**
 * @brief  	Sets the duty cycle of the specified VESC BLDC controller.
 * @param  	esc_id:	Can be a value of ESC_ID_x where x can be 1-12.
 * 			duty:	Duty cycle, can be a value between DUTY_CYCLE_MIN and
 * 					DUTY_CYCLE_MAX.
 * @retval 	None
 */
void VESC_setDutyCycle(uint8_t esc_id, float duty){
	/* Regn ut ID
	 * skaler float x 10 000
	 * legg float inn i 4 byte array
	 * send med metoden can_transmitbuffer.
	 */

	/* Check parameters. */
	if (duty < VESC_DUTY_CYCLE_MIN) return;
	if (duty > VESC_DUTY_CYCLE_MAX) return;
	if(esc_id > 255) return;

	uint32_t id = (uint32_t)(CAN_PACKET_SET_DUTY << 8) | esc_id;

	uint32_t temp_duty = (uint32_t)(duty * 10000.0f);

	uint8_t buffer[4];
	buffer[0] = temp_duty >> 24;	// MSB(Most significant byte).
	buffer[1] = temp_duty >> 16;
	buffer[2] = temp_duty >> 8;
	buffer[3] = temp_duty;			// LSB(Least significant byte).

	CAN_transmitBuffer(id, buffer, 4, CAN_ID_TYPE_EXT);
}
