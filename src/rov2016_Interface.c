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
#include <stdlib.h>
/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"
int16_t controller_data[7] = {0};

/* Private variables -------------------------------------------------------------------*/
static uint8_t dataBuffer[8] = {0};
static uint8_t counter_alive = 0;
static uint8_t temperature_check_counter = 1;
static uint8_t rpm_check_counter = 1;

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
 * @brief  Send pitch, roll, yaw and heading to topside system.
 * @param  Pitch, roll, yaw, heading in 0.1 degrees.
 * @retval None
 */
 extern void CAN_transmitDepthTemp(uint16_t depth, uint16_t int_temp, uint16_t manip_temp){
 	dataBuffer[0] = (uint8_t)(depth >> 8u);
 	dataBuffer[1] = (uint8_t)(depth & 0xFF);
 	dataBuffer[2] = (uint8_t)(int_temp >> 8u);
 	dataBuffer[3] = (uint8_t)(int_temp & 0xFF);
 	dataBuffer[4] = (uint8_t)(manip_temp >> 8u);
 	dataBuffer[5] = (uint8_t)(manip_temp & 0xFF);

 	CAN_transmitBuffer(SENSOR_DEPTH_TEMP, dataBuffer, 6, CAN_ID_STD);
}

/**
 * @brief  	Sets the duty cycle of the specified VESC BLDC controller.
 * @param  	esc_id:	Can be a value of ESC_ID_x where x can be 1-12.
 * 			duty:	Duty cycle, can be a value between DUTY_CYCLE_MIN and
 * 					DUTY_CYCLE_MAX.
 * @retval 	None
 */
extern void VESC_setDutyCycle(uint8_t esc_id, float duty){
	/* Regn ut ID
	 * skaler float x 10 000
	 * legg float inn i 4 byte array
	 * send med metoden can_transmitbuffer.
	 */

	/* Check parameters. */
	if (duty < VESC_DUTY_CYCLE_MIN) duty = VESC_DUTY_CYCLE_MIN;
	if (duty > VESC_DUTY_CYCLE_MAX) duty = VESC_DUTY_CYCLE_MAX;
	if(esc_id > 255) return;

	uint32_t id = (uint32_t)(CAN_PACKET_SET_DUTY << 8) | esc_id;

	int32_t temp_duty = (int32_t)(duty * 100000.0f);

	uint8_t buffer[4];
	buffer[0] = temp_duty >> 24;	// MSB(Most significant byte).
	buffer[1] = temp_duty >> 16;
	buffer[2] = temp_duty >> 8;
	buffer[3] = temp_duty;			// LSB(Least significant byte).

	CAN_transmitBuffer(id, buffer, 4, CAN_ID_TYPE_EXT);
}

/**
 * @brief  	Calculates tilt compensated heading.
 * 			Button/axes mapping:
 * 				0. Left trigger:			Depth -
 * 				1. Right trigger:			Depth +
 * 				2. Left stick L/R:			Sway
 * 				3. Left stick fwd/back:		Surge
 * 				4. Right stick L/R:			Roll
 * 				5. Right stick fwd/back:	Pitch
 * 				6. Bumpers L/R				Yaw
 * @param  	Pointer to controller CAN-messages.
 * @retval 	int16_t array containing controller data.
 */
extern int16_t* Interface_readController(void){
	/* Read messages from CAN receive buffer */
	uint8_t* controller_package1 = CAN_getMessagePointer(fmi_topside_xbox_ctrl);
	uint8_t* controller_package2 = CAN_getMessagePointer(fmi_topside_xbox_axes);



	/* Left trigger */
	controller_data[0] = (int16_t)( (int16_t)controller_package1[5] << 8 ) | controller_package1[4];
	controller_data[0] += 1000; // range-> 0-2000

	/* Right trigger */
	controller_data[1] = (int16_t)( (int16_t)controller_package2[3] << 8) | controller_package2[2];
	controller_data[1] += 1000; // range-> 0-2000

	/* Left stick left/right*/
	controller_data[2] = (int16_t)( (int16_t)controller_package1[1] << 8) | controller_package1[0];

	/* Left stick fwd/back */
	controller_data[3] = (int16_t)( (int16_t)controller_package1[3] << 8) | controller_package1[2];

	/* Right stick left/right */
	controller_data[4] = (int16_t)( (int16_t)controller_package1[7] << 8) | controller_package1[6];

	/* Right stick fwd/back */
	controller_data[5] = (int16_t)( (int16_t)controller_package2[1] << 8) | controller_package2[0];

	/* Bumpers */
	if (controller_package2[4] & 0x10){
		/* Left bumper pressed -> counterclockwise rotation. */
		controller_data[6] = -1000;
	} else if (controller_package2[4] & 0x20){
		/* Right bumper pressed -> clockwise rotation. */
		controller_data[6] = 1000;
	} else {
		/* No bumpers pressed */
		controller_data[6] = 0;
	}

	return controller_data;
}

/**
 * @brief  	Sends thrust to motorcontrollers.
 * 			Button/axes mapping:
 * 				0. Left trigger:			Depth -
 * 				1. Right trigger:			Depth +
 * 				2. Left stick L/R:			Sway
 * 				3. Left stick fwd/back:		Surge
 * 				4. Right stick L/R:			Roll
 * 				5. Right stick fwd/back:	Pitch
 * 				6. Bumpers L/R				Yaw
 * @param  	Pointer to controller CAN-messages.
 * @retval 	int16_t array containing controller data.
 */
extern void Interface_transmitManualThrust(void){
	float th1=0.0f, th2=0.0f, th3=0.0f, th4=0.0f, th5=0.0f, th6=0.0f, th7=0.0f, th8=0.0f;


	/* Thruster 1-4 (up-/downwards thrust) **********************************************************/
	float downthrust = (float)controller_data[0]/2000.0f;
	th1 -= downthrust;
	th2 -= downthrust;
	th3 -= downthrust;
	th4 -= downthrust;

	float upthrust = (float)controller_data[1]/2000.0f;
	th1 += upthrust;
	th2 += upthrust;
	th3 += upthrust;
	th4 += upthrust;

	float rollthrust = (float)controller_data[4]/1000.0f;
	th1 -= rollthrust;
	th2 -= rollthrust;
	th3 += rollthrust;
	th4 += rollthrust;

	float pitchthrust = (float)controller_data[5]/1000.0f;
	th1 += pitchthrust;
	th2 -= pitchthrust;
	th3 += pitchthrust;
	th4 -= pitchthrust;

	/* Thruster 5-8 (sideways thrust) ***************************************************************/
	float swaythrust = (float)controller_data[2]/1000.0f;
	th5 -= swaythrust;
	th6 += swaythrust;
	th7 -= swaythrust;
	th8 += swaythrust;

	float surgethrust = (float)controller_data[3]/1000.0f;
	th5 -= surgethrust;
	th6 -= surgethrust;
	th7 -= surgethrust;
	th8 -= surgethrust;

	float yawthrust = (float)controller_data[6]/1000.0f;
	th5 -= yawthrust;
	th6 -= yawthrust;
	th7 += yawthrust;
	th8 += yawthrust;

	/* Normalize if multiple axes active. **********************************************************/

	/* Search for largest thrust value. */
	float maxthrust_up_down = 0.95f;
	float maxthrust_sideways = 0.95f;

	if(maxthrust_up_down > abs(th1)) maxthrust_up_down = abs(th1);
	if(maxthrust_up_down > abs(th2)) maxthrust_up_down = abs(th2);
	if(maxthrust_up_down > abs(th3)) maxthrust_up_down = abs(th3);
	if(maxthrust_up_down > abs(th4)) maxthrust_up_down = abs(th4);

	if(maxthrust_sideways > abs(th5)) maxthrust_sideways = abs(th5);
	if(maxthrust_sideways > abs(th6)) maxthrust_sideways = abs(th6);
	if(maxthrust_sideways > abs(th7)) maxthrust_sideways = abs(th7);
	if(maxthrust_sideways > abs(th8)) maxthrust_sideways = abs(th8);

	if(maxthrust_up_down > 1.0f){
		th1 = th1/(maxthrust_up_down*1.06f);
		th2 = th2/(maxthrust_up_down*1.06f);
		th3 = th3/(maxthrust_up_down*1.06f);
		th4 = th4/(maxthrust_up_down*1.06f);
	} else if (maxthrust_up_down > 0.95f){
		th1 /= 1.06f;
		th2 /= 1.06f;
		th3 /= 1.06f;
		th4 /= 1.06f;
	}

	if(maxthrust_sideways > 1.0f){
		th5 = th5/(maxthrust_sideways*1.06);
		th6 = th6/(maxthrust_sideways*1.06);
		th7 = th7/(maxthrust_sideways*1.06);
		th8 = th8/(maxthrust_sideways*1.06);
	} else if (maxthrust_sideways > 0.95f){
		th5 /= 1.06f;
		th6 /= 1.06f;
		th7 /= 1.06f;
		th8 /= 1.06f;
	}

	/* Send thrust to ESC's. */
	VESC_setDutyCycle(1, th1);
	VESC_setDutyCycle(2, th2);
	VESC_setDutyCycle(3, th3);
	VESC_setDutyCycle(4, th4);

	VESC_setDutyCycle(5, th5);
	VESC_setDutyCycle(6, th6);
	VESC_setDutyCycle(7, th7);
	VESC_setDutyCycle(8, th8);
}

/**
 * @brief  	Utility function for controlling one thruster with
 * 			Xbox-triggers.
 * @param 	Thruster/esc identifier.
 * @retval 	None
 */
extern void Interface_transmitOneThruster(uint8_t thrusterId){
	float th1 = 0;

	th1 -= (float)controller_data[0]/2110.0;

	th1 += (float)controller_data[1]/2110.0;

	VESC_setDutyCycle(thrusterId, th1);
}

/**
 * @brief  	Request data from a VESC, received data gets stored
 * 			in the CAN receive buffer.
 * @param  	esc_id:				ESC identifier.
 * @param	package_request: 	CAN package to be requested from the
 * 								VESC.
 * @retval 	None
 */
extern void Interface_VESC_requestData(uint8_t esc_id, CAN_PACKET_ID package_request){
	CAN_transmitByte_EID((uint32_t)((package_request << 8)|esc_id), 0);
}

/**
 * @brief  	Retrieve an int16_t from a VESC message.
 * @param	Filter match index for the received message.
 * @retval 	None
 */
extern int32_t Interface_VESC_getInt32(uint8_t filter_match_index){
	uint8_t d0, d1, d2, d3;
	d0 = CAN_getByteFromMessage(filter_match_index, 0);
	d1 = CAN_getByteFromMessage(filter_match_index, 1);
	d2 = CAN_getByteFromMessage(filter_match_index, 2);
	d3 = CAN_getByteFromMessage(filter_match_index, 3);

	int32_t result = (int32_t)	(((uint32_t)d0 << 24)
								|((uint32_t)d1 << 16)
								|((uint16_t)d2 << 8)
								| d3);
	return result/1000;
}

/**
 * @brief  	Request temperature data from VESC's incrementally 
 			one at a time. This function should be called every 
 			~1 second. Data is read by the topside system.
 * @param	None
 * @retval 	None
 */
extern void Interface_VESC_requestTemperature(void){
	Interface_VESC_requestData(temperature_check_counter, CAN_PACKET_GET_MOSFET_TEMP);

	/* Increment counter. */
	if(temperature_check_counter < NUMBER_OF_VESCS){
		temperature_check_counter++;
	} else {
		temperature_check_counter = 1;
	}
}

/**
 * @brief  	Request rpm from VESC's incrementally 
 			one at a time. This function should be called every 
 			~10 milli second. Data is read by the topside system.
 * @param	None
 * @retval 	None
 */
 extern void Interface_VESC_requestRPM(void){
 	Interface_VESC_requestData(rpm_check_counter, CAN_PACKET_GET_RPM);

 	/* Increment counter. */
	if(rpm_check_counter < NUMBER_OF_VESCS){
		rpm_check_counter++;
	} else {
		rpm_check_counter = 1;
	}
 }
