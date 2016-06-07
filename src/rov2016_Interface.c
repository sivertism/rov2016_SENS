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
#include <math.h>
/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"
int16_t controller_data[7] = {0};

/* Private variables -------------------------------------------------------------------*/
static uint8_t dataBuffer[8] = {0};
static int16_t reg_param[7] = {0};
static uint8_t counter_alive = 0;
static uint8_t temperature_check_counter = 1;
static uint8_t rpm_check_counter = 1;
static uint8_t current_check_counter = 1;
static float th1=0.0f, th2=0.0f, th3=0.0f, th4=0.0f, th5=0.0f, th6=0.0f, th7=0.0f, th8=0.0f;
static float th1_safe=1.0f, th2_safe=0.0f, th3_safe=0.0f, th4_safe=0.0f;
static float th5_safe=0.0f, th6_safe=0.0f, th7_safe=0.0f, th8_safe=0.0f;
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
 * @brief  Send depth and temperature at pressure sensor to topside system.
 * @param  depth in mm and temp in 0.01 celsius.
 * @retval None
 */
extern void CAN_transmitDepthTemp(int32_t depth, uint16_t pressure_temp){
	dataBuffer[0] = (uint8_t)(depth >> 24u);
	dataBuffer[1] = (uint8_t)(depth >> 16u);
	dataBuffer[2] = (uint8_t)(depth >> 8u);
	dataBuffer[3] = (uint8_t)(depth & 0xFF);
	dataBuffer[4] = (uint8_t)(pressure_temp >> 8u);
	dataBuffer[5] = (uint8_t)(pressure_temp & 0xFF);

	CAN_transmitBuffer(SENSOR_DEPTH_TEMP, dataBuffer, 6, CAN_ID_STD);
}

/**
 * @brief  Send internal temperature, manipulator temperature and DC/DC temperature to topside system.
 * @param  int_temp, manip_temp, DCDC_temp in millicelsius.
 * @retval None
 */
extern void CAN_transmitTemp(uint16_t int_temp, uint16_t manip_temp, uint16_t DCDC_temp){
	dataBuffer[0] = (uint8_t)(int_temp >> 8u);
	dataBuffer[1] = (uint8_t)(int_temp & 0xFF);
	dataBuffer[2] = (uint8_t)(manip_temp >> 8u);
	dataBuffer[3] = (uint8_t)(manip_temp & 0xFF);
	dataBuffer[4] = (uint8_t)(DCDC_temp >> 8u);
	dataBuffer[5] = (uint8_t)(DCDC_temp & 0xFF);

	CAN_transmitBuffer(SENSOR_TEMP, dataBuffer, 6, CAN_ID_STD);
}

/**
 * @brief  Send three axis magnetometer sensor data.
 * @param  mx, my, mz in milligauss.
 * @retval None
 */
extern void CAN_transmitMag(uint16_t mx, uint16_t my, uint16_t mz){
	dataBuffer[0] = (uint8_t)(mx >> 8u);
	dataBuffer[1] = (uint8_t)(mx & 0xFF);
	dataBuffer[2] = (uint8_t)(my >> 8u);
	dataBuffer[3] = (uint8_t)(my & 0xFF);
	dataBuffer[4] = (uint8_t)(mz >> 8u);
	dataBuffer[5] = (uint8_t)(mz & 0xFF);

	CAN_transmitBuffer(SENSOR_MAGNETIC_FIELD, dataBuffer, 6, CAN_ID_STD);
}

/**
 * @brief  Send tree axis gyroscope sensor data.
 * @param  gx, gy, gz in degrees/sec.
 * @retval None
 */
extern void CAN_transmitGyro(uint16_t gx, uint16_t gy, uint16_t gz){
	dataBuffer[0] = (uint8_t)(gx >> 8u);
	dataBuffer[1] = (uint8_t)(gx & 0xFF);
	dataBuffer[2] = (uint8_t)(gy >> 8u);
	dataBuffer[3] = (uint8_t)(gy & 0xFF);
	dataBuffer[4] = (uint8_t)(gz >> 8u);
	dataBuffer[5] = (uint8_t)(gz & 0xFF);

	CAN_transmitBuffer(SENSOR_ANGULAR_VELOCITY, dataBuffer, 6, CAN_ID_STD);
}



extern void Interface_SendSetPoint(int16_t depth, int16_t roll, int16_t pitch){
	dataBuffer[0] = (uint8_t)(depth >> 8u);
	dataBuffer[1] = (uint8_t)(depth & 0xFF);
	dataBuffer[2] = (uint8_t)(roll >> 8u);
	dataBuffer[3] = (uint8_t)(roll & 0xFF);
	dataBuffer[4] = (uint8_t)(pitch >> 8u);
	dataBuffer[5] = (uint8_t)(pitch & 0xFF);
	CAN_transmitBuffer(SENSOR_REG_SETPOINT, dataBuffer, 6, CAN_ID_TYPE_STD);
}

/**
 * @brief	Read regulator parameters over CAN from topside
 * 		0. P trans
 * 		1. I trans
 * 		2. D trans
 * 		3. P rot
 * 		4. I rot
 * 		5. D rot
 * 		6. Regulator: 0 = off, else on
 *
 * 	@retval int16_t array containing regulator data
 */
extern void Interface_readRegparam(void){

	/* Read topside regulator parameters*/
	uint8_t* param_package1 = CAN_getMessagePointer(fmi_topside_reg_param1);
	uint8_t* param_package2 = CAN_getMessagePointer(fmi_topside_reg_param2);

	/*regulator parameters translation:  P [0] I [1] D [2]}*/
	reg_param[0] = (int16_t) (( (uint16_t)param_package1[1] << 8) | param_package1[0]);
	reg_param[1] = (int16_t) (( (uint16_t)param_package1[3] << 8) | param_package1[2]);
	reg_param[2] = (int16_t) (( (uint16_t)param_package1[5] << 8) | param_package1[4]);

	/*regulator parameters rotation:  P [3] I [4] D [5]}*/
	reg_param[3] = (int16_t) (( (uint16_t)param_package2[1] << 8) | param_package2[0]);
	reg_param[4] = (int16_t) (( (uint16_t)param_package2[3] << 8) | param_package2[2]);
	reg_param[5] = (int16_t) (( (uint16_t)param_package2[5] << 8) | param_package2[4]);

	//if(reg_param[0] == 0) return;
	Kp_t = (int32_t)reg_param[0];
	Ti_t = (int32_t)reg_param[1];
	Td_t = (int32_t)reg_param[2];
	Kp_r = (int32_t)reg_param[3];
	Ti_r = (int32_t)reg_param[4];
	Td_r = (int32_t)reg_param[5];
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

	int32_t temp_duty = (int32_t)((duty * 100000.0f));

	uint8_t buffer[4];
	buffer[0] = temp_duty >> 24;	// MSB(Most significant byte).
	buffer[1] = temp_duty >> 16;
	buffer[2] = temp_duty >> 8;
	buffer[3] = temp_duty;			// LSB(Least significant byte).

	CAN_transmitBuffer(id, buffer, 4, CAN_ID_TYPE_EXT);
}

/**
 * @brief  	Reads XBOX-controller data from topside CAN-bus message.
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
	th1=0.0f;
	th2=0.0f;
	th3=0.0f;
	th4=0.0f;
	th5=0.0f;
	th6=0.0f;
	th7=0.0f;
	th8=0.0f;

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
	th3 -= pitchthrust;
	th4 += pitchthrust;

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

	float yawthrust = ((float)controller_data[6]/1000.0f)/2;
	th5 -= yawthrust;
	th6 -= yawthrust;
	th7 += yawthrust;
	th8 += yawthrust;

	/* Limit thrust */
	if(th1>1.0f) th1 = 1.0f;
	if(th1<-1.0f) th1 = -1.0f;

	if(th2>1.0f) th2 = 1.0f;
	if(th2<-1.0f) th2 = -1.0f;

	if(th3>1.0f) th3 = 1.0f;
	if(th3<-1.0f) th3 = -1.0f;

	if(th4>1.0f) th4 = 1.0f;
	if(th4<-1.0f) th4 = -1.0f;

	if(th5>1.0f) th5 = 1.0f;
	if(th5<-1.0f) th5 = -1.0f;

	if(th6>1.0f) th6 = 1.0f;
	if(th6<-1.0f) th6 = -1.0f;

	if(th7>1.0f) th7 = 1.0f;
	if(th7<-1.0f) th7 = -1.0f;

	if(th8>1.0f) th8 = 1.0f;
	if(th8<-1.0f) th8 = -1.0f;

	/* Scale down thrust. */
	th1 = th1 * 0.33f; // 1/3
	th2 = th2 * 0.33f; // 1/3
	th3 = th3 * 0.33f; // 1/3
	th4 = th4 * 0.33f; // 1/3
	th5 = th5 * 0.33f; // 1/3
	th6 = th6 * 0.33f; // 1/3
	th7 = th7 * 0.33f; // 1/3
	th8 = th8 * 0.33f; // 1/3

	/* Save safe duty cycle. */
	th1_safe = th1;
	th2_safe = th2;
	th3_safe = th3;
	th4_safe = th4;
	th5_safe = th5;
	th6_safe = th6;
	th7_safe = th7;
	th8_safe = th8;

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
 * @brief  	Sends thruster duty cycle on the CAN-bus.
 * @param 	None
 * @retval 	None
 */
extern void Interface_transmitThrustToMatlab(void){
	/*** Send duty cycle to Matlab ***/
	int8_t duty_array[8] = {(int8_t)(th1*100.0f),
							(int8_t)(th2*100.0f),
							(int8_t)(th3*100.0f),
							(int8_t)(th4*100.0f),
							(int8_t)(th5*100.0f),
							(int8_t)(th6*100.0f),
							(int8_t)(th7*100.0f),
							(int8_t)(th8*100.0f)};

	CAN_transmitBuffer(SENSOR_THRUSTER_DUTY_MAN, duty_array, 8, 0);
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
extern void Interface_VESC_request_temp_volt(void){
	Interface_VESC_requestData(temperature_check_counter, CAN_PACKET_GET_TEMP_VOLT);
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

/**
 * @brief  Request current from VESC's incrementally
  			one at a time. This function should be called every
  			~10 milli second. Data is read by the topside system.
 * @param	None
 * @retval None
 */
extern void Interface_VESC_requestCurrent(void){
	Interface_VESC_requestData(current_check_counter, CAN_PACKET_GET_CURRENT);

	if(current_check_counter < NUMBER_OF_VESCS){
		current_check_counter++;
	} else {
		current_check_counter = 1;
	}
}

/**
 * @brief  	Retrieve a number proportional to the total amount of thrust.
 * @param	None
 * @retval 	int16_t proportional to the total amount of thrust, varies
 * 			between -100 and 100.
 */
extern int16_t Interface_getTotalDuty(void){
	uint16_t abs_thrust = 0;
	if(flag_systick_auto){
		uint8_t* thrust = CAN_getMessagePointer(fmi_auto_thrust);
		abs_thrust = abs(thrust[0]) + abs(thrust[1]) + abs(thrust[2]) + abs(thrust[3]) +
				abs(thrust[4]) + abs(thrust[5]) + abs(thrust[6]) + abs(thrust[7]);
		abs_thrust = abs_thrust/8;
	} else {
		float temp = 0.0f;
		/* Sum absolute thruster values. */
		if(th1>0.0f) temp += th1;
		else temp -= th1;
		if(th2>0.0f) temp += th2;
		else temp -= th2;
		if(th3>0.0f) temp += th3;
		else temp -= th3;
		if(th4>0.0f) temp += th4;
		else temp -= th4;
		if(th5>0.0f) temp += th5;
		else temp -= th5;
		if(th6>0.0f) temp += th6;
		else temp -= th6;
		if(th7>0.0f) temp += th7;
		else temp -= th7;
		if(th8>0.0f) temp += th8;
		else temp -= th8;

		abs_thrust = (temp*100.0f)/8;

	}
	return abs_thrust;
}
