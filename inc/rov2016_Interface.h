/**
  **************************************************************************************
  * @file    	rov2016_Interface.h
  * @author  	Sivert Sliper, Stian Soerensen
  * @version 	V1.0
  * @date   	3-February-2016
  * @brief   	This file contains macros and extern function prototypes for
  * 			rov2016_Interface.c
  **************************************************************************************
  */

/* VESC interface. */
#define NUMBER_OF_VESCS							8
#define VESC_TEMPERATURE_ALARM_LIMIT			60u
#define VESC_DUTY_CYCLE_MAX						0.94f
#define VESC_DUTY_CYCLE_MIN						-0.94f
#define VESC_DUTY_CYCLE_DIR_CHANGE				0.3f

#define	ESC_ID_1								1
#define	ESC_ID_2								2
#define	ESC_ID_3								3
#define	ESC_ID_4								4
#define	ESC_ID_5								5
#define	ESC_ID_6								6
#define	ESC_ID_7								7
#define	ESC_ID_8								8
#define	ESC_ID_9								9
#define	ESC_ID_10								10
#define	ESC_ID_11								11
#define	ESC_ID_12								12

/* Exported function prototypes --------------------------------------------------------*/
uint16_t ADC1_getChannel(uint8_t channel);
uint16_t ADC4_getChannel(uint8_t channel);

/* CANBUS to topside. */
extern void CAN_transmitQuaternions(int16_t q0, int16_t q1, int16_t q2, int16_t q3);
extern void CAN_transmitAcceleration(uint8_t* acc_array);
extern void CAN_transmit_AN_RAW(void);
extern void CAN_transmitAlive(void);
extern void CAN_transmitAHRS(int16_t pitch, int16_t roll, int16_t yaw, uint16_t heading);
extern int16_t* Interface_readController(void);
extern void CAN_transmitDepthTemp(uint16_t depth, uint16_t int_temp, uint16_t manip_temp);

/* VESC interface */
extern void VESC_setDutyCycle(uint8_t esc_id, float duty);
extern void Interface_transmitManualThrust(void);
extern void Interface_transmitOneThruster(uint8_t thrusterId);
extern void Interface_VESC_requestData(uint8_t esc_id, CAN_PACKET_ID request_message);
extern int32_t Interface_VESC_getInt32(uint8_t filter_match_index);
extern void Interface_VESC_requestTemperature(void);
extern void Interface_VESC_requestRPM(void);
