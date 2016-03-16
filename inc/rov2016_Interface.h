/**
  **************************************************************************************
  * @file    rov2016_Interface.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V1.0
  * @date    3-February-2016
  * @brief   This file contains local variables and macros for rov2016_Interface.c
  **************************************************************************************
  */

/* Exported function prototypes --------------------------------------------------------*/
uint16_t ADC1_getChannel(uint8_t channel);
uint16_t ADC4_getChannel(uint8_t channel);
extern void CAN_transmitQuaternions(int16_t q0, int16_t q1, int16_t q2, int16_t q3);
extern void CAN_transmitAcceleration(uint8_t* acc_array);
extern void CAN_transmit_AN_RAW(void);
