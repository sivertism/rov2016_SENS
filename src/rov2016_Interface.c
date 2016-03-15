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


/* Static Function declarations --------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Transmit SENSOR_AN_RAW package.
 * @param  None
 * @retval The number of unprocessed messages (uint8_t).
 */
void CAN_transmit_AN_RAW(void){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = SENSOR_AN_RAW;
	TxMsg.DLC = 8;
	TxMsg.IDE = CAN_ID_STD;
	TxMsg.RTR = CAN_RTR_DATA;

	TxMsg.Data[0] = (uint8_t) (ADC4_getChannel(0) & 0xFF);
	TxMsg.Data[1] = (uint8_t) (ADC4_getChannel(0) >> 8);
	TxMsg.Data[2] = (uint8_t) (ADC1_getChannel(1) & 0xFF);
	TxMsg.Data[3] = (uint8_t) (ADC1_getChannel(1) >> 8);
	TxMsg.Data[4] = (uint8_t) (ADC1_getChannel(2) & 0xFF);
	TxMsg.Data[5] = (uint8_t) (ADC1_getChannel(2) >> 8);
	TxMsg.Data[6] = (uint8_t) (ADC1_getChannel(3) & 0xFF);
	TxMsg.Data[7] = (uint8_t) (ADC1_getChannel(3) >> 8);

	/* Put message in Tx Mailbox and store the mailbox number. */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);

	/* Wait for Transmit */
	while(CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok);
}

/**
 * @brief  Transmit acceleration measurements in all 3 axes.
 * @param  Pointer to an array of dimension 6 containing measurements.
 * @retval None
 */
void CAN_transmitAcceleration(int8_t *acc_array){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = SENSOR_ACCELERATION;
	TxMsg.DLC = 6;

	uint8_t i = 0;
	for(i=0;i<6;i++) TxMsg.Data[i] = *acc_array++;

	/* Put message in Tx Mailbox and store the mailbox number. */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);

	/* Wait on Transmit */
	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok));
}

/**
 * @brief  Transmit 4 quaternions as 4 int16_t split into 8 bytes.
 * @param  int16_t q0, q1, q2, q3
 * @retval None.
 */
extern void CAN_transmitQuaternions(int16_t q0, int16_t q1, int16_t q2, int16_t q3){
	/* Toggle status LED */
	GPIOE->ODR ^= CAN_TX_LED << 8;

	/* Configure the message to be transmitted. */
	TxMsg.StdId = SENSOR_AHRS_QUATERNIONS;
	TxMsg.DLC = 8;

	TxMsg.Data[0] = (uint8_t)(q0 >> 8u);
	TxMsg.Data[1] = (uint8_t)(q0 & 0xFF);
	TxMsg.Data[2] = (uint8_t)(q1 >> 8u);
	TxMsg.Data[3] = (uint8_t)(q1 & 0xFF);
	TxMsg.Data[4] = (uint8_t)(q2 >> 8u);
	TxMsg.Data[5] = (uint8_t)(q2 & 0xFF);
	TxMsg.Data[6] = (uint8_t)(q3 >> 8u);
	TxMsg.Data[7] = (uint8_t)(q3 & 0xFF);

	/* Put message in Tx Mailbox and store the mailbox number. */
	TransmitMailbox = CAN_Transmit(CAN1, &TxMsg);

	/* Wait on Transmit */
	while((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok));
}
