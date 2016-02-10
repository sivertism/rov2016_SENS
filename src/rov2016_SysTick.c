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
/* Global variables -------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"


/* Function declarations ---------------------------------------------------------------*/
void SysTick_init(void);
void SysTick_Handler(void);

uint8_t USART_getRxMessage(void);
uint8_t USART_getNewBytes(void);
void USART_transmit(uint8_t data);
void USART_timestamp_transmit(uint8_t timestamp);
void USART_datalog_transmit(uint8_t header, uint16_t data);
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

void SysTick_Handler(void){
	teller++;
	/* Measure acceleration and send the data via CAN */

	/* Check for new message on CAN and update LEDs */
	if(CAN_getRxMessages()>0){
		GPIOE->ODR ^= (1u << CAN_getByteFromMessage(2,0)) << 8;
	} // end if

	if(lsm303dlhc_getValues() == LSM303DLHC_ALL_VALUES){
		if(timeStamp>=255) timeStamp = 0;
		/* 'G' - A_X, 'H' - MAG_X, 'I' - MAG_Y
		 * 'J' - MAG_Z, 'K' - Int_temp, 'L' - leak_det
		 */
		USART_timestamp_transmit(timeStamp++);
		USART_datalog_transmit('G', accelerometer_getData(ACCELEROMETER_X_AXIS));
		USART_datalog_transmit('X', magnetometer_getData(MAGNETOMETER_X_AXIS));
		USART_datalog_transmit('Y', magnetometer_getData(MAGNETOMETER_Y_AXIS));
		USART_datalog_transmit('Z', magnetometer_getData(MAGNETOMETER_Z_AXIS));
	} // end if

	if(teller>100){
		GPIOE->ODR ^= SYSTICK_LED << 8;

		accelerometer_updateValue();
		magnetometer_updateValue();

		teller = 0;
//		CAN_transmitAcceleration(&accelerometer_data);
	} // end if

} // end Systick_Handler()
