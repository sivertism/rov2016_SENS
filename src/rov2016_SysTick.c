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
#include "rov2016_Interface.h"
#include "rov2016_AHRS.h"
#include "stm32f3_discovery_lsm303dlhc.h"
#include "math.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Private variables -------------------------------------------------------------------*/

static uint8_t isActive=0, counter_10_hz=0;
static uint16_t counter_1_hz=0;

/* Private function declarations -------------------------------------------------------*/

/* Macro -------------------------------------------------------------------------------*/
//#define DEBUG_MODE

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  Configures the SysTick timer for 100 Hz interrupt frequency.
 * @param  None
 * @retval None
 */
extern void SysTick_init(void) {
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	NVIC_SetPriority(SysTick_IRQn, 7);
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
void SysTick_Handler(void){
	counter_10_hz++;
	counter_1_hz++;

	Interface_VESC_requestRPM();

	/* Check for messages from topside and set LED's accordingly. */
	if(CAN_getRxMessages()>0){
		uint8_t buttons_1 = CAN_getByteFromMessage(fmi_topside_xbox_axes,4);
		GPIOE->ODR = (uint16_t)buttons_1 << 12;
	}

	/* 10 Hz loop */
	if((counter_10_hz>9)){
		counter_10_hz = 0;
		flag_systick_update_attitude = 1;
		flag_systick_transmit_thrust = 1;
		flag_systick_update_depth = 1;
		
		/* Check for USART messages, start if 'k' */
		if (USART_getNewBytes()>0){
			uint8_t melding = USART_getRxMessage();
			if (melding == 'k') {
				isActive = 1;
				USART_transmit(USART2, 0x02); // STX
			}
			if (melding == 's'){
				isActive = 0;
				USART_transmit(USART2, 0x03); //ETX
			}
		}
	} // end 10 hz loop.


	/* 1 Hz loop */
	if(counter_1_hz>99){
		flag_systick_update_ms5803_temp = 1;
		Interface_VESC_requestTemperature();
		CAN_transmitAlive();

		GPIOE->ODR ^= (uint16_t)SYSTICK_LED << 8;
		counter_1_hz = 0;
	}// end 1 Hz loop.
} // end Systick_Handler()
