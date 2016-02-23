/*
 * File: init.c
 *
 * Brief: Contains function(s) for calling initialization functions
 */

/* Include -------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "rov2016_Accelerometer.h"
#include "rov2016_canbus.h"
#include "rov2016_UART.h"
#include "rov2016_Gyroscope.h"
#include "rov2016_TIM.h"
#include "rov2016_ADC.h"
#include "rov2016_SysTick.h"
/* Funtion Prototypes --------------------------------------------------------*/
void init(void);
void GPIO_init(void);

/* Funtion Definitions -------------------------------------------------------*/

void init(void){
	GPIO_init();
	USART_init();
	CAN_Config();
	accelerometer_init();
	magnetometer_init();
	gyroscope_init();
	ADC_init();
	TIM4_init();
	TIM2_init();
	SysTick_init();
}
