/**
  ******************************************************************************
  * @file    rov2016_GPIO.c
  * @author  Sivert Sliper, Stian G. Sørensen
  * @version V1.0
  * @date    3-February-2016
  * @brief   This file contains all the functions for the GPIO peripheral.
  ******************************************************************************
  */

/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"

/* Global variables --------------------------------------------------------------------*/

/* Private function prototypes ---------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/

/* Function definitions ----------------------------------------------------------------*/


/**
 * @brief  	Initialize the GPIO-modules for:
 * 				- Output for compass LED's. (GPIOE)
 * 				- Debugging output PC6
 * 				- Leakage detector enable. (PC13 = enable)
 * @param  	None
 * @retval 	None
 */
extern void GPIO_init(void){
	/* Enable clocks. */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE);

	/* GPIO settings */
	GPIO_InitTypeDef GPIO_init;
	GPIO_init.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_init.GPIO_OType = GPIO_OType_PP;
	GPIO_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_init.GPIO_Speed = GPIO_Speed_Level_1;

	/* Enable outputs to compass LED's */
	GPIO_init.GPIO_Pin = 0xFF00; // Pin 8-15
	GPIO_Init(GPIOE, &GPIO_init);

	 /*  PC6 debugging pin and PC13 leakage detector enable  */
	 GPIO_init.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_13;
	 GPIO_Init(GPIOC, &GPIO_init);

	 /* The leakage detector is disabled by default. */
	 GPIOC->ODR |= GPIO_Pin_13;
} // end GPIO_init()

/**
 * @brief  	Sets PC13 low to enable voltage to the leakage detector.
 * @param  	None
 * @retval 	None
 */
extern void GPIO_leakage_detector_enable(void){
	GPIOC->ODR &= ~(GPIO_Pin_13);
}

/**
 * @brief  	Sets PC13 high to disable voltage to the leakage detector.
 * @param  	None
 * @retval 	None
 */
extern void GPIO_leakage_detector_disable(void){
	GPIOC->ODR |= GPIO_Pin_13;
}
