/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_can.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "rov2016_canbus.h"

/* GLOBAL VARIABLES ----------------------------------------------------------*/
#include "def_global_vars.h"

/* Funtion Prototypes --------------------------------------------------------*/
void init(void);

/* Funtion Definitions -------------------------------------------------------*/
int main(void){
	/* Initialization *********************************************************/
	init();
	topside_xbox_ctrl_fmi = CAN_addRxFilter(TOP_XBOX_CTRLS);
	printf("CTRL FMI: %d", topside_xbox_ctrl_fmi);
	topside_xbox_axes_fmi = CAN_addRxFilter(TOP_XBOX_AXES);
	printf("AXES FMI: %d", topside_xbox_axes_fmi);


	GPIOE->ODR = 0; // Turn off LED's
	/* Private vars ***********************************************************/
	printf("Init complete.");
	/* Main loop *************************************************************/
	while(1){
//		GPIOE->ODR ^= MAIN_LOOP_LED << 8;
	} // end while
} // end main

