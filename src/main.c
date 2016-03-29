/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_gpio.h"
#include "rov2016_canbus.h"
#include "rov2016_Interface.h"

/* GLOBAL VARIABLES ----------------------------------------------------------*/
#include "def_global_vars.h"

/* Funtion Prototypes --------------------------------------------------------*/
void init(void);

/* Funtion Definitions -------------------------------------------------------*/
int main(void){
	/* Initialization *********************************************************/
	init();
	GPIOE->ODR = 0xFF00; // Turn off LED's
	fmi_topside_xbox_ctrl = CAN_addRxFilter(TOP_XBOX_CTRLS);
	fmi_topside_xbox_axes = CAN_addRxFilter(TOP_XBOX_AXES);
	fmi_vesc_current_9 = CAN_addRxFilter(VESC_CURRENT_BASE + 9);
	fmi_vesc_rpm_9 = CAN_addRxFilter(VESC_RPM_BASE + 9);
	fmi_vesc_mosfet_temperature_9 = CAN_addRxFilter(VESC_MOSFET_TEMP_BASE + 9);


	GPIOE->ODR = 0x0; // Turn off LED's
	/* Private vars ***********************************************************/

	/* Main loop *************************************************************/
	while(1){
//		GPIOE->ODR ^= MAIN_LOOP_LED << 8;
	} // end while
} // end main

