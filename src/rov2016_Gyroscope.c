/**
  ******************************************************************************
  * @file    rov2016_Gyroscope.c
  * @author	 Sivert Sliper, Stian G. Sørensen
  * @version V1.0
  * @date    10-February-2016
  * @brief   This file uses functions from the stm32f3_discovery_l3gd20
  * 		library to enable sensing of angular velocity via the l3gd20
  * 		gyroscope.
  ******************************************************************************
  */
/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "rov2016_Gyroscope.h"
#include "stm32f3_discovery_l3gd20.h"

/* Global variables --------------------------------------------------------------------*/

/* Private Function Prototypes ----------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/
static uint8_t receive_buffer[6] = {0};
static uint8_t new_values = 0;

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	Initializes the L3GD20 Gyroscope IC with the following key parameters:
 * 				Sample rate:			100 Hz
 * 				Sensitivity:			8.75 mdps/LSb
 * 				Bandwidth:				12.5 Hz
 *
 * 			Axes:
 * 				x(roll)					Points from the usb's to the LED compass.
 * 				y(pitch)				Points from the user- to the reset button.
 * 				z(yaw)					Points upwards.
 *
 * @param  None
 * @retval None
 */
extern void gyroscope_init(void){
	L3GD20_InitTypeDef gyroInit;
	gyroInit.Axes_Enable = L3GD20_AXES_ENABLE;
	gyroInit.Band_Width = L3GD20_BANDWIDTH_1;
	gyroInit.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
	gyroInit.Endianness = L3GD20_BLE_LSB;
	gyroInit.Full_Scale = L3GD20_FULLSCALE_250;
	gyroInit.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
	gyroInit.Power_Mode = L3GD20_MODE_ACTIVE;
	L3GD20_Init(&gyroInit);

} // end gyroscope_init()


/**
 * @brief	Updates gyro values to a private buffer.
 * @param	None.
 * @retval	None.
 */
extern void gyroscope_updateValue(void){
	L3GD20_Read(&receive_buffer[0], L3GD20_OUT_X_L_ADDR, 6);
	new_values = 1;
}

/**
 * @brief	Checks wether there is new data available in the private buffer.
 * @param	None.
 * @retval	uint8_t 1 if new data available, otherwise 0.
 */
extern uint8_t gyroscope_getValues(void){
	return new_values;
}

/**
 * @brief	Returns gyro-values from private buffer.
 * @param	Axis, can be a value of GYRO_AXIS_x.
 * @retval	int16_t angular velocity in 8.75 mdps/LSb.
 */
extern int16_t gyroscope_getData(uint8_t axis){
	int16_t temp;
	temp = (receive_buffer[(axis*2)+1] << 8) | receive_buffer[axis*2];
	new_values = 0;
	return temp;
}
