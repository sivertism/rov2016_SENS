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
static void bias_compensation(void);
/* Private variables -------------------------------------------------------------------*/
static uint8_t receive_buffer[6] = {0};
static uint8_t new_values = 0;
static int16_t bias_compensation_values[3] = {0};

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

	/* Configure High Pass filter */
	L3GD20_FilterConfigTypeDef filterInit;
	filterInit.HighPassFilter_Mode_Selection = L3GD20_HPM_REF_SIGNAL;
	filterInit.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_5;
	L3GD20_FilterConfig(&filterInit);

	L3GD20_FilterCmd(L3GD20_HIGHPASSFILTER_DISABLE);

	bias_compensation();
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
 * @brief	Returns gyro-values from private buffer in degrees per second.
 * @param	Axis, can be a value of GYRO_AXIS_x.
 * @retval	int16_t angular velocity in 8.75 mdps/LSb [number from datasheet].
 */
extern int16_t gyroscope_getRaw(uint8_t axis){
	new_values = 0;
	int16_t temp = ((uint16_t)receive_buffer[(axis*2)+1] << 8) | receive_buffer[axis*2];
	temp -= bias_compensation_values[axis];
	return temp;
}

/**
 * @brief	Returns gyro compensation biases.
 * @param	Axis, can be a value of GYRO_AXIS_x.
 * @retval	int16_t raw gyro bias value.
 */
extern int16_t gyroscope_getBias(uint8_t axis){
	return bias_compensation_values[axis];
}

/**
 * @brief	Returns gyro-values from private buffer in radians per second.
 * @param	Axis, can be a value of GYRO_AXIS_x.
 * @retval	float angular velocity in 1 rps/LSb.
 */
extern float gyroscope_getRPS(uint8_t axis){
	/* Convert from degrees per second to radians per second */
	//	double rps = (double)(((uint16_t)receive_buffer[(axis*2)+1] << 8) + receive_buffer[axis*2]);
	//	rps = rps - bias_compensation_values[axis]; // -32768 -> 32767
	float rps = (float)gyroscope_getRaw(axis);
	rps /= 114.2857143f;// Sensitivity (dps/LSb) // -286.72 -> 286.72
	rps *= (PI/180.0f); // dps -> rps // -5.004208 -> 5.004208
	new_values = 0;
	return rps;
}

/**
 * @brief	Finds the mean value of the gyro axes and saves it in private vars.
 * @param	None
 * @retval	None
 */
static void bias_compensation(void){
	int32_t x_temp=0,y_temp=0,z_temp=0;
	volatile uint8_t i = 0;
	volatile uint32_t j = 0;
	for(i=0;i<100;i++){
		gyroscope_updateValue();
		x_temp += gyroscope_getRaw(GYROSCOPE_X_AXIS);
		y_temp += gyroscope_getRaw(GYROSCOPE_Y_AXIS);
		z_temp += gyroscope_getRaw(GYROSCOPE_Z_AXIS);
		j = 360000;
		while(--j>0); // Wait 10 ms
	}
	x_temp /= 100;
	y_temp /= 100;
	z_temp /= 100;
	bias_compensation_values[GYROSCOPE_X_AXIS] =  (int16_t)x_temp;
	bias_compensation_values[GYROSCOPE_Y_AXIS] =  (int16_t)y_temp;
	bias_compensation_values[GYROSCOPE_Z_AXIS] =  (int16_t)z_temp;
}


