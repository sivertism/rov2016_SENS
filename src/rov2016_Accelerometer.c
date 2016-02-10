/**
  ******************************************************************************
  * @file    rov2016_Accelerometer.c
  * @author
  * @version V1.0
  * @date    8-September-2015
  * @brief   This file uses functions from the stm32f3_discovery_lsm303dlhc
  * 			library to enable reading of accelerometer data
  ******************************************************************************
  */

/* Include------------------------------------------------------------------------------*/
#include "stm32f3_discovery_lsm303dlhc.h"
#include "rov2016_Accelerometer.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Static Function declarations --------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/
static uint8_t accelerometer_receive_buffer[6];
static uint8_t magnetometer_receive_buffer[6];

/* Funksjonsprototyper */

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief   Initializes the accelerometer IC with the following key parameters:
 * 				- Samples acceleration at a frequency of 100 Hz.
 * 				- Measures acceleration in all 3 axes.
 * 				- +/- 2g full scale range.
 * 				- 12 bit resolution, i.e. 1mg.
 * 			Axes:
 * 				- x points from the usb-ports towards the LED's.
 * 				- y points from the USER-button towards the RESET-button.
 * 				- z points upwards.
 * @param  None
 * @retval None
 */
extern void accelerometer_init(void){
	LSM303DLHCAcc_InitTypeDef aks_init;
	aks_init.Power_Mode = LSM303DLHC_NORMAL_MODE;
	aks_init.AccOutput_DataRate = LSM303DLHC_ODR_100_HZ; // Sampling freq.
	aks_init.Axes_Enable = LSM303DLHC_AXES_ENABLE; // All axes.
	aks_init.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
	aks_init.High_Resolution = LSM303DLHC_HR_ENABLE;
	aks_init.High_Resolution = LSM303DLHC_BlockUpdate_Continous; // Cont. sampling.
	aks_init.Endianness = LSM303DLHC_BLE_LSB;
	LSM303DLHC_AccInit(&aks_init);
} // end accelerometer_init()

/**
 * @brief   Initializes the magnetometer with the following key parameters:
 * 				- Samples magneticc field at a frequency of 100 Hz.
 * 				- Measures magnetic field strength in all 3 axes.
 * 				- +/- 1.9 Ga full scale range.
 * 				- 12 bit resolution, i.e. 1mg.
 * 			Axes:
 * 				- x points from the usb-ports towards the LED's.
 * 				- y points from the USER-button towards the RESET-button.
 * 				- z points upwards.
 * @param  None
 * @retval None
 */
extern void magnetometer_init(void){
	LSM303DLHCMag_InitTypeDef magInit;
	magInit.MagFull_Scale = LSM303DLHC_FS_1_9_GA;
	magInit.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_DISABLE;
	magInit.MagOutput_DataRate = LSM303DLHC_ODR_100_HZ;
	magInit.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
	LSM303DLHC_MagInit(&magInit);
}

/**
 * @brief   Reads the acceleration in 3 axes from the accelerometer and stores the values
 * 			in a private buffer.
 * @param
 * @retval
 */
extern void accelerometer_updateValue(void){
	LSM303DLHC_Read(ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, accelerometer_receive_buffer, 6);
	accelerometer_data[0] = accelerometer_receive_buffer[0] >> 4;
	accelerometer_data[1] = accelerometer_receive_buffer[1];
	accelerometer_data[2] = accelerometer_receive_buffer[2] >> 4;
	accelerometer_data[3] = accelerometer_receive_buffer[3];
	accelerometer_data[4] = accelerometer_receive_buffer[4] >> 4;
	accelerometer_data[5] = accelerometer_receive_buffer[5];
} // End accelerometer_updateValue(void)

/**
 * @brief   Reads the magnetic field strength in 3 axes from the accelerometer and stores the
 * 			values in a private buffer.
 * @param
 * @retval
 */
extern void magnetometer_updateValue(void){
	LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, magnetometer_receive_buffer, 6);
	magnetometer_data[0] = magnetometer_receive_buffer[0] >> 4;
	magnetometer_data[1] = magnetometer_receive_buffer[1];
	magnetometer_data[2] = magnetometer_receive_buffer[2] >> 4;
	magnetometer_data[3] = magnetometer_receive_buffer[3];
	magnetometer_data[4] = magnetometer_receive_buffer[4] >> 4;
	magnetometer_data[5] = magnetometer_receive_buffer[5];
} // End magnetometer_updateValue()

/**
 * @brief  Returns the acceleration data for the selected axis.
 * @param  uint8_t axis - The wanted axis, X=0, Y=1, Z=2
 * @retval The acceleration data for the selected axis (int16_t) in 1mg/LSb.
 */
extern int16_t accelerometer_getData(uint8_t axis){
	uint16_t temp;
	if(axis == 0) temp   = (accelerometer_receive_buffer[1] << 8) | accelerometer_receive_buffer[0];
	else if(axis == 1) temp   = (accelerometer_receive_buffer[3] << 8) | accelerometer_receive_buffer[2];
	else if(axis == 2) temp   = (accelerometer_receive_buffer[5] << 8) | accelerometer_receive_buffer[4];
	return ((int16_t)(temp)) >> 4;
}
