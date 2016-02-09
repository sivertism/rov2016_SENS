/**
  **************************************************************************************
  * @file    rov2016_Accelerometer.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains local variables and macros for rov2016_Accelerometer.h
  **************************************************************************************
  */

/* Include -----------------------------------------------------------------------------*/

/* Macro ------------------------------------------------------------------------------*/
/* Extern variables -------------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/

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
extern void accelerometer_init(void); // Initializes the accelerometer.

/**
 * @brief   Reads the acceleration in 3 axes from the accelerometer and stores the values
 * 			in a private buffer.
 * @param
 * @retval
 */
extern void accelerometer_readValue(void);

/**
 * @brief  Returns the acceleration data for the selected axis.
 * @param  uint8_t axis - The wanted axis, X=0, Y=1, Z=2
 * @retval The acceleration data for the selected axis (int16_t) in 1mg/LSb.
 */
extern int16_t accelerometer_getData(uint8_t axis);
