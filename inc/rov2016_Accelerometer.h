/**
  **************************************************************************************
  * @file    rov2016_Accelerometer.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains local variables and macros for rov2016_Accelerometer.h
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
#define ACCELEROMETER_X_AXIS							0
#define ACCELEROMETER_Y_AXIS							1
#define ACCELEROMETER_Z_AXIS							2

/* Extern variables --------------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/

/* Init functions */
extern void accelerometer_init(void);
extern void magnetometer_init(void);
/* Utility */
extern void accelerometer_updateValue(void);
extern int16_t accelerometer_getData(uint8_t axis);
