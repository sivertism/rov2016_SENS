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

/* Calibration values. */
#define MAG_X_OFFSET									18.0f
#define MAG_X_SCALE										506.0f
#define MAG_Y_OFFSET									-148.5f
#define MAG_Y_SCALE										519.5f
#define MAG_Z_OFFSET									-48.0f
#define MAG_Z_SCALE										453.0f


/* Macros for getData methods. */
#define ACCELEROMETER_X_AXIS							0
#define ACCELEROMETER_Y_AXIS							1
#define ACCELEROMETER_Z_AXIS							2
#define MAGNETOMETER_X_AXIS								0
#define MAGNETOMETER_Y_AXIS								1
#define MAGNETOMETER_Z_AXIS								2

/* Macros for getValues method.*/
#define ACC_NEW_VALUE_X									0b0000001
#define ACC_NEW_VALUE_Y									0b0000010
#define ACC_NEW_VALUE_Z									0b0000100
#define MAG_NEW_VALUE_X									0b0001000
#define MAG_NEW_VALUE_Y									0b0010000
#define MAG_NEW_VALUE_Z									0b0100000
#define LSM303DLHC_ALL_VALUES							0b0111111

/* Extern variables --------------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/

/* Init functions */
extern void accelerometer_init(void);
extern void magnetometer_init(void);
/* Utility */
extern void accelerometer_updateValue(void);
extern void magnetometer_updateValue(void);
extern int16_t accelerometer_getRawData(uint8_t axis);
extern float magnetometer_getData(uint8_t axis);
extern uint8_t lsm303dlhc_getValues(void);
extern int16_t magnetometer_getRawData(uint8_t axis);
