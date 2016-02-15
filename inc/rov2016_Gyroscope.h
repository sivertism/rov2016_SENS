/**
  **************************************************************************************
  * @file    rov2016_Gyroscope.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    10-February-2016
  * @brief   This file contains local variables and macros for rov2016_Gyroscope.h
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
#define PI												3.141592654f
/* Macros for getData methods. */
#define GYROSCOPE_X_AXIS								0
#define GYROSCOPE_Y_AXIS								1
#define GYROSCOPE_Z_AXIS								2

/* Extern variables --------------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/

/* Init functions */
extern void gyroscope_init(void);
/* Utility */
extern void gyroscope_updateValue(void);
extern int16_t gyroscope_getRaw(uint8_t axis);
extern float gyroscope_getRPS(uint8_t axis);
extern uint8_t gyroscope_getValues(void);
extern int16_t gyroscope_getBias(uint8_t axis);
