/**
  **************************************************************************************
  * @file    	rov2016_AHRS.h
  * @author  	Sivert Sliper, Stian Soerensen
  * @version 	V01
  * @date    	19-March-2016
  * @brief   	This file contains macros  and extern function prototypes for
  * 			rov2016_AHRS.h
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/

/* Extern variables --------------------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/
float AHRS_magnetometer_heading(float mx, float my, float mz);
extern int32_t AHRS_accelerometer_roll(int16_t ay, int16_t az);
extern int32_t AHRS_accelerometer_pitch(int16_t ax, int16_t ay, int16_t az);
extern int32_t AHRS_tilt_compensated_heading(int32_t pitch, int32_t roll, float mx, float my, float mz);
