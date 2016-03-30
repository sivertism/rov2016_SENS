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
extern float MCD_APP_TEAM_AHRS(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz);
extern float AHRS_accelerometer_roll(int16_t ay, int16_t az);
extern float AHRS_accelerometer_pitch(int16_t ax, int16_t ay, int16_t az);
extern float AHRS_tilt_compensated_heading(float pitch, float roll, float mx, float my, float mz);