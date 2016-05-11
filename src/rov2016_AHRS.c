/**
 **************************************************************************************
 * @file    rov2016_AHRS.c
 * @author  Sivert Sliper, Stian Soerensen
 * @version V1.0
 * @date    3-February-2016
 * @brief   This file contains functions for implementing a Attitude Heading Reference
 * 			System based on 3 dimensional accelerometer, magnetometer and gyroscope
 * 			sensors.
 **************************************************************************************
 */

/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include <math.h>
#include <stdint.h>

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Static Function declarations --------------------------------------------------------*/
//float magnetometer_heading(int16_t mx, int16_t my, int16_t mz);

/* Private variables -------------------------------------------------------------------*/
/* Sensor fusion*/
int32_t acc_pitch_k_1=0, acc_roll_k_1=0, acc_pitch_k=0, acc_roll_k=0;
int32_t delta_roll_acc=0, delta_pitch_acc=0;
int32_t delta_pitch_k=0, delta_roll_k=0;
int32_t pitch_roll_buff[2] = {0};

float weight_acc=0.0f; weight_acc_pitch=0.0f; weight_acc_roll=0.0f;
float gx_k=0.0f, gy_k=0.0f, gz_k=0.0f;
float delta_pitch_gyro=0.0f, delta_roll_gyro=0.0f;
float weight_gyro_pitch=0.0f; weight_gyro_roll=0.0f;
float est_delta_pitch=0.0f; est_delta_roll=0.0f;
float est_pitch_k=0.0f; est_pitch_k_1=0.0f;
float est_roll_k=0.0f; est_roll_k_1=0.0f;


float timestep = 0.1f; // Timestep = 0.1s.
float gain_pitch_gyro = 0.25f; // 1/4
float gain_roll_gyro = 0.11; // 1/9


/* Heading calc. */
typedef enum{
	QUADRANT_1, 		// X>0, Y>0
	QUADRANT_2,			// X<0, Y>0
	QUADRANT_3,			// X<0, Y<0
	QUADRANT_4,			// X>0, Y<0
} quadrant;

/* Macro ----------------------------------------------------------------*/
#define PI				3.14159265f
/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	Calculates heading based on 3D magnetometer data.
 * @param  	int16_t mx, my, mz magnetometer data in 3D.
 * @retval 	Heading in degrees of clockwise rotation arround the z-axis
 * 			referenced to north. Return value of -1 indicates an error.
 */
float AHRS_magnetometer_heading(float mx, float my, float mz){

	/* Find quadrant. */
	quadrant quad = QUADRANT_1;
	if(mx > 0){
		if (my > 0){
			quad = QUADRANT_1;
		} else {
			quad = QUADRANT_4;
		}
	} else {
		if (my > 0){
			quad = QUADRANT_2;
		} else {
			quad = QUADRANT_3;
		}
	}

	/* Step 3: Calculate heading angle */
	float heading = -1.0f;

	switch(quad){
	case QUADRANT_1:
		heading = atan(my/mx);
		break;
	case QUADRANT_2:
		heading = atan(my/mx)+ PI;
		break;
	case QUADRANT_3:
		heading = atan(my/mx) + PI;
		break;
	case QUADRANT_4:
		heading = atan(my/mx) + 2*PI;
		break;
	default:
		heading = -2.0f;
	}

	/* Convert to degrees. */
	heading *= 180.0/PI;

	/* Correct for south/north error.*/
	// if(heading<180){
	// 	heading = heading + 180;
	// } else {
	// 	heading = heading -180;
	// }

	return heading;
}

/**
 * @brief  	Calculates pitch based on 3D accelerometer data.
 * @param  	int16_t ax, ay, az accelerometerdata in 3D.
 * @retval 	Pitch value in degrees*100.
 */

extern int32_t AHRS_accelerometer_pitch(int16_t ax, int16_t ay, int16_t az){
	float f_ax_g = ((float)ax)/1000.0;
	float f_ay_g = ((float)ay)/1000.0;
	float f_az_g = ((float)az)/1000.0;

	float ayz_abs = sqrtf(f_ay_g*f_ay_g + f_az_g*f_az_g);
	float pitch_rad = -atan2f(f_ax_g, ayz_abs);

	/* Rad -> deg */
	pitch_rad = pitch_rad*180.0/PI;
	return (int32_t)(pitch_rad*100.0f);
}

/**
 * @brief  	Calculates roll based on 3D accelerometer data.
 * @param  	int16_t ay, az accelerometerdata in 3D.
 * @retval 	Roll value in degrees*100.
 */

extern int32_t AHRS_accelerometer_roll(int16_t ay, int16_t az){
	float f_ay_g = ((float)ay)/1000.0;
	float f_az_g = ((float)az)/1000.0;

	if (f_az_g == 0) return 0.0f;
	return (int32_t)((atan2(f_ay_g, f_az_g)*180.0/PI)*100.0f);
}

/**
 * @brief  	Calculates tilt compensated heading.
 * @param  	float pitch and roll in degrees.
 * @param	float mx, my, mz accelerometerdata in 3D.
 * @retval 	Roll value in degrees.
 */
extern int32_t AHRS_tilt_compensated_heading(int32_t pitch, int32_t roll, int32_t mx, int32_t my, int32_t mz){

	float pitch_f = ((float)pitch)/100.0f;
	float roll_f = ((float)roll)/100.0f;
	float mx_f = ((float)mx)/100.0f;
	float my_f = ((float)my)/100.0f;
	float mz_f = ((float)mz)/100.0f;
	/* Normalize magnetometer readings. */
	float magnetometer_magnitude = sqrtf(mx_f*mx_f + my_f*my_f + mz_f*mz_f);
	float mx_norm, my_norm, mz_norm;
	mx_norm = mx_f/magnetometer_magnitude;
	my_norm = my_f/magnetometer_magnitude;
	mz_norm = mz_f/magnetometer_magnitude;

	/* Calculate tilt compensated magnetic sensor measurements mx_2, my_2, mz_2 based on eq. 12 from
	 * STM's AN3192, Appendix A.2.
	 */
	float sinpitch = sinf(pitch_f*PI/180);
	float cospitch = cosf(pitch_f*PI/180);
	float sinroll = sinf(roll_f*PI/180);
	float cosroll = cosf(roll_f*PI/180);

	float mx_2, my_2;
	mx_2 = mx_norm*cospitch + mz_norm*sinpitch;
	my_2 = mx_norm*sinroll*sinpitch + my_norm*cosroll - mz_norm*sinroll*cospitch;
	//	mz_2 = -mx_norm*cosroll*sinpitch + my_norm*sinroll + mz_norm*cosroll*cospitch;

	/* Find quadrant. */
	quadrant quad = QUADRANT_1;
	if(mx_2 > 0){
		if (my_2 > 0){
			quad = QUADRANT_1;
		} else {
			quad = QUADRANT_4;
		}
	} else {
		if (my_2 > 0){
			quad = QUADRANT_2;
		} else {
			quad = QUADRANT_3;
		}
	}

	/* Calculate tilt compensated heading based on eqn 13 from STM's AN3192, Appendix A.2. */
	float heading = -2;
	switch(quad){
	case QUADRANT_1:
		heading = atan(my_2/mx_2);
		break;
	case QUADRANT_2:
		heading = atan(my_2/mx_2)+ PI;
		break;
	case QUADRANT_3:
		heading = atan(my_2/mx_2) + PI;
		break;
	case QUADRANT_4:
		heading = atan(my_f/mx_f) + 2*PI;
		break;
	default:
		heading = -2.0f;
	}

	/* Rad -> deg */
	heading = heading*180/PI;

	if((mx_2 == 0) && (my_2<0)) heading = 90.0;
	if((mx_2 == 0) && (my_2>0)) heading = 270.0;

	if(heading < 180) heading += 180;
	else heading -= 180;

	return (int32_t)(heading*100.0f);
}

/**
 * @brief  	Calculates estimates for pitch and roll angles
 * @param  	int16_t ax, ay, az: Acceleration in 10^-3 g.
 * @param	int16_t gx, gy, gz: Angular velocity in 8.75 mdps per LSb.
 * @param 	int16_t abs_thrust: Number proportional to the total thruster duty cycle.
 * @retval 	Roll value in degrees*100.
 */
extern int32_t * AHRS_sensor_fusion(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz){
	/* Accelerometer pitch, roll *******************************************************/
	/* Calculate pitch, roll */
	acc_pitch_k = AHRS_accelerometer_pitch(ax, ay, az);
	acc_roll_k = AHRS_accelerometer_roll(ay, az);

	/* Calculate accelerometers delta_pitch, delta_roll */
	delta_pitch_k = acc_pitch_k - acc_pitch_k_1;
	delta_roll_k = acc_roll_k - acc_roll_k_1;
	acc_pitch_k_1 = acc_pitch_k;
	acc_roll_k_1 = acc_roll_k;

	/* Calculate accelerometer weight -------------------------------------------------*/
	/* Thruster compensation*/
	float tot_thrust = (float)Interface_getTotalDuty()*0.01f;
	if(tot_thrust < 0.25f){
		weight_acc = 1 - 3.0*tot_thrust;
	} else {
		weight_acc = 1 - 0.75;
	}

	/* lowpass */
	if(delta_pitch_k > 100){
		weight_acc_pitch = weight_acc/((float)delta_pitch_k*0.01f);
	} else if (delta_pitch_k < -100) {
		weight_acc_pitch = - weight_acc/((float)delta_pitch_k*0.01f);
	} else{
		weight_acc_pitch = weight_acc;
	}

	if(delta_roll_k > 100){
		weight_acc_roll = weight_acc/((float)delta_roll_k*0.01f);
	} else if (delta_roll_k < -100) {
		weight_acc_roll = - weight_acc/((float)delta_roll_k*0.01f);
	} else{
		weight_acc_roll = weight_acc;
	}

	delta_pitch_acc = (float)acc_pitch_k - est_pitch_k_1;
	delta_roll_acc = (float)acc_roll_k - est_roll_k_1;

	/* Gyroscope estimates *************************************************************/
	/* Digital -> dps */
	gx_k = gx*0.00875f;
	gy_k = gy*0.00875f;
	gz_k = gz*0.00875f;

	/* Calc delta pitch,roll */
	delta_pitch_gyro = timestep * gy_k;
	delta_roll_gyro = timestep * gx_k;

	/* Gyroscope weights. -------------------------------------------------------------*/
	if (gy_k > 0){
		weight_gyro_pitch = gain_pitch_gyro * gy_k;
	} else{
		weight_gyro_pitch = - gain_pitch_gyro * gy_k;
	}

	if (gx_k > 0){
		weight_gyro_roll = gain_roll_gyro * gx_k;
	} else{
		weight_gyro_roll = - gain_roll_gyro * gx_k;
	}

	if(weight_gyro_pitch > 1.0f) weight_gyro_pitch = 1.0f;
	if(weight_gyro_roll > 1.0f) weight_gyro_roll = 1.0f;

	/* Sensor fusion *******************************************************************/
	est_delta_pitch = 0.01f*delta_pitch_acc * weight_acc_pitch;
	est_delta_pitch += delta_pitch_gyro * weight_gyro_pitch;
	est_delta_pitch /= (weight_acc_pitch + weight_gyro_pitch);

	est_delta_roll = 0.01f*delta_roll_acc * weight_acc_roll;
	est_delta_roll += delta_roll_gyro * weight_gyro_roll;
	est_delta_roll /= (weight_acc_roll + weight_gyro_roll);

	est_pitch_k = est_pitch_k_1 + est_delta_pitch;
	est_roll_k = est_roll_k_1 + est_delta_roll;

	est_pitch_k_1 = est_pitch_k;
	est_roll_k_1 = est_roll_k;

	/* Return estimated pitch and roll */
	pitch_roll_buff[0] = (int32_t)(est_pitch_k*100.0f);
	pitch_roll_buff[1] = (int32_t)(est_roll_k*100.0f);

	return pitch_roll_buff;
}
