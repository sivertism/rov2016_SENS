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
#include "math.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Static Function declarations --------------------------------------------------------*/
//float magnetometer_heading(int16_t mx, int16_t my, int16_t mz);

/* Private variables -------------------------------------------------------------------*/
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
	if(heading<180){
		heading = heading + 180;
	} else {
		heading = heading -180;
	}

	return heading;
}

/**
 * @brief  	Calculates pitch based on 3D accelerometer data.
 * @param  	int16_t ax, ay, az accelerometerdata in 3D.
 * @retval 	Pitch value in degrees.
 */

extern float AHRS_accelerometer_pitch(int16_t ax, int16_t ay, int16_t az){
	float f_ax_g = (float)ax/1000.0;
	float f_ay_g = (float)ay/1000.0;
	float f_az_g = (float)az/1000.0;

	float ayz_abs = sqrtf(f_ay_g*f_ay_g + f_az_g*f_az_g);

	float pitch_rad = -atan2f(f_ax_g, ayz_abs);
	return pitch_rad*180/PI;
}

/**
 * @brief  	Calculates roll based on 3D accelerometer data.
 * @param  	int16_t ay, az accelerometerdata in 3D.
 * @retval 	Roll value in degrees.
 */

extern float AHRS_accelerometer_roll(int16_t ay, int16_t az){
	float f_ay_g = (float)ay/1000.0;
	float f_az_g = (float)az/1000.0;

	if (f_az_g == 0) return 0.0f;
	return atan2(f_ay_g, f_az_g)*180/PI;
}

/**
 * @brief  	Calculates tilt compensated heading.
 * @param  	float pitch and roll.
 * 			float mx, my, mz accelerometerdata in 3D.
 * @retval 	Roll value in degrees.
 */
extern float AHRS_tilt_compensated_heading(float pitch, float roll, float mx, float my, float mz){
	/* Normalize magnetometer readings. */
	float magnetometer_magnitude = sqrtf(mx*mx + my*my + mz*mz);
	float mx_norm, my_norm, mz_norm;
	mx_norm = mx/magnetometer_magnitude;
	my_norm = my/magnetometer_magnitude;
	mz_norm = mz/magnetometer_magnitude;

	/* Calculate tilt compensated magnetic sensor measurements mx_2, my_2, mz_2 based on eq. 12 from
	 * STM's AN3192, Appendix A.2.
	 */
	float sinpitch = sinf(pitch*PI/180);
	float cospitch = cosf(pitch*PI/180);
	float sinroll = sinf(roll*PI/180);
	float cosroll = cosf(roll*PI/180);

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
		heading = atan(my/mx) + 2*PI;
		break;
	default:
		heading = -2.0f;
	}

	/* Rad -> deg */
	heading = heading*180/PI;

	if((mx_2 == 0) && (my_2<0)) heading = 90.0;
	if((mx_2 == 0) && (my_2>0)) heading = 270.0;

	return heading;
}

extern float MCD_APP_TEAM_AHRS(float ax, float ay, float az, float mx, float my, float mz, float gx, float gy, float gz){

	float fNormAcc,fSinRoll,fCosRoll,fSinPitch,fCosPitch = 0.0f, RollAng = 0.0f, PitchAng = 0.0f;
	float fTiltedX,fTiltedY = 0.0f;
	volatile float HeadingValue = 0.0f;

	ax = ax/100.0;
	ay = ay/100.0;
	az = az/100.0;
    fNormAcc = sqrt((ax*ax)+(ay*ay)+(ay*ay));

    fSinRoll = -ay/fNormAcc;
    fCosRoll = sqrt(1.0-(fSinRoll * fSinRoll));
    fSinPitch = ax/fNormAcc;
    fCosPitch = sqrt(1.0-(fSinPitch * fSinPitch));
   if ( fSinRoll >0)
   {
     if (fCosRoll>0)
     {
       RollAng = acos(fCosRoll)*180/PI;
     }
     else
     {
       RollAng = acos(fCosRoll)*180/PI + 180;
     }
   }
   else
   {
     if (fCosRoll>0)
     {
       RollAng = acos(fCosRoll)*180/PI + 360;
     }
     else
     {
       RollAng = acos(fCosRoll)*180/PI + 180;
     }
   }

    if ( fSinPitch >0)
   {
     if (fCosPitch>0)
     {
          PitchAng = acos(fCosPitch)*180/PI;
     }
     else
     {
        PitchAng = acos(fCosPitch)*180/PI + 180;
     }
   }
   else
   {
     if (fCosPitch>0)
     {
          PitchAng = acos(fCosPitch)*180/PI + 360;
     }
     else
     {
        PitchAng = acos(fCosPitch)*180/PI + 180;
     }
   }

    if (RollAng >=360)
    {
      RollAng = RollAng - 360;
    }

    if (PitchAng >=360)
    {
      PitchAng = PitchAng - 360;
    }

    fTiltedX = mx*fCosPitch+mz*fSinPitch;
    fTiltedY = mx*fSinRoll*fSinPitch+my*fCosRoll-my*fSinRoll*fCosPitch;

    HeadingValue = (float) ((atan2f((float)fTiltedY,(float)fTiltedX))*180)/PI;

    if (HeadingValue < 0)
    {
      HeadingValue = HeadingValue + 360;
    }
    return HeadingValue;
}
