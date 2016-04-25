/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "rov2016_canbus.h"
#include "rov2016_Interface.h"
#include "rov2016_Accelerometer.h"
#include "rov2016_SPI.h"
#include "main.h"
#include <math.h>
#include <stdint.h>


/* GLOBAL VARIABLES ----------------------------------------------------------*/
#include "def_global_vars.h"

/* PRIVATE VARIABLES ---------------------------------------------------------*/
static int32_t mx=0.0f, my=0.0f, mz=0.0f;
static int16_t mx_raw=0, my_raw=0, mz_raw=0;
static int32_t heading = 0.0f, pitch=0.0f, roll=0.0f;
static int32_t depth = 0;
static uint16_t int_temp = 0;
static int16_t ax=0, ay=0, az=0;
static int32_t surface_pressure = 9800;
static int32_t current_pressure = 0;


/* Funtion Definitions -------------------------------------------------------*/
int main(void){
	/* Initialization *********************************************************/
	init();
	GPIOE->ODR = 0xFF00; // Turn off LED's
	fmi_topside_xbox_ctrl = CAN_addRxFilter(TOP_XBOX_CTRLS);
	fmi_topside_xbox_axes = CAN_addRxFilter(TOP_XBOX_AXES);
	fmi_topside_sens_ctrl = CAN_addRxFilter(TOP_SENS_CTRL);


	GPIOE->ODR = 0x0; // Turn off LED's
	/* Private vars ***********************************************************/

	/* Main loop **************************************************************/
	while(1){

//		/* Calibrate gyroscope. */
//		if (flag_systick_calibrate_gyro){
//			gyroscope_bias_compensation();
//			CAN_deleteRxByte(fmi_topside_sens_ctrl, 0);
//		}

		/* Zero depth */
		if (flag_systick_zero_pressure){
			surface_pressure = current_pressure;
			CAN_deleteRxByte(fmi_topside_sens_ctrl, 1);
			flag_systick_zero_pressure = 0;
		}

		/* Calculate pitchand roll and transmit via CAN-bus */
		if (flag_systick_update_attitude){
			accelerometer_updateValue();
			magnetometer_updateValue();
			//gyroscope_updateValue();

			ax = accelerometer_getRawData(ACCELEROMETER_X_AXIS);
			ay = accelerometer_getRawData(ACCELEROMETER_Y_AXIS);
			az = accelerometer_getRawData(ACCELEROMETER_Z_AXIS);

			mx = magnetometer_getData(MAGNETOMETER_X_AXIS);
			my = magnetometer_getData(MAGNETOMETER_Y_AXIS);
			mz = magnetometer_getData(MAGNETOMETER_Z_AXIS);

			mx_raw = magnetometer_getRawData(MAGNETOMETER_X_AXIS);
			my_raw = magnetometer_getRawData(MAGNETOMETER_Y_AXIS);
			mz_raw = magnetometer_getRawData(MAGNETOMETER_Z_AXIS);

			/* Sensor axes -> rov-axes.*/
			int16_t temp = ax;
			ax = -az;
			az = -ay;
			ay = temp;

			float temp2 = mx;
			mx = -mz;
			mz = -my;
			my = temp2;

			pitch = AHRS_accelerometer_pitch(ax, ay, az);
			roll = AHRS_accelerometer_roll(ay, az);

			//***********
			CAN_transmitQuaternions(mx_raw, my_raw, mz_raw, 0); // sender rå magnetometerverdier

			uint8_t acc_array[6] = {(uint8_t)(ax >> 8),
									(uint8_t)(ax & 0xFF),
									(uint8_t)(ay >> 8),
									(uint8_t)(ay & 0xFF),
									(uint8_t)(az >> 8),
									(uint8_t)(az & 0xFF)};

			CAN_transmitAcceleration(acc_array);
			//***********

			if(!flag_systick_update_heading){
			CAN_transmitAHRS((int16_t)(-pitch/10), (int16_t)(roll/10), 0, \
				(uint16_t)(heading/10));
			}
			flag_systick_update_attitude = 0;
		}

		/* Calculate heading and transmit pitch, roll, heading to topside. */
		if(flag_systick_update_heading){
			heading = AHRS_tilt_compensated_heading(pitch, roll, mx, my, mz);
			CAN_transmitAHRS((int16_t)(-pitch/10), (int16_t)(roll/10), 0, \
				(uint16_t)(heading/10));
			flag_systick_update_heading = 0;
		}

		/* Update temperature for the MS5803 pressure sensor. */
		if (flag_systick_update_ms5803_temp){
			MS5803_updateDigital(MS5803_CONVERT_TEMPERATURE);
			flag_systick_update_ms5803_temp = 0;
		}

		/* Update and send depth[mm] and internal temperature [celsius] to topside. */
		if (flag_systick_update_depth){
			MS5803_updateDigital(MS5803_CONVERT_PRESSURE);
			current_pressure = MS5803_getPressure();
			depth = ((current_pressure - surface_pressure)*10000)/((int32_t)(RHO_POOL*G_STAVANGER));
			if(depth < 0) depth = 0;
			uint16_t pressure_temp = (uint16_t) MS5803_getTemperature();
			int_temp = ADC_getInternalTemperature();
			CAN_transmitDepthTemp((int16_t)depth, int_temp, 0, pressure_temp);
			flag_systick_update_depth = 0;
		}

		/* Transmit duty cycle to thrusters. */
		if (flag_systick_transmit_thrust){
			int16_t* controller_vals = Interface_readController();
			Interface_transmitManualThrust();
			flag_systick_transmit_thrust = 0;
		}
	} // end while
} // end main

