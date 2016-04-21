/* Includes ------------------------------------------------------------------*/
#include "stm32f30x_gpio.h"
#include "rov2016_canbus.h"
#include "rov2016_Interface.h"
#include "rov2016_Accelerometer.h"
#include "rov2016_SPI.h"


/* GLOBAL VARIABLES ----------------------------------------------------------*/
#include "def_global_vars.h"

/* PRIVATE VARIABLES ---------------------------------------------------------*/
static float mx=0.0, my=0.0, mz=0.0;
static float heading = 0.0f, pitch=0.0f, roll=0.0f;
static int32_t depth = 0;
static uint16_t int_temp = 0;
static int16_t ax=0, ay=0, az=0;
static int32_t surface_pressure = 9800;
static int32_t current_pressure = 0;

/* Funtion Prototypes --------------------------------------------------------*/
void init(void);

/* Funtion Definitions -------------------------------------------------------*/
int main(void){
	/* Initialization *********************************************************/
	init();
	GPIOE->ODR = 0xFF00; // Turn off LED's
	fmi_topside_xbox_ctrl = CAN_addRxFilter(TOP_XBOX_CTRLS);
	fmi_topside_xbox_axes = CAN_addRxFilter(TOP_XBOX_AXES);


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

			mx = (float) magnetometer_getData(MAGNETOMETER_X_AXIS);
			my = (float) magnetometer_getData(MAGNETOMETER_Y_AXIS);
			mz = (float) magnetometer_getData(MAGNETOMETER_Z_AXIS);

			/* Sensor axes -> rov-axes.*/
			int16_t temp = ax;
			ax = ay;
			ay = az;
			az = temp;

			float temp2 = mx;
			mx = my;
			my = mz;
			mz = temp2;

			pitch = AHRS_accelerometer_pitch(ax, ay, az);
			roll = AHRS_accelerometer_roll(ay, az);

			if(!flag_systick_update_heading){
			CAN_transmitAHRS((int16_t)(pitch*10), (int16_t)(roll*10), (int16_t)(heading*10), \
				(uint16_t)(heading*10));
			}

			flag_systick_update_attitude = 0;
		}

		/* Calculate heading and transmit pitch, roll, heading to topside. */
		if(flag_systick_update_heading){
			heading = AHRS_tilt_compensated_heading(pitch, roll, mx, my, mz);
			CAN_transmitAHRS((int16_t)(pitch*10), (int16_t)(roll*10), (int16_t)(heading*10), \
				(uint16_t)(heading*10));
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
			depth = ((current_pressure - surface_pressure)*9823)/(10*1000);
			uint16_t pressure_temp = (uint16_t) MS5803_getTemperature();
			int_temp = ADC_getInternalTemperature();
			CAN_transmitDepthTemp((uint16_t)depth, int_temp, 0, pressure_temp);
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

