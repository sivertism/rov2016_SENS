/* Includes ------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "rov2016_canbus.h"
#include "rov2016_Interface.h"
#include "rov2016_Accelerometer.h"
#include "rov2016_Gyroscope.h"
#include "rov2016_SPI.h"
#include "rov2016_ADC.h"
#include <math.h>
#include <stdint.h>


/* GLOBAL VARIABLES ----------------------------------------------------------*/
#include "def_global_vars.h"

/* Macro -------------------------------------------------------------------------------*/

/* Water density */
#define RHO_POOL										996
#define RHO_SEA											1025

/* Gravitational acceleration */
#define G_STAVANGER										9.824f
#define G_HOUSTON										9.792f


void init(void);

/* PRIVATE VARIABLES ---------------------------------------------------------*/
static int32_t mx=0, my=0, mz=0;
static int16_t ax=0, ay=0, az=0;
static int16_t gx=0, gy=0, gz=0;
static int32_t heading = 0, pitch=0, roll=0;
static uint16_t int_temp=0, DCDC_temp=0, manip_temp=0;

static int32_t surface_pressure = 9985;
static int32_t current_pressure = 0;


/* Funtion Definitions -------------------------------------------------------*/
int main(void){
	/* Initialization *********************************************************/
	init();
	GPIOE->ODR = 0xFF00; // Turn off LED's
	fmi_topside_xbox_ctrl = CAN_addRxFilter(TOP_XBOX_CTRLS);
	fmi_topside_xbox_axes = CAN_addRxFilter(TOP_XBOX_AXES);
	fmi_topside_sens_ctrl = CAN_addRxFilter(TOP_SENS_CTRL);
	fmi_topside_reg_param1 = CAN_addRxFilter(TOP_REG_PARAM1);
	fmi_topside_reg_param2 = CAN_addRxFilter(TOP_REG_PARAM2);

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

		/* Calculate attitude and transmit via CAN-bus */
		if (flag_systick_update_attitude){
			accelerometer_updateValue();
			magnetometer_updateValue();
			gyroscope_updateValue();

			ax = accelerometer_getRawData(ACCELEROMETER_X_AXIS);
			ay = accelerometer_getRawData(ACCELEROMETER_Y_AXIS);
			az = accelerometer_getRawData(ACCELEROMETER_Z_AXIS);

			mx = magnetometer_getData(MAGNETOMETER_X_AXIS);
			my = magnetometer_getData(MAGNETOMETER_Y_AXIS);
			mz = magnetometer_getData(MAGNETOMETER_Z_AXIS);

			gx = gyroscope_getRaw(GYROSCOPE_X_AXIS);
			gy = gyroscope_getRaw(GYROSCOPE_Y_AXIS);
			gz = gyroscope_getRaw(GYROSCOPE_Z_AXIS);

			/* Sensor axes -> rov-axes.*/
			int16_t temp = ax;
			ax = -az;
			az = -ay;
			ay = temp;

			float temp2 = mx;
			mx = -mz;
			mz = -my;
			my = temp2;

			int16_t temp3 = gx;
			gx = -gz;
			gy = -gy;
			gz = -temp3;

			attitude = AHRS_sensor_fusion(ax, ay, az, gx, gy, gz);

			/* Send magnetometer, gyroscope and accelerometer data over CAN */
			uint8_t acc_array[6] = {(uint8_t)(ax >> 8), (uint8_t)(ax & 0xFF),
					(uint8_t)(ay >> 8), (uint8_t)(ay & 0xFF),
					(uint8_t)(az >> 8), (uint8_t)(az & 0xFF)};
			CAN_transmitAcceleration(acc_array);
			CAN_transmitMag(mx, my, mz);
			CAN_transmitGyro(gx, gy, gz);
			int16_t totThrust = Interface_getTotalDuty();
			if(!flag_systick_update_heading){
				CAN_transmitAHRS((int16_t)(-attitude[0]/10), (int16_t)(attitude[1]/10), totThrust,
						(uint16_t)(heading/10));
			}
			flag_systick_update_attitude = 0;
		}

		/* Calculate heading and transmit pitch, roll, heading to topside. */
		if(flag_systick_update_heading){
			heading = AHRS_tilt_compensated_heading(attitude[0], attitude[1], mx, my, mz);
			CAN_transmitAHRS((int16_t)(-attitude[0]/10), (int16_t)(attitude[1]/10), 0,
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

			CAN_transmitDepthTemp(depth, pressure_temp);
			flag_systick_update_depth = 0;
		}

		/* Update and transmit temperature measurements */
		if(flag_systick_update_temp){
			DCDC_temp = ADC_getTemperature(TEMP_DCDC);
			int_temp = ADC_getTemperature(TEMP_INT);
			manip_temp = ADC_getTemperature(TEMP_MANIP);
			CAN_transmitTemp(int_temp, manip_temp, DCDC_temp);
			flag_systick_update_temp = 0;
		}

		/* Transmit duty cycle to thrusters. */
		if (flag_systick_transmit_thrust){
			regulateThrusters();

			if(!flag_systick_auto){
				int16_t* controller_vals = Interface_readController();
				Interface_transmitManualThrust();
				Interface_transmitThrustToMatlab();
			}
			flag_systick_transmit_thrust = 0;
		}

		/* Transmit leak status */
		if(flag_systick_update_leak){
			uint16_t leak_status = ADC_getLeakStatus();
			uint8_t transmitBuffer[2] = {(uint8_t)(leak_status & 0xFF), (uint8_t)(leak_status >> 8)};
			CAN_transmitBuffer(SENSOR_LEAKAGE_ALARM, transmitBuffer , 2, CAN_ID_TYPE_STD);
			flag_systick_update_leak = 0;
		}

	} // end while
} // end main

