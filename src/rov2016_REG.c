/**
 *****************************************************************************
 * @title   ROV2016_REG main
 * @author  H.Line
 * @date    26 Feb 2016
 * @brief   Main file for ROV regulator
 ********************************
 */

/* Include .h files from other modules here.*/
#include "stm32f30x.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "float.h"
#include <stdio.h>
#include "rov2016_canbus.h"
#include "rov2016_Interface.h"
#include "rov2016_UART.h"
#include "rov2016_SysTick.h"
#include "rov2016_REG.h"
#include "rov2016_SPI.h"



/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Macro--------------------------------------------------------------------------------*/
#define DEBUG_MODE

/* Private Function Prototypes ----------------------------------------------------------*/
static void CalcThrust();
static void matrix_multiply(int32_t* pSrc1, int32_t* pSrc2, int32_t* pDst, uint8_t src1_rows,
		uint8_t src1_columns, uint8_t src2_rows, uint8_t src2_columns);



/* Private variables -------------------------------------------------------------------*/
static int16_t setPReg[6] 	= {0}; 												/* 	Set point for roll, pitch and depth(x 1/1000)*/
static int32_t setPAcc[6] 	= {0};												/* 	Set point for surge, sway and yaw(x 1/1000)*/
static int16_t SensDat[6]	= {0}; 												/* 	Data from sensor node(x 1/1000)*/
static int32_t uP[8] 		= {0}; 												/* 	Thruster gain in uNewton*/
static int32_t mP[8] 		= {0}; 												/* 	Thruster gain in milliNewton*/
static float ThG[8] 		= {0}; 												/* 	Thruster gain -1 to 1 for CAN transmittion*/
static int16_t startbuffer  = {0};												/* 	Buffer in mN before thruster starts to rotate*/
static int16_t *controller_data;												/* 	Joystick data*/
static int32_t *sens_dat;														/* 	Sensor data*/
static int16_t Rflag = 0;
static int16_t Pflag = 0;
static int16_t Hflag = 0;

/* Regulator variables -------------------------------------------------------------------*/
static int32_t Err[6] 		= {0}; 												/*	Error, from set point to sensor value*/
static int32_t ErrOld[6] 	= {0}; 												/*	Last Error, from set point to sensor value*/
static int32_t KpReg[6]		= {0};												/*  Gain from regulator 			(x 1/1000 N)*/
static int32_t KPmax[6] 	= {0};												/*  Maximum axis gain 				(x 1/1000 N)*/
static int32_t ABmax[6] 	= {2830, 2830, 4000, 600 , 640, 710}; 				/*  Maximum axis gain 				(x 1/1000 N)*/
static int16_t Pmax 		= 0;												/*  Maximum thruster gain 			(x 1/1000 N)*/
static int32_t Kp[6]		= {0};

static int32_t IbrakeRot 		= 60000;										/* 	Maximum value from integrator Rot(x 1/1000 N)*/
static int16_t DbrakeRot 		= 60000;										/*	Maximum value from derivate	 Rot(x 1/1000 N)*/

static int16_t IbrakeTr 		= 30000;										/* 	Maximum value from integrator Tra(x 1/1000 N)*/
static int16_t DbrakeTr 		= 30000;										/* 	Maximum value from derivate	 Tra(x 1/1000 N)*/

static int32_t Phi;																/*	Proportional value for height control*/
static int32_t Ppi;																/*	Proportional value for pitch control*/
static int32_t Pro;																/*	Proportional value for roll control*/

static int32_t Ihi;																/*	Integral value for height control*/
static int32_t Ipi;																/*	Integral value for pitch control*/
static int32_t Iro;																/*	Integral value for roll control*/

static int32_t Dhi;																/*	Derivative value for height control*/
static int32_t Dpi;																/*	Derivative value for pitch control*/
static int32_t Dro;																/*	Derivative value for roll control*/

/* Low pass D */
static int32_t depth_k_1 = 0;
static int32_t d_heave;															/*	Derivative value for roll control*/


// Thruster force matrix calculation: PP(8x1) = ABR(8*6) * KP(6x1)
static int32_t ABR [8][6] 	={
		{0,0,250,1667,-1563,0},
		{0,0,250,1667,1563,0},
		{0,0,250,-1667,1563,0},
		{0,0,250,-1667,-1563,0},
		{354,-354,0,0,0,-1407},
		{354,354,0,0,0,-1407},
		{354,-354,0,0,0,1407},
		{354,354,0,0,0,1407}
};

extern void regulateThrusters(){
	Interface_readRegparam();
	controller_data = Interface_readController();

	/*Update sensor mm*/
	SensDat[AXIS_HEAVE] = depth;
	/*Update sensor mrad*/
	SensDat[AXIS_ROLL]  = attitude[1]/10; // decidegrees
	SensDat[AXIS_ROLL]  = SensDat[AXIS_ROLL] * 157/90; //milliradians
	SensDat[AXIS_PITCH] = -attitude[0]/10;
	SensDat[AXIS_PITCH] = SensDat[AXIS_PITCH] * 157/90;

	if(flag_systick_auto){

		//printf("Roll setpoint: %d", controller_data[XBOX_CTRL_ROLL]/2);
		/*Update manual gain*/
		if (controller_data[XBOX_CTRL_ROLL] != 0){
			setPReg[AXIS_ROLL] = -(controller_data[XBOX_CTRL_ROLL]);
			Rflag = 1;
		}else{
			if (Rflag > 0){
				setPReg[AXIS_ROLL] =0; //Sens[AXIS_ROLL];
				Rflag = 0;
			}
		}

		/*Update additional manual gain*/
		if (controller_data[XBOX_CTRL_PITCH] != 0){
			setPReg[AXIS_PITCH] = -(controller_data[XBOX_CTRL_PITCH]);
			Pflag = 1;
		}else{
			if (Pflag > 0){
				setPReg[AXIS_PITCH] = 0;//Sens[AXIS_PITCH];
				Pflag = 0;
			}
		}

		/*Update additional manual gain*/
		if (controller_data[XBOX_CTRL_HEAVE] != 0){
			setPReg[AXIS_HEAVE] = SensDat[AXIS_HEAVE] - (controller_data[XBOX_CTRL_HEAVE]);
			Hflag = 1;
		}else{
			if (Hflag > 0){
				setPReg[AXIS_HEAVE] = SensDat[AXIS_HEAVE];
				Hflag = 0;
			}
		}

		/*Update manual gain*/
		setPAcc[AXIS_SURGE] = -controller_data[XBOX_CTRL_SURGE]*10;
		setPAcc[AXIS_SWAY] 	= controller_data[XBOX_CTRL_SWAY]*10;
		setPAcc[AXIS_YAW]	= (controller_data[XBOX_CTRL_YAWR])-(controller_data[XBOX_CTRL_YAWL]);

		/*Regulate Thruster values*/
		CalcThrust();


	} // end autoreg
	else {
		setPReg[AXIS_HEAVE] = SensDat[AXIS_HEAVE];
	}

	uint8_t setPointBuffer = {(uint8_t)(setPReg[AXIS_HEAVE]&0xFF),
								(uint8_t)(setPReg[AXIS_HEAVE]>>8),
								(uint8_t)(setPReg[AXIS_PITCH]&0xFF),
								(uint8_t)(setPReg[AXIS_PITCH]>>8),
								(uint8_t)(setPReg[AXIS_ROLL]&0xFF),
								(uint8_t)(setPReg[AXIS_ROLL]>>8)};
	CAN_transmitBuffer(SENSOR_REG_SETPOINT, setPointBuffer, 6, CAN_ID_TYPE_STD);
}

extern void CalcThrust(){
	/*
	 * @brief  Calculate the thruster gain with PID regulator
	 */

	// 1: Calculate error
	ErrOld[AXIS_HEAVE] = Err[AXIS_HEAVE];
	ErrOld[AXIS_ROLL] = Err[AXIS_ROLL];
	ErrOld[AXIS_PITCH] = Err[AXIS_PITCH];
	Err[AXIS_HEAVE] = setPReg[AXIS_HEAVE] - SensDat[AXIS_HEAVE];
	Err[AXIS_ROLL] = setPReg[AXIS_ROLL] + SensDat[AXIS_ROLL];
	Err[AXIS_PITCH] = setPReg[AXIS_PITCH] + SensDat[AXIS_PITCH];

	// 2: Calculate thruster gain from PID regulator
	// Depth
	Phi  = -Err[AXIS_HEAVE]*Kp_t;														// Proportional part

	Ihi -= Kp_t*(Ts/Ti_t)*Err[AXIS_HEAVE]/100;											// Integral part
	if (Ihi > IbrakeTr) Ihi = IbrakeTr;
	if (Ihi < IbrakeTr*-1) Ihi = IbrakeTr*-1;

	d_heave = 0.9*depth_k_1 + 0.1*SensDat[AXIS_HEAVE];
	Dhi  = (Kp_t*Td_t*(d_heave-depth_k_1))/(Ts*10);										// Diff. part
	depth_k_1 = d_heave;
	if (Dhi > DbrakeTr) Dhi = DbrakeTr;
	if (Dhi < DbrakeTr*-1) Dhi = DbrakeTr*-1;
	KpReg[AXIS_HEAVE] = ((int32_t) (Phi + Ihi + Dhi)/10);								// Sum all three, mN/mNm

	// Roll
	Pro  = Kp_r*Err[AXIS_ROLL];																// Proportional part
	Iro += Kp_r*(Ts/Ti_r)*Err[AXIS_ROLL]/100;												// Integral part
	if (Iro > IbrakeRot) Iro = IbrakeRot;													// Integral brake
	if (Iro < IbrakeRot*-1) Iro = IbrakeRot*-1;
	Dro  = Kp_r*Ts*(ErrOld[AXIS_ROLL]-Err[AXIS_ROLL])/10000;								// Diff. part
	if (Dro > DbrakeRot) Dro = DbrakeRot;													// Derivate brake
	if (Dro < DbrakeRot*-1) Dro = DbrakeRot*-1;
	KpReg[AXIS_ROLL]  = (int32_t) (Pro + Iro + Dro)/10;										// Sum all three, mN/mNm

	// Pitch
	Ppi = Kp_r * Err[AXIS_PITCH];															// Proportional part
	Ipi += Kp_r * (Ts/Ti_r) * Err[AXIS_PITCH] / 100;										// Integral part
	if (Ipi > IbrakeRot) Ipi = IbrakeRot;
	if (Ipi < IbrakeRot * -1) Ipi = IbrakeRot * -1;
	Dpi  = Kp_r * Ts * (ErrOld[AXIS_PITCH] - Err[AXIS_PITCH]) / 10000;						// Diff. part
	if (Dpi > DbrakeRot) Dpi = DbrakeRot;
	if (Dpi < DbrakeRot*-1) Dpi = DbrakeRot*-1;
	KpReg[AXIS_PITCH]  = (int32_t) (Ppi + Ipi + Dpi)/10;									// Sum all three, mN/mNm

	// 3: Sum up regulator output + setPAcc, and scale. (mN/mNm)
	volatile uint8_t i;
	for (i=0; i<6; i++){
		Kp[i] =  KpReg[i] + setPAcc[i];
		if(Kp[i] > KPmax[i]) Kp[i] = KPmax[i];
		if(Kp[i] < KPmax[i]*-1) Kp[i] = KPmax[i]*-1;
	}

	// 4: Update values in PP
	// mP(6x1) = ABR(8*6) * KP(6x1);
	matrix_multiply(&ABR[0][0],Kp, uP, 8, 6, 6, 1);

	//addAndScale;
	for (i=0; i<8; i++){
		mP[i] = uP[i]/1000;
	}
	for (i=0; i<8; i++){
		if (mP[i] > startbuffer){
			if(mP[i] > Pmax) mP[i] = Pmax;
			ThG[i] =   (((float)mP[i]) * 0.0000277f) + 0.06194f;
		}else if(mP[i] < -startbuffer){
			if(mP[i] < -Pmax) mP[i] = -Pmax;
			ThG[i] =  (((float)mP[i]) * 0.0000250f) - 0.06525f;
		}else{
			ThG[i] = 0.0f;
		}
	}

	/* Send thrust to ESC's. */
	uint8_t p = 0;
	for(p = 0; p < 8; p++){
		VESC_setDutyCycle(p+1, (float) ThG[p]);
	};

	/*** Send duty cycle to Matlab ***/
	int8_t duty_array[8] = {(int8_t)(ThG[0]*100.0f),
			(int8_t)(ThG[1]*100.0f),
			(int8_t)(ThG[2]*100.0f),
			(int8_t)(ThG[3]*100.0f),
			(int8_t)(ThG[4]*100.0f),
			(int8_t)(ThG[5]*100.0f),
			(int8_t)(ThG[6]*100.0f),
			(int8_t)(ThG[7]*100.0f)};
	CAN_transmitBuffer(SENSOR_THRUSTER_DUTY_MAN,duty_array,8, 0);
}

/*Set regulator parameters*/
extern void setUpReg(int16_t timeStamp, int16_t sKp_t, int16_t sTi_t, int16_t sTd_t,
		int16_t sKp_r, int16_t sTi_r, int16_t sTd_r,  int16_t maxThrust){
	/*Set regulator parameters
	 * timeStamp 	= time stamp in milli seconds
	 * sKp_t 		= translational KP value 	(*10)
	 * sTi_t 		= translational TI value 	(*10)
	 * sTd_t 		= translational TD value 	(*10)
	 * sKp_r 		= rotational KP value 		(*10)
	 * sTi_r 		= rotational TI value		(*10)
	 * sTd_r 		= rotational TD value		(*10)
	 * maxThrust 	= maximal thruster values, in Newton
	 *
	 * */

	Ts = timeStamp;

	/*Translational regulator parameters*/
	Kp_t = sKp_t;
	Ti_t = sTi_t;
	Td_t = sTd_t;

	/*Rotational regulator parameters*/
	Kp_r = sKp_r;
	Ti_r = sTi_r;
	Td_r = sTd_r;
	/*Max thrust in axis*/
	Pmax = maxThrust * 1000;
	volatile uint8_t i;
	for (i=0; i<6; i++){
		KPmax[i] = ABmax[i]*maxThrust;

	}
}

static void matrix_multiply(int32_t* pSrc1, int32_t* pSrc2, int32_t* pDst, uint8_t src1_rows,
		uint8_t src1_columns, uint8_t src2_rows, uint8_t src2_columns){

	/* @brief Function for multiplying two matrixes in C.
	 * @param
	 * 1:source M1
	 * 2:source M2
	 * 3:source destination M
	 * 4:number of rows M1
	 * 5:number of columns M1
	 * 6:number of rows M2
	 * 7:number of columns M2
	 * @date 25.04.2016
	 *
	 * By Hartvik and Olav*/

	if(src1_columns != src2_rows){
		/*Matrix dimentions must agree*/
		return;
	}

	int32_t* p1 = pSrc1;
	int32_t* p2 = pSrc2;
	int32_t* p3 = pDst;
	int32_t sum=0, n1, n2;

	uint8_t dst_columns = src2_columns;

	uint8_t i, j, k;

	for(i=0; i<src1_rows; i++){
		for(j=0; j<src2_columns; j++) {
			for(k=0; k<src1_columns; k++){
				n1 = *p1;
				n2 = *p2;
				sum += n1*n2;
				p1 ++;
				p2 += src2_columns;
			}
			// Reset to prev position.
			p1 --;
			p2 -= src2_columns;
			p1 = p1 - src1_columns + 1;
			p2 = p2 - (src2_rows-1)*src2_columns + 1;

			// Last iteration.
			if(j == src2_columns-1) p2 = pSrc2;

			*p3 = sum;
			p3 ++;
			sum = 0;
		}
		p1 = pSrc1 + (i+1)*src1_columns;
		p2 = pSrc2;
		p3 = pDst + (i+1)*dst_columns;
	}

}
