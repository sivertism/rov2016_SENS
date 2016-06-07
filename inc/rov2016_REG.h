/*
  **************************************************************************************
  * @file    rov2016_REG.h
  * @author  Hartvik Line, Olav H. Karstensen
  * @version V01
  * @date    07-Mars-2016
  * @brief   This file contains macros and extern function definitions for
  			 rov2016_REG.c.
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/

/* Axis definition */
#define AXIS_SURGE 					0
#define AXIS_SWAY 					1
#define AXIS_HEAVE 					2
#define AXIS_ROLL 					3
#define AXIS_PITCH 					4
#define AXIS_YAW 					5

/* Xbox axis definition */
#define XBOX_CTRL_SURGE 			3
#define XBOX_CTRL_SWAY 				2
#define XBOX_CTRL_HEAVE 			6
#define XBOX_CTRL_ROLL 				4
#define XBOX_CTRL_PITCH 			5
#define XBOX_CTRL_YAWL 				0
#define XBOX_CTRL_YAWR 				1

/* ID list, RANGE = [0...0x7FF] --------------------------------------------------------*/

/* Exported function prototypes --------------------------------------------------------*/

/* Declare global functions here, preferrably with comments to explain the functions.*/



// Macro -------------------------------------------------------------------------------


extern void regulateThrusters();
extern void setUpReg(int16_t timeStamp, int16_t sKp_t, int16_t sTi_t, int16_t sTd_t,
		int16_t sKp_r, int16_t sTi_r, int16_t sTd_r, int16_t maxThrust);


