/**
  **************************************************************************************
  * @file    rov2016_GPIO.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains macros and extern function prototypes for rov2016_GPIO.c
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/

/* Extern function prototypes ----------------------------------------------------------*/
extern void GPIO_init(void);
extern void GPIO_leakage_detector_enable(void);
extern void GPIO_leakage_detector_disable(void);
