/**
  **************************************************************************************
  * @file    rov2016_Filter.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains macros and extern function prototypes for rov2016_SPI.c
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
#define
/* Macros for the MS5803-14BA pressure sensor */
#define MS5803_RESET					0x1E
#define MS5803_CONVERT_PRESSURE			0x48
#define MS5803_CONVERT_TEMPERATURE		0x58
#define MS5803_ADC_READ					0x00
#define MS5803_PROM_READ_BASE			0xA0

/* Extern function prototypes ----------------------------------------------------------*/
extern void SPI2_Init(void);
extern void MS5803_Init(void);






