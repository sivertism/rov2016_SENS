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

/* Pointer to send 8 bits through SPI2. Using SPIx->DR results in 16 bits being sent. */
#define SPI2_DR_8BIT					*((uint8_t*) 0x4000380D)

/* Macros for the MS5803-14BA pressure sensor */
/* Commands. */
#define MS5803_RESET					0x1E
#define MS5803_CONVERT_PRESSURE			0x48
#define MS5803_CONVERT_TEMPERATURE		0x58
#define MS5803_ADC_READ					0x00
#define MS5803_PROM_READ_BASE			0xA0

/* Extern function prototypes ----------------------------------------------------------*/
extern void SPI2_Init(void);
extern void MS5803_Init(void);
extern void MS5803_updateDigital(uint8_t sensor);
extern int32_t MS5803_getTemperature();
extern int32_t MS5803_getPressure();





