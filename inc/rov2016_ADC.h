/**
  **************************************************************************************
  * @file    rov2016_ADC.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains macros and extern function prototypes for rov2016_ADC.c
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
/* Macros for selecting ADC-channel in ADC_getChannel().*/
#define ADC_CHANNEL_INT_TEMP													0
#define ADC_CHANNEL_LEAKAGE														1
#define ADC_CHANNEL_AN_IN2														2
#define ADC_CHANNEL_CUR_IN1														3
#define ADC_CHANNEL_CUR_IN2														4
#define ADC_CHANNEL_AN_IN1														5

/* Extern function prototypes ----------------------------------------------------------*/
extern void ADC_init(void);
extern uint8_t ADC_getValues(void);
extern uint16_t ADC_getChannel(uint8_t channel);
extern uint16_t ADC_getInternalTemperature(void);
