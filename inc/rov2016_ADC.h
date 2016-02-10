/**
  **************************************************************************************
  * @file    ADC_metoder.h
  * @author  Sivert Sliper, Stian Soerensen
  * @version V01
  * @date    03-February-2016
  * @brief   This file contains macros and extern function prototypes for ADC_metoder.c
  **************************************************************************************
  */

/* Macro -------------------------------------------------------------------------------*/
/* Macros for selecting ADC-channel in ADC_getChannel().*/
#define ADC_CHANNEL_AN_IN1														0B000001
#define ADC_CHANNEL_INT_TEMP													0B000010
#define ADC_CHANNEL_LEAKAGE														0B000100
#define ADC_CHANNEL_AN_IN2														0B001000
#define ADC_CHANNEL_CUR_IN1														0B010000
#define ADC_CHANNEL_CUR_IN2														0B100000

/* Extern function prototypes ----------------------------------------------------------*/
extern void ADC_init(void);
extern uint8_t ADC_getValues(void);
extern uint16_t ADC_getChannel(uint8_t channel);
