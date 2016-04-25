/**
 **************************************************************************************
 * @file    rov2016_ADC.c
 * @author  Sivert Sliper, Stian Soerensen
 * @version V1.0
 * @date    3-February-2016
 * @brief   This file contains all the functions for the SPI peripheral.
 **************************************************************************************
 */

/* Include------------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"
#include "rov2016_SPI.h"
#include "stm32f30x_misc.h"

/* Macro -------------------------------------------------------------------------------*/
#define DEBUG_MODE

/* Global variables --------------------------------------------------------------------*/

/* Private function prototypes ---------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/
static uint8_t buffer[16];
static uint8_t buffer_pos = 0;
static uint8_t isValid = 0;
static uint32_t temperature_digital, pressure_digital;
static int32_t dT; // Temperature difference, used for calculating pressure.

/* Sensor coefficients. */
static uint16_t p_sens; 	// C1 - Pressure sensitivity.
static uint16_t p_offset; 	// C2 - Pressure offset.
static uint16_t t_cps; 		// C3 - Temperature Coefficient of Pressure Sensitivity.
static uint16_t t_cpo; 		// C4 - Temperature Coefficient of Pressure offset.
static uint16_t t_ref;	 	// C5 - Referance Temperature.
static uint16_t t_sens;		// C6 - Temperature Coefficient of the temperature.

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	Stores data received from the pressure sensor.
 * @param  	None
 * @retval 	None
 */
void SPI2_IRQHandler(void){
	/* Receive PROM calibration coeffs. */
	if(isValid){
		buffer[buffer_pos++] = SPI_ReceiveData8(SPI2);//SPI2->DR; // Read receive buffer.
	} else {
		uint8_t dummy;
		dummy = SPI_ReceiveData8(SPI2);//SPI2->DR;
	}
	/* Interrupt should be automatically cleared by hardware. */
}

/**
 * @brief  	Initializes the SPI-module with the following key settings:
 * @param  	None
 * @retval 	None
 */
extern void SPI2_Init(void){

	/* Clock init **********************************************************************/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* GPIO Init ***********************************************************************/
	/* PB9:			CS
	 * PB13:		SPI2_SCK
	 * PB14:		SPI2_MISO
	 * PB15:		SPI2_MOSI
	 */
	/* SCK, MISO, MOSI */
	GPIO_InitTypeDef SPI_GPIO_Init;
	SPI_GPIO_Init.GPIO_Mode  = GPIO_Mode_AF;
	SPI_GPIO_Init.GPIO_Speed = GPIO_Speed_Level_1;
	SPI_GPIO_Init.GPIO_OType = GPIO_OType_PP;
	SPI_GPIO_Init.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	SPI_GPIO_Init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &SPI_GPIO_Init);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);

	/* Chip select (CS)*/
	SPI_GPIO_Init.GPIO_Mode = GPIO_Mode_OUT;
	SPI_GPIO_Init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &SPI_GPIO_Init);

	/* SPI Init ************************************************************************/

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Interrupt handler settings */
	NVIC_InitTypeDef NVIC_InitStruct;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannel = SPI2_IRQn;
	NVIC_Init(&NVIC_InitStruct);

	/* RXNE threshold = 8 bit */
	SPI2->CR2 |= SPI_CR2_FRXTH;

	/* Interrupt request enable */
	SPI2->CR2 |= SPI_CR2_RXNEIE;

	/* Initiate CS as high. */
	GPIOB->ODR |= GPIO_Pin_9;

	/* Enable SPI2 */
	SPI2->CR1 |= SPI_CR1_SPE;

	/* Send testbyte */
	//	SPI2_DR_8BIT = 0x12;
}

/**
 * @brief  	Reads the PROM value from the pressure sensor and calculates calibration vals.
 * @param  	None
 * @retval 	None
 */
extern void MS5803_Init(void){

	buffer_pos = 0;

	/* Chip select */
	GPIOB->ODR &= ~GPIO_Pin_9;

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); // Wait for complete transmission.

	//	SPI2_DR_8BIT = (uint8_t)MS5803_RESET; // Reset sensor to load PROM-content.
	SPI_SendData8(SPI2, MS5803_RESET);

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); // Wait for complete transmission.
	/* Wait ~3ms for the calibration values to be loaded to PROM. */
	volatile uint32_t i = 36000;
	while(i-->0);
	/* Chip select */
	GPIOB->ODR |= GPIO_Pin_9;

	for(i=0; i<8; i++){
		/* Chip select */
		GPIOB->ODR &= ~GPIO_Pin_9;
		//		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		/* Send read-commands for the PROM bytes. */
		SPI_SendData8(SPI2, (uint8_t)(MS5803_PROM_READ_BASE + 2*i));

		/* Wait for finished transmission (TX FIFO contains less than 8 bits. */
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));

		/* Send 3 empty bytes to read PROM content. */
		isValid = 1;
		uint8_t null = 0;
		SPI_SendData8(SPI2, null);
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI_SendData8(SPI2, null);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));

		isValid = 0;
		/* Chip select */
		GPIOB->ODR |= GPIO_Pin_9;

	}
	/* Chip select */
	GPIOB->ODR |= GPIO_Pin_9;

	/* Calculate coefficients. */
	p_sens = ((uint16_t)buffer[2] << 8) | buffer[3];
	p_offset = ((uint16_t)buffer[4] << 8) | buffer[5];
	t_cps = ((uint16_t)buffer[6] << 8) | buffer[7];
	t_cpo = ((uint16_t)buffer[8] << 8) | buffer[9];
	t_ref = ((uint16_t)buffer[10] << 8) | buffer[11];
	t_sens = ((uint16_t)buffer[12] << 8) | buffer[13];
}

/**
 * @brief  	Updates the digital temperature value from the MS5803 pressure sensor.
 * @param  	sensor: Specifies the sensor to be read. can be
 * 			MS5803_CONVERT_TEMPERATURE or MS5803_CONVERT_PRESSURE.
 * @retval 	None
 */
extern void MS5803_updateDigital(uint8_t sensor){
	/* Reset buffer index. */
	buffer_pos = 0;

	/* Chip select */
	GPIOB->ODR &= ~GPIO_Pin_9;

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); // Wait for available FIFO.
	SPI_SendData8(SPI2, sensor);

	/* Wait ~9ms for the conversion. */
	volatile uint32_t i = 100000;
	while(i-->0);

	/* Chip select */
	GPIOB->ODR |= GPIO_Pin_9;
	GPIOB->ODR &= ~GPIO_Pin_9;

	SPI_SendData8(SPI2, MS5803_ADC_READ);
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));
	isValid = 1;

	/* Read 24 bits. Data is received through interrupts. */
	SPI_SendData8(SPI2, 0);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_SendData8(SPI2, 0);
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
	SPI_SendData8(SPI2, 0);

	/* Wait for complete transmission. */
	while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_BSY));
	isValid = 0; // Deactivate reading (flag for interrupt routine).

	/* Chip select */
	GPIOB->ODR |= GPIO_Pin_9;

	/* Store values from buffer. */
	if(sensor == MS5803_CONVERT_TEMPERATURE){
		temperature_digital = (uint32_t)(
							  ((uint32_t) buffer[0] << 16)
							| ((uint32_t) buffer[1] << 8)
							| buffer[2]);
	} else {
		pressure_digital = 	(uint32_t)(
							((uint32_t) buffer[0] << 16)
						| 	((uint32_t) buffer[1] << 8)
						| 	buffer[2]);
	}
}

/**
 * @brief  	Calculates and returns the temperature read by the MS5803 sensor.
 * @param  	None
 * @retval 	Temperature in 0.01*C resolution
 */
extern int32_t MS5803_getTemperature(){
	/* Calculations are based on the MS5803 data sheet, page 8. */
	dT = temperature_digital - (uint32_t)(t_ref << 8);
	int32_t temperature = 2000 + (int32_t)(((int64_t)dT*t_sens) >> 23);
	return temperature;
}

/**
 * @brief  	Caution! Should be called after the temperature is initially read.
 * 			Calculates and returns the pressure read by the MS5803 sensor.
 * @param  	None
 * @retval 	Pressure in 0.1 mbar resolution.
 */
extern int32_t MS5803_getPressure(){
	/* Calculations are based on the MS503 data sheet, page 8. */
	int64_t offset = ((uint32_t)p_offset << 16) + (((int64_t)t_cpo*dT)>>7);
	int64_t sens = ((uint32_t)p_sens << 15) + (((int64_t)t_cps*dT) >> 8);
	int32_t pressure = ( (pressure_digital*sens >> 21) - offset) >> 15;
	return pressure;
}
