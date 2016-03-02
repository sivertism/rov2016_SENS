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


/* Global variables --------------------------------------------------------------------*/

/* Private function prototypes ---------------------------------------------------------*/

/* Private variables -------------------------------------------------------------------*/
static uint8_t PROM_buffer[16];
static uint8_t PROM_buffer_pos = 0;
static uint8_t isDownloadingPROM = 0;
static uint8_t isPressure = 0;
static uint8_t isTemperature = 0;
/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	Stores data received from the pressure sensor.
 * @param  	None
 * @retval 	None
 */
void SPI2_IRQHandler(void){
	/* Receive PROM calibration coeffs. */
	if(isDownloadingPROM){
		PROM_buffer[PROM_buffer_pos] = (uint8_t)SPI2->DR; // Read receive buffer.
		PROM_buffer_pos++;
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
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_7);

	/* Chip select (CS)*/
	SPI_GPIO_Init.GPIO_Mode = GPIO_Mode_OUT;
	SPI_GPIO_Init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOB, &SPI_GPIO_Init);

	/* SPI Init ************************************************************************/
	/* Idle state:			Low
	 * Capture:				First edge
	 */
	SPI2->CR1 &= ~(SPI_CR1_SPE); // Disable SPI2
	SPI2->CR1 |= 	(uint32_t)(
					0b011000		 // Baud rate = F_pclk /16
					| SPI_CR1_MSTR);  // Master mode
	SPI2->CR2 |= 	(uint16_t)((0b0111)<<8) // Data size = 8 bit.
					| SPI_CR2_RXNEIE;		// Enable interrupt on receive.

	/* Enable SPI2 */
	SPI2->CR1 |= SPI_CR1_SPE;
}

/**
 * @brief  	Reads the PROM value from the pressure sensor and calculates calibration vals.
 * @param  	None
 * @retval 	None
 */
extern void MS5803_Init(void){
	isDownloadingPROM  = 1;
	PROM_buffer_pos = 0;

	SPI2->DR = MS5803_RESET; // Reset sensor to load PROM-content.
	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); // Wait for complete transmission.

	/* Wait ~3ms for the calibration values to be loaded to PROM. */
	volatile uint32_t i = 108000;
	while(i-->0);

	for(i=0; i<16; i++){
		SPI2->DR = MS5803_RESET+i; // Send read-commands for the PROM bytes.
		/* Wait for finished transmission (Transmit Empty). */
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE))

		/* Send 3 empty bytes to read PROM content. */
		SPI2->DR = (uint8_t)0;
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI2->DR = (uint8_t)0;
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI2->DR = (uint8_t)0;
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));

//		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE));

	}
	isDownloadingPROM = 0;
}
