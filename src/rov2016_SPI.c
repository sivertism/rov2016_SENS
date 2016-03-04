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
static uint8_t PROM_buffer[16];
static uint8_t PROM_buffer_pos = 0;
static uint8_t isDownloadingPROM = 0;
static uint8_t isPressure = 0;
static uint8_t isTemperature = 0;
static uint8_t isValid = 0;
/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	Stores data received from the pressure sensor.
 * @param  	None
 * @retval 	None
 */
void SPI2_IRQHandler(void){
	/* Receive PROM calibration coeffs. */
	if(isDownloadingPROM && isValid){
		PROM_buffer[PROM_buffer_pos] = SPI2->DR; // Read receive buffer.
		printf("Prom byte %d is %d", PROM_buffer_pos, PROM_buffer[PROM_buffer_pos]);
		PROM_buffer_pos++;
	} else {
		uint8_t dummy = SPI2->DR;
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
	/* Idle state:			Low
	 * Capture:				First edge
	 */
	//SPI2->CR1 &= ~(SPI_CR1_SPE); 				// Disable SPI2
	/*SPI_I2S_DeInit(SPI2);

	SPI2->CR1 |=    SPI_BaudRatePrescaler_16	// Baud rate = F_pclk /16
					| SPI_CR1_MSTR				// Master mode.
					| SPI_CR1_SSM;

	SPI2->CR2 |= 	SPI_DataSize_8b 			// Data size = 8 bit.
					| SPI_CR2_RXNEIE			// Enable interrupt on receive.
					| SPI_CR2_FRXTH;			// RXNE threshold = 8bit.
	 */

	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI2, &SPI_InitStructure);

	/* Interrupt handler settings */
		NVIC_InitTypeDef NVIC_InitStruct;
		NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
		NVIC_InitStruct.NVIC_IRQChannel = SPI2_IRQn;
		NVIC_Init(&NVIC_InitStruct);

	/* RXNE threshold = 8 bit */
	SPI2->CR2 |= SPI_CR2_FRXTH;

	/* Interrupt request enable */
	SPI2->CR2 |= SPI_CR2_RXNEIE;


	/* Initiate CS as high. */
	GPIOB->ODR |= GPIO_Pin_9;

	/* Empty contents of receive buffer */
	uint8_t dummy;
	while(SPI2->SR & SPI_SR_RXNE){
		printf("RXNE...\n");
		dummy = SPI2->DR;
	}

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
#ifdef DEBUG_MODE
	printf("Initiating MS5803...\n");
#endif

	isDownloadingPROM  = 1;
	PROM_buffer_pos = 0;

	/* Chip select */
	GPIOB->ODR &= ~GPIO_Pin_9;

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); // Wait for complete transmission.

	SPI2_DR_8BIT = (uint8_t)MS5803_RESET; // Reset sensor to load PROM-content.

	while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE)); // Wait for complete transmission.
	/* Wait ~3ms for the calibration values to be loaded to PROM. */
	volatile uint32_t i = 36000;
	while(i-->0);

	for(i=0; i<8; i++){

		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI2_DR_8BIT = (uint8_t)(MS5803_PROM_READ_BASE+i); // Send read-commands for the PROM bytes.

		/* Wait for finished transmission (TX FIFO contains less than 8 bits. */
		while(SPI_GetTransmissionFIFOStatus(SPI2) > SPI_TransmissionFIFOStatus_Empty);

		/* Send 3 empty bytes to read PROM content. */
		isValid = 1;
		SPI2_DR_8BIT = (uint8_t)0;
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI2_DR_8BIT = (uint8_t)0;
		while(!SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE));
		SPI2_DR_8BIT = (uint8_t)0;
		while(SPI_GetTransmissionFIFOStatus(SPI2) > SPI_TransmissionFIFOStatus_Empty);
		isValid = 0;
	}
	/* Chip select */
	GPIOB->ODR |= GPIO_Pin_9;

	isDownloadingPROM = 0;
}
