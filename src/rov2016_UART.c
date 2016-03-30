/**
  ******************************************************************************
  * @file    UART_metoder.c
  * @author  Sivert Sliper and Stian Sørensen
  * @version V1.0
  * @date    08-February-2016
  * @brief   This file contains all the functions prototypes for the ADC firmware
  *          library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Include---- -------------------------------------------------------------------------*/
#include "rov2016_UART.h"

/* Global variables --------------------------------------------------------------------*/
#include "extern_decl_global_vars.h"

/* Local variables ---------------------------------------------------------------------*/

/* Receive buffer */
static uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
static uint8_t rx_buffer_write_counter = 0;
static uint8_t rx_buffer_read_counter = 0;
static uint8_t new_bytes = 0;
static uint8_t hex2ascii_table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/* Private function prototypes ---------------------------------------------------------*/

/* Function definitions ----------------------------------------------------------------*/

/**
 * @brief  	Receives message from FT232 USB to serial converter. Received messages are
 * 			stored in an emulated FIFO, and the counter new_bytes is incremented.
 * @param  None
 * @retval None
 */
extern void USART2_IRQHandler(void){

	/* Toggle status LED*/
//	GPIOE->ODR ^= UART_RX_LED << 8;

	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		rx_buffer[rx_buffer_write_counter++] = USART_ReceiveData(USART2);
		if(rx_buffer_write_counter >= RX_BUFFER_SIZE) rx_buffer_write_counter = 0;
		new_bytes++;

	}// end if
} // end USART2_IRQHandler()

/**
 * @brief  	Initializes UART1with interrupt on received message. Connected to X8.
 * 			Baudrate:			115200 bit/s
 * 			Word length:		8 bit
 * 			Parity: 			None
 * 			Stop bits:			1
 *
 * @param  None
 * @retval None
 */
extern void USART1_init(void){
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStruct_USART;

	/* Interrupt setup */
	NVIC_InitStruct_USART.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStruct_USART.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct_USART.NVIC_IRQChannelSubPriority = 7;
	NVIC_InitStruct_USART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct_USART);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Clock setup */
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART1, &USART_ClockInitStructure);

	/* UART setup */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity =  USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1, &USART_InitStructure);

	/* Interrupt on receive buffer not empty */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	/* GPIO Config */
	GPIO_InitTypeDef UART_GPIO_Init;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* USART1 Tx (PA9) */
	UART_GPIO_Init.GPIO_Mode  = GPIO_Mode_AF;
	UART_GPIO_Init.GPIO_Speed = GPIO_Speed_Level_1;
	UART_GPIO_Init.GPIO_OType = GPIO_OType_PP;
	UART_GPIO_Init.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	UART_GPIO_Init.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &UART_GPIO_Init);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);

	/* USART2 Rx (PA10) */
	UART_GPIO_Init.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &UART_GPIO_Init);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

	/* Enable USART1 */
	USART_Cmd(USART1, ENABLE);
}


/**
 * @brief  	Initializes UART2 with interrupt on received message. Used for serial
 * 			communication over USB.
 * 			Baudrate:			115200 bit/s
 * 			Word length:		8 bit
 * 			Parity: 			None
 * 			Stop bits:			1
 *
 * @param  None
 * @retval None
 */
extern void USART2_init(void){
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStruct_USART;

	/* Interrupt setup */
	NVIC_InitStruct_USART.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStruct_USART.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct_USART.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct_USART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct_USART);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Clock setup */
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART2, &USART_ClockInitStructure);

	/* UART setup */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity =  USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART2, &USART_InitStructure);

	/* Interrupt on receive buffer not empty */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* GPIO Config */
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* USART2 Tx (PA2) */
	GPIO_InitStructure_UART.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_InitStructure_UART.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_7);

	/* USART2 Rx (PA3) */
	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_7);

	/* Enable USART2 */
	USART_Cmd(USART2, ENABLE);
} // end USART_init()

/**
 * @brief  	Initializes UART3 with interrupt on received message. Connected to X9.
 * 			Baudrate:			115200 bit/s
 * 			Word length:		8 bit
 * 			Parity: 			None
 * 			Stop bits:			1
 *
 * @param  None
 * @retval None
 */
extern void USART3_init(void){
	USART_InitTypeDef USART_InitStructure;
	USART_ClockInitTypeDef  USART_ClockInitStructure;
	NVIC_InitTypeDef NVIC_InitStruct_USART;

	/* Interrupt setup */
	NVIC_InitStruct_USART.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStruct_USART.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct_USART.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct_USART.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct_USART);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* Clock setup */
	USART_ClockStructInit(&USART_ClockInitStructure);
	USART_ClockInit(USART3, &USART_ClockInitStructure);

	/* UART setup */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity =  USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3, &USART_InitStructure);

	/* Interrupt on receive buffer not empty */
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

	/* GPIO Config */
	GPIO_InitTypeDef UART_GPIO_Init;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/* USART3 Tx (PB10) */
	UART_GPIO_Init.GPIO_Mode  = GPIO_Mode_AF;
	UART_GPIO_Init.GPIO_Speed = GPIO_Speed_Level_1;
	UART_GPIO_Init.GPIO_OType = GPIO_OType_PP;
	UART_GPIO_Init.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	UART_GPIO_Init.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOB, &UART_GPIO_Init);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_7);

	/* USART3 Rx (PB11) */
	UART_GPIO_Init.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOB, &UART_GPIO_Init);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_7);

	/* Enable USART3 */
	USART_Cmd(USART3, ENABLE);
}

/**
 * @brief  	Transmits one byte to the USB COM-port.
 * @param  	USARTx: 	USART module to transmit via. x can be 1,2,3.
 * 			data:		One byte to be transmitted.
 * @retval None
 */
extern void USART_transmit(USART_TypeDef* USARTx,uint8_t data){

	/* Toggle status LED*/
//	GPIOE->ODR ^= UART_TX_LED << 8;

	/* Wait for USART_TX not busy */
	while(USART_GetFlagStatus(USARTx, USART_FLAG_BUSY) != RESET);

	/* Send byte */
	USART_SendData(USARTx, (uint8_t) data);

	/* Wait for transmission complete. */
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) == RESET);
} // end USART_transmit()

/**
 * @brief  	Transmits one unsigned 16-bit value via serial USB.
 * 			Used for displaying data on a PC-based logging program.
 * @param 	header:	Header to indicate the content of the sent data.
 * @param	data:	A 16-bit value to be transmitted.
 * @retval None
 */
extern void USART_python_logger_transmit(uint8_t header, uint16_t data){
	/*	1. Transmit header
	 * 	2. Convert and transmit each hex digit of the data coded in ascii.
	 * 	3. Transmit timestamp header
	 * 	4. Transmit timestamp
	 * 	5. Increment timestamp
	 */

	USART_transmit(USART2, header);
	//	USART_transmit((uint8_t)data & 0xFF);
	//	USART_transmit((uint8_t)(data >> 8));
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data >> 12 & 0x000F)])); //(bit 12-15)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data >> 8 & 0x000F)])); // 	(bit 8-11)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data >> 4 & 0x000F)])); // 	(bit 4-7)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data & 0x000F)])); // 		(bit 0-3)
}

/**
 * @brief  	Transmits a series of  4 16-bit integers separated by commas.
 * @param 	None
 * @param	data:	A 16-bit value to be transmitted.
 * @retval None
 */
extern void USART_matlab_visualizer_transmit(int16_t data1, int16_t data2, int16_t data3, int16_t data4){

	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data1 >> 12 & 0x000F)])); //			(bit 12-15)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data1 >> 8 & 0x000F)])); // 	(bit 8-11)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data1 >> 4 & 0x000F)])); // 	(bit 4-7)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data1 & 0x000F)])); // 		(bit 0-3)
	USART_transmit(USART2, ',');

	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data2 >> 12 & 0x000F)])); //(bit 12-15)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data2 >> 8 & 0x000F)])); // 	(bit 8-11)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data2 >> 4 & 0x000F)])); // 	(bit 4-7)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data2 & 0x000F)])); // 		(bit 0-3)
	USART_transmit(USART2, ',');

	USART_transmit(USART2, hex2ascii_table[(uint8_t)(data3 >> 12 & 0x000F)]); //(bit 12-15)
	USART_transmit(USART2, hex2ascii_table[(uint8_t)(data3 >> 8 & 0x000F)]); // 	(bit 8-11)
	USART_transmit(USART2, hex2ascii_table[(uint8_t)(data3 >> 4 & 0x000F)]); // 	(bit 4-7)
	USART_transmit(USART2, hex2ascii_table[(uint8_t)(data3 & 0x000F)]); // 		(bit 0-3)
	USART_transmit(USART2, ',');

	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data4 >> 12 & 0x000F)])); //(bit 12-15)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data4 >> 8 & 0x000F)])); // 	(bit 8-11)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data4 >> 4 & 0x000F)])); // 	(bit 4-7)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(data4 & 0x000F)])); // 		(bit 0-3)
	USART_transmit(USART2, 10);
}

/**
 * @brief  	Transmits a timestamp for PC-based logging.
 * @param  None
 * @retval None
 */

extern void USART_timestamp_transmit(uint8_t timestamp){
	USART_transmit(USART2, 'T');
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(timestamp >> 4 & 0x000F)])); // 	(bit 4-7)
	USART_transmit(USART2, (uint8_t)(hex2ascii_table[(timestamp & 0x000F)])); // 		(bit 0-3)
}

/**
 * @brief  	Returns the number of unread bytes in the emulated UART receive FIFO.
 * @param  None
 * @retval The number of unread bytes in the emulated UART receive FIFO.
 */
extern uint8_t USART_getNewBytes(void){
	return new_bytes;
}

/**
 * @brief  	Returns the number of unread bytes in the emulated UART receive FIFO.
 * @param  None
 * @retval The number of unread bytes in the emulated UART receive FIFO.
 */
extern uint8_t USART_getRxMessage(void){
	if (rx_buffer_read_counter >= RX_BUFFER_SIZE){
		rx_buffer_read_counter = 0;
	}

	new_bytes--;
	return rx_buffer[rx_buffer_read_counter++];
}
