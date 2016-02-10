/* Includes ----------------------------------------------------------------------------*/
#include "stm32f30x.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_misc.h"
#include "extern_decl_global_vars.h"

/* Private macros ----------------------------------------------------------------------*/
#define RX_BUFFER_SIZE						10

/* External function prototypes  -------------------------------------------------------*/
extern void USART_init(void);
extern void USART_transmit(uint8_t data);
extern void USART2_IRQHandler(void);
extern void USART_timestamp_transmit(uint8_t timestamp);
extern void USART_datalog_transmit(uint8_t header, uint16_t data);

