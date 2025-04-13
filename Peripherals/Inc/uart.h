/*
 * uart.h
 *
 * Declares function prototypes for UART initialization, interrupt-driven data transmission,
 * and Modbus frame reception, along with related definitions and static-line functions.
 */

#ifndef INC_UART_H_
#define INC_UART_H_

#include "error.h"
#include "FreeRTOSTasks.h"
#include "mcu.h"
#include "rcc.h"

/**
 * Size of the array holding semaphore handles for UART instances.
 * Size is set to 7 to allow direct indexing (i.e., USART1 at index 1,
 * ..., USART6 at index 6). Index 0 is unused.
 * Note: Index 0 is unused as there is no USART0. This is not related
 * to the number of UART instances defined in uart_instance_e enum.
 */
#define UART_SEMAPHORE_ARRAY_SIZE   (7)

/**
 * UART timeout in milliseconds.
 */
#define UART_TIMEOUT_MS     10UL

/**
 * UART timeout in FreeRTOS ticks.
 */
#define UART_TIMEOUT_TICKS  pdMS_TO_TICKS(UART_TIMEOUT_MS)

/**
 * Defines for SR bits used in the UART2 ISR handler.
 */
#define UART2_RXNE_SET    (USART2->SR & USART_SR_RXNE)
#define UART2_TXE_SET     (USART2->SR & USART_SR_TXE)
#define UART2_TC_SET      (USART2->SR & USART_SR_TC)
#define UART2_IDLE_SET    (USART2->SR & USART_SR_IDLE)
#define UART2_ORE_ERROR   (USART2->SR & USART_SR_ORE)
#define UART2_NE_ERROR    (USART2->SR & USART_SR_NE)
#define UART2_FE_ERROR    (USART2->SR & USART_SR_FE)
#define UART2_PE_ERROR    (USART2->SR & USART_SR_PE)

/**
 * Enumerates UART hardware instances for initialization and configuration.
 * Used to map UART peripherals to their synchronization mechanisms and resources.
 *
 * Examples:
 * - Specifies the UART instance to initialize in `uart_init()`.
 * - Maps to semaphore handles in `uart_get_semaphore_handle()`.
 * - Used in ISR for synchronization operations.
 */
typedef enum
{
  uart1 = 1,
  uart2 = 2,
  uart3 = 3,
  uart4 = 4,
  uart5 = 5,
  uart6 = 6
} uart_instance_e;

/**
 * Commonly used UART baud rates.
 */
typedef enum
{
  UART_BAUDRATE_9600  = 9600,
  UART_BAUDRATE_19200 = 19200,
  UART_BAUDRATE_38400 = 38400,
  UART_BAUDRATE_57600 = 57600,
  UART_BAUDRATE_115200 = 115200,
  UART_BAUDRATE_230400 = 230400,
  UART_BAUDRATE_460800 = 460800,
  UART_BAUDRATE_921600 = 921600
} uart_baudrate_e;

/**
 * Word length options for UART communication.
 */
typedef enum
{
  UART_WORDLEN_8BIT = 0,
  UART_WORDLEN_9BIT
} uart_wordlen_e;

/**
 * Parity options for UART communication.
 */
typedef enum
{
  UART_PARITY_NONE = 0,
  UART_PARITY_EVEN,
  UART_PARITY_ODD
} uart_parity_e;

/**
 * Stop bit options for UART communication.
 * @note: 0.5 and 1.5 stop bits are not included.
 */
typedef enum
{
  UART_STOPBITS_ONE  = 0,
  UART_STOPBITS_TWO,
} uart_stop_bits_e;

/**
 * Retrieves the peripheral clock speed for the specified UART instance.
 * @param uart_instance UART instance.
 * @return Clock speed in Hz.
 */
static inline uint32_t uart_get_pclk(USART_TypeDef *uart_instance)
{
  if (uart_instance == USART1 || uart_instance == USART6)
  {
    return rcc_get_pclk2_freq();  // APB2
  }
  else
  {
    return rcc_get_pclk1_freq();  // APB1
  }
}

/**
 * Configures the word length for the UART instance.
 * @param uart_instance UART instance.
 * @param word_len Word length (8 or 9 bits).
 */
static inline void uart_set_word_length(USART_TypeDef *uart_instance, uart_wordlen_e word_len)
{
  if (word_len == UART_WORDLEN_8BIT)
  {
    uart_instance->CR1 &= ~(USART_CR1_M);
  }
  else
  {
    uart_instance->CR1 |= (USART_CR1_M);
  }
}

/**
 * Configures the parity setting for the UART instance.
 * @param uart_instance UART instance.
 * @param parity Parity mode (None, Even, Odd).
 */
static inline void uart_set_parity(USART_TypeDef *uart_instance, uart_parity_e parity)
{
  if (parity == UART_PARITY_NONE)
  {
    uart_instance->CR1 &= ~(USART_CR1_PCE);
  }
  else
  {
    if (parity == UART_PARITY_EVEN)
    {
      uart_instance->CR1 |= (USART_CR1_PCE);
      uart_instance->CR1 &= ~(USART_CR1_PS);
    }
    else
    {
      uart_instance->CR1 |= (USART_CR1_PCE);
      uart_instance->CR1 |= (USART_CR1_PS);
    }
  }
}

/**
 * Configures the number of stop bits for the UART instance.
 * @param uart_instance UART instance.
 * @param stop_bits Number of stop bits (One or Two).
 */
static inline void uart_set_stop_bits(USART_TypeDef *uart_instance, uart_stop_bits_e stop_bits)
{
  if (stop_bits == UART_STOPBITS_ONE)
  {
    uart_instance->CR2 &= ~(USART_CR2_STOP);
  }
  else  // USART_TWO_STOP_BITS
  {
    uart_instance->CR2 |= (USART_CR2_STOP_1);
  }
}

/**
 * Enables the UART instance.
 * @param uart_instance UART instance.
 */
static inline void uart_enable(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_UE);
}

/**
 * Disables the UART instance.
 * @param uart_instance UART instance.
 */
static inline void uart_disable(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_UE);
}

/**
 * Enables the UART transmitter.
 * @param uart_instance UART instance.
 */
static inline void uart_enable_tx(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_TE);
}

/**
 * Disables the UART transmitter.
 * @param usart_instance UART instance.
 */
static inline void uart_disable_tx(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_TE);
}

/**
 * Enables the UART receiver.
 * @param uart_instance UART instance.
 */
static inline void uart_enable_rx(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_RE);
}

/**
 * Disables the UART receiver.
 * @param uart_instance UART instance.
 */
static inline void uart_disable_rx(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_RE);
}

/**
 * Enables the TX empty interrupt.
 * @param uart_instance USART instance.
 */
static inline void uart_enable_txe_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_TXEIE);
}

/**
 * Checks if the TX empty interrupt is enabled.
 * @param uart_instance USART instance.
 */
static inline uint32_t uart_is_txe_interrupt_enabled(USART_TypeDef *uart_instance)
{
  return (((uart_instance->CR1 & USART_CR1_TXEIE) == USART_CR1_TXEIE) ? 1UL : 0UL);
}

/**
 * Disables the TX empty interrupt.
 * @param uart_instance UART instance.
 */
static inline void uart_disable_txe_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_TXEIE);
}

/**
 * Enables the Transmission Complete interrupt.
 * @param uart_instance UART instance.
 */
static inline void uart_enable_tc_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_TCIE);
}

/**
 * Checks if the Transmission Complete (TC) interrupt is enabled for the specified UART instance.
 * @param uart_instance UART instance.
 * @return 1 if TC interrupt is enabled, 0 otherwise.
 */
static inline uint32_t uart_is_tc_interrupt_enabled(USART_TypeDef *uart_instance)
{
  return (((uart_instance->CR1 & USART_CR1_TCIE) == USART_CR1_TCIE) ? 1UL : 0UL);
}

/**
 * Disables the Transmission Complete interrupt.
 * @param uart_instance UART instance.
 */
static inline void uart_disable_tc_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_TCIE);
}

/**
 * Enables the RX not empty interrupt.
 * @param uart_instance UART instance.
 */
static inline void uart_enable_rxne_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_RXNEIE);
}

/**
 * Checks if the RX not empty interrupt is enabled for the specified UART instance.
 * @param uart_instance UART instance.
 * @return 1 if enabled, 0 otherwise.
 */
static inline uint32_t uart_is_rxne_interrupt_enabled(USART_TypeDef *uart_instance)
{
  return (((uart_instance->CR1 & USART_CR1_RXNEIE) == USART_CR1_RXNEIE) ? 1UL : 0UL);
}

/**
 * Disables the RX not empty interrupt.
 * @param uart_instance UART instance.
 */
static inline void uart_disable_rxne_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_RXNEIE);
}

/**
 * Enables UART idle line detection interrupts.
 * @param uart_instance UART instance.
 */
static inline void uart_enable_idle_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 |= (USART_CR1_IDLEIE);
}

/**
 * Checks if the UART idle line detection interrupt is enabled.
 * @param uart_instance UART instance.
 * @return 1 if enabled, 0 otherwise.
 */
static inline uint32_t uart_is_idle_interrupt_enabled(USART_TypeDef *uart_instance)
{
  return (((uart_instance->CR1 & USART_CR1_IDLEIE) == USART_CR1_IDLEIE) ? 1UL : 0UL);
}

/**
 * Disables UART idle line detection interrupts.
 * @param uart_instance UART instance.
 */
static inline void uart_disable_idle_interrupt(USART_TypeDef *uart_instance)
{
  uart_instance->CR1 &= ~(USART_CR1_IDLEIE);
}

/**
 * Initializes the specified UART instance with predefined settings.
 *
 * Sets up synchronization primitives, enables clocks, configures communication
 * parameters (baud rate, word length, parity, and stop bits), initializes interrupts, and
 * provides flexibility for adding unique initialization code for different UART instances.
 *
 * @param uart_instance Enumeration specifying the UART instance to initialize.
 */
void uart_init(uart_instance_e uart_instance);

/**
 * Transmits an array of bytes over UART using interrupt-based synchronization.
 *
 * Uses TXE (Transmit Data Register Empty) and TC (Transmission Complete)
 * interrupts to transmit bytes and confirm the completion of the transmission.
 * A semaphore ensures efficient synchronization between the ISR and the calling context.
 *
 * @param uart_instance The UART peripheral instance to use for transmission.
 * @param data_buffer Pointer to the data array to be transmitted.
 * @param data_length The number of bytes to transmit.
 * @return ERR_OK on successful transmission, ERR_TIMEOUT if TXE or TC flags timeout,
 *         ERR_INVALID_PARAM for null parameters, or ERR_FAIL for other errors.
 */
error_t uart_transmit_bytes(USART_TypeDef *uart_instance, uint8_t *data_buffer, uint16_t data_length);


#endif /* INC_UART_H_ */





