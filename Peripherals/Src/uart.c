/*
 * uart.c
 *
 * Provides UART initialization and transmitting data using interrupt and semaphore synchronization.
 * Includes support for receiving Modbus-specific frames with synchronization using task notifications.
 */

#include "uart.h"
#include "modbus_slave.h"
#include "modbus_slave_task.h"
#include "rcc_clock_defs.h"
#include "semphr.h"

/**
 * Array of semaphore handles for UART synchronization.
 * Used to synchronize UART operations (e.g., transmission) with their ISR, ensuring proper timing and sequence.
 * The array size matches the number of supported UART instances.
 */
static SemaphoreHandle_t uart_semaphore_handle[UART_SEMAPHORE_ARRAY_SIZE] = {NULL};

/**
 * Retrieves the semaphore handle for a given UART instance.
 * Maps a UART instance to its semaphore for synchronizing ISR and transmit operations.
 * @param usart_instance UART instance.
 * @return Semaphore handle for the specified UART, or NULL if unrecognized.
 */
static SemaphoreHandle_t uart_get_semaphore_handle(USART_TypeDef *usart_instance)
{
  uint8_t usart_index;

  switch ((uint32_t)usart_instance)
  {
      case (uint32_t)USART1:
          usart_index = uart1;
          break;

      case (uint32_t)USART2:
          usart_index = uart2;
          break;

      case (uint32_t)USART3:
          usart_index = uart3;
          break;

      case (uint32_t)UART4:
          usart_index = uart4;
          break;

      case (uint32_t)UART5:
          usart_index = uart5;
          break;

      case (uint32_t)USART6:
          usart_index = uart6;
          break;

      default:
          return NULL;  // return NULL directly if the USART instance is not recognized
  }

  // Check that the index is within the valid range
  if (usart_index < sizeof(uart_semaphore_handle) / sizeof(uart_semaphore_handle[0]))
  {
    return uart_semaphore_handle[usart_index];
  }

  return NULL;
}

/**
 * Enables the TXE interrupt and waits for the TXE flag to be set.
 * Synchronizes UART transmission by waiting for the semaphore released in the ISR.
 * @param usart_instance UART instance.
 * @return ERR_OK on success, ERR_TIMEOUT on semaphore timeout.
 */
static error_t uart_synch_interrupt_txe(USART_TypeDef *usart_instance)
{
  error_t status = ERR_OK;

  // Enable TXE interrupt
  uart_enable_txe_interrupt(usart_instance);

  // Wait until TXE is set
  if (xSemaphoreTake(uart_get_semaphore_handle(usart_instance), UART_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Enables the TC interrupt and waits for the TC flag to be set.
 * Ensures UART transmission is complete by waiting for the semaphore released in the ISR.
 * @param usart_instance UART instance.
 * @return ERR_OK on success, ERR_TIMEOUT on semaphore timeout.
 */
static error_t uart_synch_interrupt_tc(USART_TypeDef *usart_instance)
{
  error_t status = ERR_OK;

  // Enable TC interrupt
  uart_enable_tc_interrupt(usart_instance);

  // Wait until TC is set
  if (xSemaphoreTake(uart_get_semaphore_handle(usart_instance), UART_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Sets the baud rate for the specified USART instance.
 *
 * Configures the Baud Rate Register (BRR) based on the peripheral clock (PCLK) frequency,
 * desired baud rate, and oversampling mode (by 8 or 16). Refer to the formula on page 808
 * of the STM32 reference manual for detailed calculations.
 *
 * The BRR value is derived by dividing the PCLK by the desired baud rate multiplied by the
 * oversampling factor. The result is split into mantissa and fraction parts for BRR formatting.
 *
 * @param uart_instance The USART instance to configure.
 * @param baudrate Desired baud rate in bits per second (bps).
 */
static void uart_set_baudrate(USART_TypeDef *uart_instance, uint32_t baudrate)
{
    // Get the PCLK value corresponding to the USART instance
    uint32_t pclk = uart_get_pclk(uart_instance);

    // Check if oversampling is by 8 or 16
    uint8_t over8 = uart_instance->CR1 & USART_CR1_OVER8;
    uint32_t oversampling = 8 * (2 - over8);              // This will be 8 if OVER8 is set, 16 otherwise

    // Calculate mantissa and fraction parts
    uint32_t mantissa = pclk / (baudrate * oversampling);
    uint32_t fraction = ((pclk * 16) / (baudrate * oversampling)) - (mantissa * 16);

    // Combine mantissa and fraction parts and write to BRR
    uart_instance->BRR = (mantissa << 4) | fraction;
}

/**
 * Configures the baud rate, word length, parity, and stop bits for a USART instance.
 * @param usart_instance The USART instance to configure.
 * @param baudrate Desired baud rate in bits per second (bps).
 * @param parity Parity mode (None, Even, or Odd).
 * @param word_len Word length (data bits per frame: 8 or 9 bits).
 * @param stop_bits Number of stop bits (One or Two).
 */
static void usart_set_comm_options(USART_TypeDef *usart_instance,
                                    uint32_t baudrate,
                                    uart_parity_e parity,
                                    uart_wordlen_e word_len,
                                    uart_stop_bits_e stop_bits)
{
  // Set baudrate
  uart_set_baudrate(usart_instance, baudrate);

  // Set parity
  uart_set_parity(usart_instance, parity);

  // Set word length
  uart_set_word_length(usart_instance, word_len);

  // Set number of stop bits
  uart_set_stop_bits(usart_instance, stop_bits);
}

void uart_init(uart_instance_e uart_instance)
{
  switch (uart_instance)
  {
    case uart1:
      // Add initialization code for UART1 here
      break;

    case uart2:
      // Create the Binary Semaphore for UART interrupt synchronization
      uart_semaphore_handle[uart2] = xSemaphoreCreateBinary();
      configASSERT(uart_semaphore_handle[uart2] != NULL);

      // Enable the UART2 clock
      RCC_USART2_CLK_ENABLE();

      // Enable the UART transmitter, receiver and enable the module
      uart_enable_tx(USART2);
      uart_enable_rx(USART2);
      uart_enable(USART2);

      // Set communication parameters
      usart_set_comm_options(USART2, UART_BAUDRATE_9600, UART_PARITY_EVEN, UART_WORDLEN_9BIT, UART_STOPBITS_ONE);

      // Enable idle line detection interrupt
      uart_enable_idle_interrupt(USART2);

      // Enable IRQ in the NVIC
      NVIC_EnableIRQ(USART2_IRQn);
      break;

    case uart3:
      // Add initialization code for UART3 here
      break;

    case uart4:
      // Add initialization code for UART4 here
      break;

    case uart5:
      // Add initialization code for UART5 here
      break;

    case uart6:
      // Add initialization code for UART6 here
      break;

    default:
      // Handle unknown uart_instance values here
      break;
  }
}

error_t uart_transmit_bytes(USART_TypeDef *uart_instance, uint8_t *data_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;

  if (uart_instance == NULL || data_buffer == NULL)
  {
    return ERR_INVALID_PARAM;
  }

  // Continue only if the uartx semaphore handle is not NULL
  if (uart_get_semaphore_handle(uart_instance))
  {
    uint16_t tx_count = data_length;
    uint16_t tx_byte_num = 0;

    while (tx_count)
    {
      // Wait for TXE
      if (uart_synch_interrupt_txe(uart_instance) != ERR_OK)
      {
        status = ERR_TIMEOUT;
        break;
      }

      // Send byte and clear the TXE flag
      uart_instance->DR = data_buffer[tx_byte_num++];

      tx_count--;
    }

    // If all bytes sent, check for transmission complete
    if (status == ERR_OK)
    {
      if (uart_synch_interrupt_tc(uart_instance) != ERR_OK)
      {
          status = ERR_TIMEOUT;
      }
    }
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

/**
 * USART2 interrupt handler for the Modbus Slave.
 *
 * This handler manages:
 *  - Error conditions (Overrun, Noise, Framing, Parity) by clearing the corresponding flags.
 *  - Data reception by handling RXNE interrupts and storing received bytes in the Modbus buffers.
 *  - Idle line detection to trigger the Modbus Slave task for processing received frames.
 *  - Data transmission by handling TXE and TC interrupts and signaling completion through semaphores.
 *
 * Synchronization between the ISR and FreeRTOS tasks is achieved using lightweight task notifications
 * and semaphores to minimize ISR overhead.
 */

void USART2_IRQHandler(void)
{
  BaseType_t higher_priority_task_woken = pdFALSE;

  /* Prioritize and handle errors */
  if (USART2->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
  {
    if (UART2_ORE_ERROR)
    {
      // Handle overrun error
      (void)USART2->DR;  // Clear the ORE flag
    }

    if (UART2_NE_ERROR)
    {
      // Handle noise error
      (void)USART2->DR;  // Clear the NE flag
    }

    if (UART2_FE_ERROR)
    {
      // Handle framing error
      (void)USART2->DR;  // Clear the FE flag
    }

    if (UART2_PE_ERROR)
    {
      // Handle parity error
      (void)USART2->DR;  // Clear the PE flag
    }

    // Exit ISR after handling errors
    portYIELD_FROM_ISR(higher_priority_task_woken);
    return;
  }

  ///////////////////////////////
  //// Handle data reception ////
  ///////////////////////////////
  /* Check for RXNE */
  if (uart_is_rxne_interrupt_enabled(USART2))
  {
    if (UART2_RXNE_SET)
    {
      if (modbus_buffers.rx_byte_num < sizeof(modbus_buffers.rx_data))
      {
        modbus_buffers.rx_data[modbus_buffers.rx_byte_num++] = (uint8_t)USART2->DR;
      }
      else
      {
        // Handle buffer overrun scenario, perhaps reset rx_byte_num or log an error.
      }
    }
  }

  /**
   * Check for UART IDLE line detection and trigger the Modbus Slave task to process the frame.
   * This is achieved via a lightweight task notification, ensuring minimal ISR overhead.
   */
  if (uart_is_idle_interrupt_enabled(USART2))
  {
    if (UART2_IDLE_SET)
    {
      // Clear the IDLE flag by reading the status and data registers to
      volatile uint32_t tmpreg = USART2->SR;  // Read status register
      tmpreg = USART2->DR;                    // Read data register to clear IDLE flag
      (void)tmpreg;

      // Disable RXNE interrupts until processed by the Modbus Slave task
      uart_disable_rxne_interrupt(USART2);

      // Let the Modbus task know
      modbus_slave_task_send_notification_from_isr(&higher_priority_task_woken);
    }
  }

  //////////////////////////////////
  //// Handle data transmission ////
  //////////////////////////////////
  /* Check for TXE */
  if (uart_is_txe_interrupt_enabled(USART2))
  {
    if (UART2_TXE_SET)
    {
      // Disable TXE interrupts
      uart_disable_txe_interrupt(USART2);

      // Give the semaphore and set xHigherPriorityTaskWoken to pdTRUE
      xSemaphoreGiveFromISR(uart_semaphore_handle[uart2], &higher_priority_task_woken);
    }
  }

  /* Check for TC */
  if (uart_is_tc_interrupt_enabled(USART2))
  {
    if (UART2_TC_SET)
    {
      // Clear the TC flag
      USART2->SR &= ~(USART_SR_TC);

      // Disable TC interrupts
      uart_disable_tc_interrupt(USART2);

      // Give the semaphore and set xHigherPriorityTaskWoken to pdTRUE
      xSemaphoreGiveFromISR(uart_semaphore_handle[uart2], &higher_priority_task_woken);
    }
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}




