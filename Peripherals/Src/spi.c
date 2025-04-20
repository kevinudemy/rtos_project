/*
 * spi.c
 *
 *  Contains function definitions required for SPI initialization, transmitting
 *  and receiving data using interrupts and DMA with semaphore synchronization.
 */

#include "spi.h"
#include "dma.h"
#include "rcc_clock_defs.h"

#include "semphr.h"

// SPI semaphore handles
static SemaphoreHandle_t spi_semaphore_handle[SPI_SEMAPHORE_ARRAY_SIZE] = {NULL};

/**
 * Gets the Semaphore handle for an SPI instance.
 * @param spi_instance SPI intance.
 * @return spi_semaphore_handle, otherwise NULL.
 */
static SemaphoreHandle_t spi_get_semaphore_handle(SPI_TypeDef *spi_instance)
{
  uint8_t spi_index;

  switch ((uint32_t)spi_instance)
  {
      case (uint32_t)SPI1:
          spi_index = spi1;
          break;

      case (uint32_t)SPI2:
          spi_index = spi2;
          break;

      case (uint32_t)SPI3:
          spi_index = spi3;
          break;

      case (uint32_t)SPI4:
          spi_index = spi4;
          break;

      default:
          return NULL;  // return NULL directly if the SPI instance is not recognized
  }

  // Check that the index is within the valid range
  if (spi_index < sizeof(spi_semaphore_handle) / sizeof(spi_semaphore_handle[0]))
  {
    return spi_semaphore_handle[spi_index];
  }

  return NULL;
}

/**
 * Waits for the TXE (Transmit Buffer Empty) flag for the specified SPI instance.
 * Returns ERR_TIMEOUT if the flag is not set within the specified timeout.
 *
 * @param spi_instance SPI instance.
 * @return ERR_OK if successful, ERR_TIMEOUT otherwise.
 */
static error_t spi_wait_for_txe(SPI_TypeDef *spi_instance)
{
  error_t status = ERR_OK;

  TickType_t spi_timeout = SPI_TIMEOUT_TICKS;
  TickType_t start_tick;

  // Get the starting tick count and wait until the Tx buffer is empty
  start_tick = xTaskGetTickCount();
  while ((spi_instance->SR & SPI_SR_TXE) != SPI_SR_TXE)
  {
    if ((xTaskGetTickCount() - start_tick) >= spi_timeout)
    {
      status = ERR_TIMEOUT;
      break;
    }
  }

  return status;
}

/**
 * Waits for the BSY (Busy) flag to clear for the specified SPI instance.
 * Returns ERR_TIMEOUT if the flag remains set beyond the timeout period.
 *
 * @param spi_instance SPI instance.
 * @return ERR_OK if successful, ERR_TIMEOUT otherwise.
 */
static error_t spi_wait_for_bsy_clear(SPI_TypeDef *spi_instance)
{
  error_t status = ERR_OK;

  TickType_t spi_timeout = SPI_TIMEOUT_TICKS;
  TickType_t start_tick;

  // Get the starting tick count and wait until the Tx buffer is empty
  start_tick = xTaskGetTickCount();
  while ((spi_instance->SR & SPI_SR_BSY) == SPI_SR_BSY)
  {
    if ((xTaskGetTickCount() - start_tick) >= spi_timeout)
    {
      status = ERR_TIMEOUT;
      break;
    }
  }

  return status;
}

/**
 * Enables the TXE interrupt and waits for the TXE flag to be set using a semaphore.
 * Returns ERR_TIMEOUT if the flag is not set within the timeout.
 *
 * @param spi_instance SPI instance.
 * @return ERR_OK if successful, ERR_TIMEOUT otherwise.
 */
static error_t spi_synch_txe_interrupt(SPI_TypeDef *spi_instance)
{
  error_t status = ERR_OK;

  // Enable TXE interrupt
  spi_enable_txe_interrupt(spi_instance);

  // Wait until TXE is set
  if (xSemaphoreTake(spi_get_semaphore_handle(spi_instance), SPI_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Enables the RXNE interrupt and waits for the RXNE flag to be set using a semaphore.
 * Returns ERR_TIMEOUT if the flag is not set within the timeout.
 *
 * @param spi_instance SPI instance.
 * @return ERR_OK if successful, ERR_TIMEOUT otherwise.
 */
static error_t spi_synch_rxne_interrupt(SPI_TypeDef *spi_instance)
{
  error_t status = ERR_OK;

  // Enable RXNE interrupt
  spi_enable_rxne_interrupt(spi_instance);

  // Wait until RXNE is set
  if (xSemaphoreTake(spi_get_semaphore_handle(spi_instance), SPI_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Disables DMA resources for SPI1 to prevent conflicts and ensure stability.
 *
 * Deactivates SPI1 RX and TX DMA streams, waits for completion, and resets
 * the SPI1 DMA interface for safe reuse or power-down.
 *
 * @param spi_instance Pointer to the SPI1 instance.
 */
static void spi1_disable_dma_resources(SPI_TypeDef *spi_instance)
{
    // Disable DMA streams used for SPI1 RX and TX
    dma_disable_stream(DMA2_Stream2); // Used for SPI1 RX
    dma_disable_stream(DMA2_Stream3); // Used for SPI1 TX

    // Wait for streams to be effectively disabled
    dma_wait_for_stream_disabled(DMA2_Stream2);
    dma_wait_for_stream_disabled(DMA2_Stream3);

    // Disable SPI1 DMA interface to ensure proper reset of DMA settings
    spi_disable(spi_instance);          // Disable the SPI1 peripheral
    spi_disable_rx_dma(spi_instance);   // Disable SPI1 RX DMA
    SPI_disable_tx_dma(spi_instance);   // Disable SPI1 TX DMA
}

void spi_init(spi_instance_e spi_instance_enum)
{
  switch (spi_instance_enum)
  {
    case spi1:
      // Create the Binary Semaphore for SPI1/DMA interrupt synchronization
      spi_semaphore_handle[spi1] = xSemaphoreCreateBinary();
      configASSERT(spi_semaphore_handle[spi1] != NULL);

      // Enable the SPI1 clock
      RCC_SPI1_CLK_ENABLE();

      /* SPI1 configuration for FRAM */
      // SPI_CR1_MSTR:  Master configuration
      // SPI_CR1_SSI:   Internal slave select (for use with SSM)
      // SPI_CR1_SSM:   Software slave select management
      // SPI_CR1_BR_0:  fPCLK/4 --> see RCC_Init() for accurate adjustment
      // SPI_CR2_ERRIE:   Error interrupt enable
      SPI1->CR1 = (SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_CR1_BR_0);
      SPI1->CR2 = (SPI_CR2_ERRIE);

      // Enable SPI1
      spi_enable(SPI1);

      // Enable IRQ in the NVIC
      NVIC_EnableIRQ(SPI1_IRQn);
      break;

    case spi2:
      // Add initialization code for SPI2 here
      break;

    case spi3:
      // Add initialization code for SPI3 here
      break;

    case spi4:
      // Add initialization code for SPI4 here
      break;

    default:
      // Handle unknown SPI_Instance values here
      break;
  }
}

error_t spi_transmit_single_byte(SPI_TypeDef *spi_instance, uint8_t tx_byte)
{
  error_t status = ERR_OK;

  spi_enable(spi_instance);

  // Continue only if the SPIx Semaphore instance is not NULL
  if (spi_get_semaphore_handle(spi_instance))
  {
    // Temporary variable used for clearning the RXNE flag
    uint8_t temp_read;

    // Send the byte
    *(__IO uint8_t *)&spi_instance->DR = tx_byte;

    // Wait for TXE
    if (spi_synch_txe_interrupt(spi_instance) != ERR_OK)
    {
      status = ERR_TIMEOUT;
    }

    if ((spi_instance->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
    {
      // Clear the RXNE flag by reading the DR and capture data
      temp_read = *(uint8_t *)&spi_instance->DR;
    }

    (void)temp_read; // Avoid compiler warning for unused variable
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t spi_receive_single_byte(SPI_TypeDef *spi_instance, uint8_t *rx_byte_ptr)
{
  error_t status = ERR_OK;

  spi_enable(spi_instance);

  // Continue only if the SPIx semaphore instance is not NULL
  if (spi_get_semaphore_handle(spi_instance))
  {
    // Dummy byte for triggering SPI clock
    uint8_t tx_data = 0xFF;

    // Send dummy data
    *(__IO uint8_t *)&spi_instance->DR = tx_data;

    // Wait for RXNE
    if (spi_synch_rxne_interrupt(spi_instance) != ERR_OK)
    {
      status = ERR_TIMEOUT;
    }
    else
    {
      // Capture data in Rx FIFO
      *rx_byte_ptr = *((__IO uint8_t *)&(spi_instance->DR));
    }
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t spi_transmit_bytes(SPI_TypeDef *spi_instance, uint8_t *tx_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;

  spi_enable(spi_instance);

  // Continue only if the SPIx semaphore handle is not NULL
  if (spi_get_semaphore_handle(spi_instance))
  {
    // Temporary variable used for clearing the RXNE flag
    uint8_t temp_read;

    uint16_t tx_count = data_length;
    uint16_t tx_byte_num = 0;

    // Send the data
    while (tx_count)
    {
      // Send byte and clear the TXE flag
      *(__IO uint8_t *)&spi_instance->DR = tx_buffer[tx_byte_num++];

      // Wait for TXE
      if (spi_synch_txe_interrupt(spi_instance) != ERR_OK)
      {
        status = ERR_TIMEOUT;
        break;
      }

      if ((spi_instance->SR & SPI_SR_RXNE) == SPI_SR_RXNE)
      {
        // Clear the RXNE flag by reading DR and capture data
        temp_read = *(uint8_t *)&spi_instance->DR;
      }

      tx_count--;
    }

    (void)temp_read;  // Avoid compiler warning for unused variable
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t spi_receive_bytes(SPI_TypeDef *spi_instance, uint8_t *rx_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;

  spi_enable(spi_instance);

  // Continue only if the SPIx semaphore handle is not NULL
  if (spi_get_semaphore_handle(spi_instance))
  {
    // Dummy byte
    uint8_t tx_data = 0xFF;

    uint16_t rx_count = data_length;
    uint16_t rx_byte_num = 0;

    // Receive the data
    while (rx_count)
    {
      // Send dummy data
      *(__IO uint8_t *)&spi_instance->DR = tx_data;

      // Wait for RXNE
      if (spi_synch_rxne_interrupt(spi_instance) != ERR_OK)
      {
        status = ERR_TIMEOUT;
        break;
      }

      // Capture data in the RX FIFO and clear the RXNE flag
      rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(spi_instance->DR));

      rx_count--;
    }
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t spi_transmit_bytes_dma(spi_instance_e spi_instance_enum, uint8_t *tx_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;
  uint8_t *dummy_rx;

  switch (spi_instance_enum)
  {
    case spi1: /* FRAM */
      if (spi_semaphore_handle[spi1])
      {
        spi_enable(SPI1);

        // Set Tx source, Enable TC, enable stream3 and Tx SPI DMA enable for data transmission
        dma_configure_stream(DMA2_Stream3, (uint32_t*)tx_buffer, &(SPI1->DR), data_length, DMA_MINC_ENABLE);
        dma_enable_transfer_complete_interrupt(DMA2_Stream3);
        dma_enable_stream(DMA2_Stream3);
        spi_enable_tx_dma(SPI1);

        // Set Rx source, disable TC, enable stream2 and Rx SPI DMA enable for dummy recieves (prevent OVR)
        dma_configure_stream(DMA2_Stream2, (uint32_t*)&dummy_rx, &(SPI1->DR), data_length, DMA_MINC_DISABLE);
        dma_disable_transfer_complete_interrupt(DMA2_Stream2);
        dma_enable_stream(DMA2_Stream2);
        spi_enable_rx_dma(SPI1);

        // Non-blocking wait for the SPI transmit to finish (see DMA2 Stream3 ISR)
        if (xSemaphoreTake(spi_get_semaphore_handle(SPI1), SPI_TIMEOUT_TICKS) != pdTRUE)
        {
          status = ERR_TIMEOUT;
        }

        // Wait for all data to be transmitted (TXE)
        if (status == ERR_OK && (SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
        {
            status = spi_wait_for_txe(SPI1);
        }

        // Wait for SPI not busy (BSY)
        if (status == ERR_OK && (SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY)
        {
            status = spi_wait_for_bsy_clear(SPI1);
        }

        // Disable streams (pg. 223 reference manual - see warning)
        spi1_disable_dma_resources(SPI1);
      }
      else
      {
        status = ERR_FAIL;
      }
      break;

    case spi2:
      break;
    case spi3:
      break;
    case spi4:
      break;

    default:
      status = ERR_INVALID_PARAM;
      break;
  }

  return status;
}

error_t spi_receive_bytes_dma(spi_instance_e spi_instance_enum, uint8_t *rx_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;
  uint8_t dummy_byte = 0;

  switch (spi_instance_enum)
  {
    case spi1: /* FRAM */
      if (spi_semaphore_handle[spi1])
      {
        spi_enable(SPI1);

        // Set the Rx source, enable TC, enable stream2 and Rx DMA enable for receiving
        dma_configure_stream(DMA2_Stream2, (uint32_t*)rx_buffer, &(SPI1->DR), data_length, DMA_MINC_ENABLE);
        dma_enable_transfer_complete_interrupt(DMA2_Stream2);
        dma_enable_stream(DMA2_Stream2);
        spi_enable_rx_dma(SPI1);

        // Set the Tx source, disable TC, enable stream3 and Tx SPI DMA enable for dummy transmission
        dma_configure_stream(DMA2_Stream3, (uint32_t*)&dummy_byte, &(SPI1->DR), data_length, DMA_MINC_DISABLE);
        dma_disable_transfer_complete_interrupt(DMA2_Stream3);
        dma_enable_stream(DMA2_Stream3);
        spi_enable_tx_dma(SPI1);

        // Non-blocking wait for the SPI transmit to finish (see DMA2 Stream2 ISR)
        if (xSemaphoreTake(spi_get_semaphore_handle(SPI1), SPI_TIMEOUT_TICKS) != pdTRUE)
        {
          status = ERR_TIMEOUT;
        }

        // Wait for TXE
        if (status == ERR_OK && (SPI1->SR & SPI_SR_TXE) != SPI_SR_TXE)
        {
            status = spi_wait_for_txe(SPI1);
        }

        // Wait for SPI not busy (BSY)
        if (status == ERR_OK && (SPI1->SR & SPI_SR_BSY) == SPI_SR_BSY)
        {
            status = spi_wait_for_bsy_clear(SPI1);
        }

        // Disable streams (pg. 223 reference manual - see warning)
        spi1_disable_dma_resources(SPI1);
      }
      else
      {
        status = ERR_FAIL;
      }
      break;

    case spi2:
      break;
    case spi3:
      break;
    case spi4:
      break;

    default:
      status = ERR_INVALID_PARAM;
      break;
  }

  return status;
}

/**
 * SPI1 Interrupt Service Routine.
 *
 * Handles SPI1 interrupts for non-blocking SPI operations and SPI errors and
 * performs the following:
 * - RXNE Handling: Gives a semaphore when the receive buffer is not empty, indicating data is received.
 * - TXE Handling: Gives a semaphore when the transmit buffer is empty, indicating ready for next transmission.
 * - Overrun Error Handling: Clears overrun error flag to maintain SPI communication integrity.
 */
void SPI1_IRQHandler(void)
{
  portBASE_TYPE higher_priority_task_woken = pdFALSE;

  /* Check for RXNE */
  if (spi_is_enabled_rxne_interrupt(SPI1))
  {
    if (SPI1_RXNE_SET)
    {
      // Disable RXNE interrupts
      spi_disable_rxne_interrupt(SPI1);

      // Give the SPI1 semaphore to the SPI Receive function
      xSemaphoreGiveFromISR(spi_semaphore_handle[spi1], &higher_priority_task_woken);
    }
  }

  /* Check for TXE */
  if (spi_is_enabled_txe_interrupt(SPI1))
  {
    if (SPI1_TXE_SET)
    {
      // Disable TXE interrupts
      spi_disable_txe_interrupt(SPI1);

      // Give SPI1 semaphore to the SPI Transmit function
      xSemaphoreGiveFromISR(spi_semaphore_handle[spi1], &higher_priority_task_woken);
    }
  }

  /* Check if Overrun ocurred */
  if (SPI1_OVR_SET)
  {
    // Clear overrun flag
    spi_clear_flag_ovr(SPI1);
  }

  /* Immediately switch to the higher priority task */
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * DMA2 Stream2 Interrupt Service Routine for SPI1 RX.
 *
 * Handles the completion of data reception for SPI1 via DMA2 Stream2 and
 * performs the following:
 * - Clears the relevant DMA interrupt flags.
 * - Gives a semaphore to signal the completion of SPI1 receive operation using DMA.
 * - Handles direct mode and transfer errors by clearing the respective flags.
 * @note Stream3 interrupt flag is also cleared here as it's used for dummy writes.
 */
void DMA2_Stream2_IRQHandler(void)
{
  portBASE_TYPE higher_priority_task_woken = pdFALSE;

  /* Handle FRAM RX */
  if (DMA2_STREAM2_TCIF)
  {
    // Clear the Stream2 interrupt flag
    DMA2->LIFCR = DMA_LIFCR_CTCIF2;

    // Let's clear the Stream3 interrupt flag here as we're using Stream3 for dummy writes
    if (DMA2_STREAM3_TCIF)
    {
      DMA2->LIFCR = DMA_LIFCR_CTCIF3;
    }

    // Disable interrupts
    dma_disable_transfer_complete_interrupt(DMA2_Stream2);

    // Give the SPI1 Semaphore to the SPI Receive Bytes DMA function
    xSemaphoreGiveFromISR(spi_semaphore_handle[spi1], &higher_priority_task_woken);
  }

  /* Check for direct mode error */
  if (DMA2_STREAM2_DMEIF)
  {
    // Clear the direct mode error flag
    DMA2->LIFCR = DMA_LIFCR_CDMEIF2;
  }

  /* Check for transfer error */
  if (DMA2_STREAM2_TEIF)
  {
    // Clear the transfer error flag
    DMA2->LIFCR = DMA_LIFCR_CTEIF2;
  }

  /* Immediately switch to the higher priority task */
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * DMA2 Stream3 Interrupt Service Routine for SPI1 TX.
 *
 * Manages the completion of data transmission for SPI1 via DMA2 Stream3 and
 * performs the following:
 * - Clears DMA interrupt flags for the stream.
 * - Gives a semaphore to indicate the completion of SPI1 transmit operation using DMA.
 * - Handles direct mode and transfer errors by clearing the respective flags.
 * Note: Stream2 interrupt flag is also cleared to prevent overruns, as mentioned in the reference manual.
 */
void DMA2_Stream3_IRQHandler(void)
{
  portBASE_TYPE higher_priority_task_woken = pdFALSE;

  /* Handle FRAM TX */
  if (DMA2_STREAM3_TCIF)
  {
    // Clear the Stream3 interrupt flag
    DMA2->LIFCR = DMA_LIFCR_CTCIF3;

    // Also clear the Stream2 interrupt flag here as we're using Rx to prevent overruns (page 861 ref. manual)
    if (DMA2_STREAM2_TCIF)
    {
      DMA2->LIFCR = DMA_LIFCR_CTCIF2;
    }

    // Disable interrupts
    dma_disable_transfer_complete_interrupt(DMA2_Stream3);

    // Give the SPI1 Semaphore to the SPI Receive Bytes DMA function
    xSemaphoreGiveFromISR(spi_semaphore_handle[spi1], &higher_priority_task_woken);
  }

  /* Check for direct mode error */
  if (DMA2_STREAM3_DMEIF)
  {
    // Clear the direct mode error flag
    DMA2->LIFCR = DMA_LIFCR_CDMEIF3;
  }

  /* Check for transfer error */
  if (DMA2_STREAM3_TEIF)
  {
    // Clear the transfer error flag
    DMA2->LIFCR = DMA_LIFCR_CTEIF3;
  }

  /* Immediately switch to the higher priority task */
  portYIELD_FROM_ISR(higher_priority_task_woken);
}




