/*
 * dma.c
 *
 * Contains function definitions required for DMA initialization and operations required by the application.
 */

#include "dma.h"
#include "rcc_clock_defs.h"

/**
 * Initializes DMA2 Stream 2 for SPI1 RX (Receive).
 * Configures the stream for medium priority, enabling direct mode and transfer error interrupts.
 * @param channel The DMA channel associated with SPI1 RX.
 */
static void dma2_stream2_init(dma_channel_e channel)
{
  /* DMA2 Stream 2 configuration for FRAM SPI1 RX */
  // DMA_SxCR_DMEIE:  Direct Mode error interrupt enable
  // DMA_SxCR_TEIE:   Transfer error interrupt enable
  // DMA_SxCR_PL_0:   Priority level 01 = medium
  DMA2_Stream2->CR |= (DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_PL_0);
  DMA2_Stream2->CR &= ~(DMA_SxCR_CHSEL);                // Clear the channel settings
  DMA2_Stream2->CR |= (channel << DMA_SxCR_CHSEL_Pos);  // Specify the channel
  NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
 * Initializes DMA2 Stream 3 for SPI1 TX (Transmit).
 * Configures the stream for high priority, memory-to-peripheral direction,
 * enabling direct mode and transfer error interrupts to ensure Tx completion.
 * @param channel The DMA channel associated with SPI1 TX.
 */
static void dma2_stream3_init(dma_channel_e channel)
{
  /* DMA2 Stream 3 configuration for FRAM SPI1 TX */
  // DMA_SxCR_DMEIE:  Direct Mode error interrupt enable
  // DMA_SxCR_TEIE:   Transfer error interrupt enable
  // DMA_SxCR_DIR_0:  Transfer direction to peripheral
  // DMA_SxCR_PL_1:   Priority level 10 = high, ensure that the Tx clock is generated before receiving
  DMA2_Stream3->CR |= (DMA_SxCR_DMEIE | DMA_SxCR_TEIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL_1);
  DMA2_Stream3->CR &= ~(DMA_SxCR_CHSEL);                // Clear the channel settings
  DMA2_Stream3->CR |= (channel << DMA_SxCR_CHSEL_Pos);  // Specify the channel
  NVIC_EnableIRQ(DMA2_Stream3_IRQn);
}

void dma_init(void)
{
  RCC_DMA2_CLK_ENABLE();

  dma2_stream2_init(DMA_CHANNEL_3); // SPI1 RX for FRAM
  dma2_stream3_init(DMA_CHANNEL_3); // SPI1 TX for FRAM
}

error_t dma_wait_for_stream_disabled(DMA_Stream_TypeDef *stream)
{
  error_t status = ERR_OK;

  TickType_t dma_timeout = DMA_TIMEOUT_TICKS;

  // Get the starting tick count and wait until the EN bit is "0"
  TickType_t start_tick_count = xTaskGetTickCount();

  while ((stream->CR & DMA_SxCR_EN) == DMA_SxCR_EN)
  {
    if ((xTaskGetTickCount() - start_tick_count) >= dma_timeout)
    {
      status = ERR_TIMEOUT;
      break;
    }
  }

  return status;
}

void dma_configure_stream(DMA_Stream_TypeDef *stream,
                          uint32_t *mem_addr,
                          volatile uint32_t *periph_addr,
                          uint16_t data_length,
                          dma_minc_e mem_inc_enable)
{
  stream->M0AR = (uint32_t)mem_addr;
  stream->PAR = (uint32_t)periph_addr;
  stream->NDTR = data_length;
  mem_inc_enable ? (stream->CR |= (DMA_SxCR_MINC)) : (stream->CR &= ~(DMA_SxCR_MINC));
}




