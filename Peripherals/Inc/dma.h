/*
 * dma.h
 *
 * Provides definitions, static inline functions and function prototypes for initializing and managing DMA.
 */

#ifndef INC_DMA_H_
#define INC_DMA_H_

#include "error.h"
#include "FreeRTOSTasks.h"
#include "mcu.h"

/**
 * Defines the available DMA channels for configuration and use.
 */
typedef enum
{
  DMA_CHANNEL_0 = 0,
  DMA_CHANNEL_1,
  DMA_CHANNEL_2,
  DMA_CHANNEL_3,
  DMA_CHANNEL_4,
  DMA_CHANNEL_5,
  DMA_CHANNEL_6,
  DMA_CHANNEL_7
} dma_channel_e;

/**
 * Used for enabling or disabling memory increment in DMA transfers.
 * When enabled, the memory address is incremented after each data transfer.
 */
typedef enum
{
  DMA_MINC_DISABLE = 0,
  DMA_MINC_ENABLE
} dma_minc_e;

/**
 * DMA timeout in ms.
 */
#define DMA_TIMEOUT_MS                10UL

/**
 * DMA timeout in FreeRTOS ticks.
 */
#define DMA_TIMEOUT_TICKS             pdMS_TO_TICKS(DMA_TIMEOUT_MS)

/**
 * Enables the specified DMA stream, setting the EN bit in the DMA stream's control register.
 * @param Stream the specified DMA1 or DMA2 stream (0 - 7).
 */
static inline void dma_enable_stream(DMA_Stream_TypeDef *stream)
{
  stream->CR |= (DMA_SxCR_EN);
}

/**
 * Disables the specified DMA stream.
 * Should be followed by a check to ensure the stream is fully disabled.
 * @param Stream the specified DMA1 or DMA2 stream (0 - 7).
 * @note Follow this function with DMA_Stream_IsDisabled().
 */
static inline void dma_disable_stream(DMA_Stream_TypeDef *stream)
{
  stream->CR &= ~(DMA_SxCR_EN);
}

/**
 * Enables the Transfer Complete interrupt for a specified DMA stream.
 * This interrupt is triggered when a DMA transfer is successfully completed.
 * @param stream The specified DMA1 or DMA2 stream (0 - 7).
 */
static inline void dma_enable_transfer_complete_interrupt(DMA_Stream_TypeDef *stream)
{
  stream->CR |= (DMA_SxCR_TCIE);
}

/**
 * Disables the Transfer Complete interrupt for a specified DMA stream.
 * Use this to stop receiving interrupt notifications when a DMA transfer completes.
 * @param stream The specified DMA1 or DMA2 stream (0 - 7).
 */
static inline void dma_disable_transfer_complete_interrupt(DMA_Stream_TypeDef *stream)
{
  stream->CR &= ~(DMA_SxCR_TCIE);
}

/**
 * Initializes DMA streams for specific system peripherals (SPI1 RX/TX).
 * This includes enabling the DMA2 clock and configuring each stream for optimal operation with its peripheral.
 */
void dma_init(void);

/**
 * Waits for a DMA stream to be fully disabled within a timeout.
 *
 * Checks the EN bit in the stream configuration register and returns
 * ERR_OK if the stream is disabled, or ERR_TIMEOUT on timeout.
 *
 * @param stream The DMA stream to check.
 * @return ERR_OK if disabled, ERR_TIMEOUT otherwise.
 */
error_t dma_wait_for_stream_disabled(DMA_Stream_TypeDef *stream);

/**
 * Configures a DMA stream for data transfer.
 *
 * Sets memory and peripheral addresses, data length, and memory increment settings.
 *
 * @param stream The DMA stream to configure.
 * @param mem_addr Base memory address for the transfer.
 * @param periph_addr Peripheral data register address.
 * @param data_length Number of bytes to transfer.
 * @param mem_inc_enable Flag to enable/disable memory address increment.
 */
void dma_configure_stream(DMA_Stream_TypeDef *stream,
                          uint32_t *mem_addr,
                          volatile uint32_t *periph_addr,
                          uint16_t data_length,
                          dma_minc_e mem_inc_enable);


#endif /* INC_DMA_H_ */

