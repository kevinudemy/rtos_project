/*
 * spi.h
 *
 *  Contains function prototypes and required definitions for SPI initialization, transmitting
 *  and receiving data using interrupts and DMA.
 */

#ifndef INC_SPI_H_
#define INC_SPI_H_

#include "error.h"
#include "mcu.h"

#include "FreeRTOS.h"

/**
 * Size of the array holding semaphore handles for SPI instances.
 * Size is set to 5 to allow direct indexing (i.e., SPI1 at index 1,
 * ..., SPI4 at index 4). Index 0 is unused.
 * Note: Index 0 is unused as there is no SPI0. This is not related
 * to the number of SPI instances defined in spi_instance_e enum.
 */
#define SPI_SEMAPHORE_ARRAY_SIZE      (5)

/**
 * SPI timeout in ms.
 */
#define SPI_TIMEOUT_MS                10UL

/**
 * SPI timeout in FreeRTOS ticks.
 */
#define SPI_TIMEOUT_TICKS             pdMS_TO_TICKS(SPI_TIMEOUT_MS)

/**
 * Defines for SR bits used in the SPI1 ISR handler.
 */
#define SPI1_RXNE_SET       (SPI1->SR & SPI_SR_RXNE)
#define SPI1_TXE_SET        (SPI1->SR & SPI_SR_TXE)
#define SPI1_OVR_SET        (SPI1->SR & SPI_SR_OVR)

/**
 * DMA2 Stream2 (RX) flags, used for SPI1 RX.
 */
#define DMA2_STREAM2_TCIF   (DMA2->LISR & DMA_LISR_TCIF2)  // Transfer complete flag
#define DMA2_STREAM2_DMEIF  (DMA2->LISR & DMA_LISR_DMEIF2) // Direct mode error flag
#define DMA2_STREAM2_TEIF   (DMA2->LISR & DMA_LISR_TEIF2)  // Transfer error flag

// DMA2 Stream3 (TX) Flags, used for SPI1 TX.
#define DMA2_STREAM3_TCIF   (DMA2->LISR & DMA_LISR_TCIF3)  // Transfer complete flag
#define DMA2_STREAM3_DMEIF  (DMA2->LISR & DMA_LISR_DMEIF3) // Direct mode error flag
#define DMA2_STREAM3_TEIF   (DMA2->LISR & DMA_LISR_TEIF3)  // Transfer error flag

/**
 * Identifies the specific SPI hardware instance to be initialized and configured.
 * Used in spi_init() to select the appropriate SPI peripheral (SPI1, SPI2, SPI3, SPI4)
 * for initialization and setup.
 */
typedef enum
{
  spi1 = 1,
  spi2 = 2,
  spi3 = 3,
  spi4 = 4
} spi_instance_e;

/**
 * Enables the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance to be enabled.
 */
static inline void spi_enable(SPI_TypeDef *spi_instance)
{
  if ((spi_instance->CR1 & SPI_CR1_SPE) != SPI_CR1_SPE)
  {
    spi_instance->CR1 |= (SPI_CR1_SPE);
  }
}

/**
 * Disables the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance to be disabled.
 */
static inline void spi_disable(SPI_TypeDef *spi_instance)
{
  spi_instance->CR1 &= ~(SPI_CR1_SPE);
}

/**
 * Enables the transmit buffer empty interrupt for the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_enable_txe_interrupt(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 |= (SPI_CR2_TXEIE);
}

/**
 * Disables the transmit buffer empty interrupt for the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_disable_txe_interrupt(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 &= ~(SPI_CR2_TXEIE);
}

/**
 * Checks if the transmit buffer empty interrupt is enabled for the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 * @return Non-zero if enabled; otherwise, zero.
 */
static inline uint32_t spi_is_enabled_txe_interrupt(SPI_TypeDef *spi_instance)
{
  return ((spi_instance->CR2 & SPI_CR2_TXEIE) ? 1UL : 0UL);
}

/**
 * Enables the receive buffer not empty interrupt for the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_enable_rxne_interrupt(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 |= (SPI_CR2_RXNEIE);
}

/**
 * Disables the receive buffer not empty interrupt for the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_disable_rxne_interrupt(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 &= ~(SPI_CR2_RXNEIE);
}

/**
 * Checks if the receive buffer not empty interrupt is enabled for the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 * @return Non-zero if enabled; otherwise, zero.
 */
static inline uint32_t spi_is_enabled_rxne_interrupt(SPI_TypeDef *spi_instance)
{
  return ((spi_instance->CR2 & SPI_CR2_RXNEIE) ? 1UL : 0UL);
}

/**
 * Clears the overrun error flag for the specified SPI instance.
 * @note Clearing is done by reading the SPI_DR register followed by a read of the SPI_SR register.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_clear_flag_ovr(SPI_TypeDef *spi_instance)
{
  __IO uint32_t tmpreg;
  tmpreg = spi_instance->DR;
  (void) tmpreg;
  tmpreg = spi_instance->DR;
  (void) tmpreg;
}

/**
 * Enables DMA for the Rx buffer of the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_enable_rx_dma(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 |= (SPI_CR2_RXDMAEN);
}

/**
 * Disables DMA for the Rx buffer of the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_disable_rx_dma(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 &= ~(SPI_CR2_RXDMAEN);
}

/**
 * Enables DMA for the Tx buffer of the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void spi_enable_tx_dma(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 |= (SPI_CR2_TXDMAEN);
}

/**
 * Disables DMA for the Tx buffer of the specified SPI instance.
 * @param spi_instance Pointer to the SPI instance.
 */
static inline void SPI_disable_tx_dma(SPI_TypeDef *spi_instance)
{
  spi_instance->CR2 &= ~(SPI_CR2_TXDMAEN);
}

/**
 * Initializes SPI1, SPI2, SPI3, or SPI4 as per application requirements.
 * Creates binary semaphores and configures interrupt synchronization.
 * @param spi_instance_enum SPI instance to initialize.
 */
void spi_init(spi_instance_e spi_instance_enum);

/**
 * Transmits a single byte over SPI with interrupt and semaphore synchronization.
 * @param spi_instance SPI instance for transmission.
 * @param tx_byte Byte to transmit.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
error_t spi_transmit_single_byte(SPI_TypeDef *spi_instance, uint8_t tx_byte);

/**
 * Receives a single byte over SPI by transmitting a dummy byte for clock generation.
 * Uses interrupt and semaphore synchronization.
 * @param spi_instance SPI instance for reception.
 * @param rx_byte_ptr Pointer to store the received byte.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
error_t spi_receive_single_byte(SPI_TypeDef *spi_instance, uint8_t *rx_byte_ptr);

/**
 * Transmits multiple bytes over SPI with interrupt and semaphore synchronization.
 * @param spi_instance SPI instance for transmission.
 * @param tx_buffer Pointer to the transmission buffer.
 * @param data_length Number of bytes to transmit.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
error_t spi_transmit_bytes(SPI_TypeDef *spi_instance, uint8_t *tx_buffer, uint16_t data_length);

/**
 * Receives multiple bytes over SPI by transmitting dummy bytes for clock generation.
 * Uses interrupt and semaphore synchronization.
 * @param spi_instance SPI instance for reception.
 * @param rx_buffer Pointer to the reception buffer.
 * @param data_length Number of bytes to receive.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
error_t spi_receive_bytes(SPI_TypeDef *spi_instance, uint8_t *rx_buffer, uint16_t data_length);

/**
 * Transmits multiple bytes over SPI using DMA with interrupt and semaphore synchronization.
 * Configures DMA streams for TX and dummy RX, ensuring proper clocking and cleanup.
 * @param spi_instance_enum SPI instance for transmission.
 * @param tx_buffer Pointer to the transmission buffer.
 * @param data_length Number of bytes to transmit.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT or ERR_FAIL.
 */
error_t spi_transmit_bytes_dma(spi_instance_e spi_instance_enum, uint8_t *tx_buffer, uint16_t data_length);

/**
 * Receives multiple bytes over SPI using DMA with interrupt and semaphore synchronization.
 * Configures DMA streams for RX and dummy TX for clock generation, ensuring proper cleanup.
 * @param spi_instance_enum SPI instance for reception.
 * @param rx_buffer Pointer to the reception buffer.
 * @param data_length Number of bytes to receive.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT or ERR_FAIL.
 */
error_t spi_receive_bytes_dma(spi_instance_e spi_instance_enum, uint8_t *rx_buffer, uint16_t data_length);


#endif /* INC_SPI_H_ */





