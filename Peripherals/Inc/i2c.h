/*
 * i2c.h
 *
 *  Contains function prototypes required for I2C initialization, transmitting
 *  and receiving data using interrupts.
 */

#ifndef INC_I2C_H_
#define INC_I2C_H_

#include <stdbool.h>

#include "error.h"
#include "FreeRTOSTasks.h"
#include "mcu.h"

/**
 * Size of the array holding semaphore handles for I2C instances.
 * Size is set to 4 to allow direct indexing (i.e., I2C1 at index 1,
 * ..., I2C3 at index 3). Index 0 is unused.
 * @Note Index 0 is unused as there is no I2C0. This is not related
 * to the number of I2C instances defined in I2C_Instance_e enum.
 */
#define I2C_SEMAPHORE_ARRAY_SIZE      (4)

/**
 * I2C timeout in ms.
 */
#define I2C_TIMEOUT_MS                100UL

/**
 * I2C timeout in FreeRTOS ticks.
 */
#define I2C_TIMEOUT_TICKS             pdMS_TO_TICKS(I2C_TIMEOUT_MS)

/**
 * I2C 7-BIT write/read modifiers.
 */
#define I2C_7BIT_ADDR_WRITE(ADDRESS)  ((uint8_t)((ADDRESS) & (~0x01)))
#define I2C_7BIT_ADDR_READ(ADDRESS)   ((uint8_t)((ADDRESS) | (0x01)))

/**
 * Defines for SR bits used in the I2C1 ISR handler.
 */
#define I2C1_SB_SET         (I2C1->SR1 & I2C_SR1_SB)        // Check if Start Bit (SB) is set
#define I2C1_ADDR_SET       (I2C1->SR1 & I2C_SR1_ADDR)      // Check if Address Sent (ADDR) is set
#define I2C1_TXE_SET        (I2C1->SR1 & I2C_SR1_TXE)       // Check if Transmit Data Register Empty (TXE) is set
#define I2C1_BTF_SET        (I2C1->SR1 & I2C_SR1_BTF)       // Check if Byte Transfer Finished (BTF) is set
#define I2C1_RXNE_SET       (I2C1->SR1 & I2C_SR1_RXNE)      // Check if Receive Data Register Not Empty (RXNE) is set

/**
 * Defines for SR1 error bits used in the I2C1 Error ISR handler.
 */
#define I2C1_BERR_ERROR     (I2C1->SR1 & I2C_SR1_BERR)      // Bus Error
#define I2C1_ARLO_ERROR     (I2C1->SR1 & I2C_SR1_ARLO)      // Arbitration Lost Error
#define I2C1_AF_ERROR       (I2C1->SR1 & I2C_SR1_AF)        // Acknowledge Failure Error
#define I2C1_OVR_ERROR      (I2C1->SR1 & I2C_SR1_OVR)       // Overrun/Underrun Error
#define I2C1_TIMEOUT_ERROR  (I2C1->SR1 & I2C_SR1_TIMEOUT)   // Timeout Error
#define I2C1_ALERT_ERROR    (I2C1->SR1 & I2C_SR1_SMBALERT)  // SMBus Alert Error

/**
 * Identifies the specific I2C hardware instance to be initialized and configured.
 * Used in i2c_init() to select the appropriate I2C peripheral (I2C1, I2C2, I2C3)
 * for initialization and setup.
 */
typedef enum
{
  i2c1 = 1,
  i2c2 = 2,
  i2c3 = 3
} i2c_instance_e;

/**
 * I2C1 Synchronization flags.
 * These flags are used to synchronize various stages of I2C communication in interrupt service routines.
 * They are set before enabling the corresponding interrupt and cleared when the condition is met in the ISR.
 * @Note Create flags for other I2C instances as required.
 */
typedef struct
{
  bool I2C1_WAIT_SB;
  bool I2C1_WAIT_ADDR;
  bool I2C1_WAIT_TXE;
  bool I2C1_WAIT_RXNE;
  bool I2C1_WAIT_BTF;
} i2c_synch_flags_t;

/**
 * Enables an I2C instance.
 * @param i2c_instance I2C instance.
 */
static inline void i2c_enable(I2C_TypeDef *i2c_instance)
{
  if ((I2C1->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
  {
    i2c_instance->CR1 |= (I2C_CR1_PE);
  }
}

/**
 * Disables an I2C instance.
 * @param i2c_instance I2C instance.
 */
static inline void i2c_disable(I2C_TypeDef *i2c_instance)
{
  i2c_instance->CR1 &= ~(I2C_CR1_PE);
}

/**
 * Resets the specified I2C instance.
 *
 * This function triggers a software reset to restore the I2C peripheral
 * to its default reset state.
 *
 * @param i2c_instance I2C instance.
 */
static inline void i2c_reset(I2C_TypeDef *i2c_instance)
{
  // Set the SWRST bit to reset I2C
  i2c_instance->CR1 |= (I2C_CR1_SWRST);

  // Clear the SWRST bit to exit the reset state
  i2c_instance->CR1 &= ~(I2C_CR1_SWRST);
}

/**
 * Enables error interrupts for the specified I2C instance.
 *
 * This function enables the generation of an interrupt when an error condition
 * is detected, e.g., bus error, overrun, and arbitration lost.
 * @note See page 783 of the reference manual.
 * @param i2c_instance I2C instance.
 */
static inline void i2c_enable_error_interrupts(I2C_TypeDef *i2c_instance)
{
  // Set the ITERREN bit to enable error interrupts
  i2c_instance->CR2 |= (I2C_CR2_ITERREN);
}

/**
 * Enables events interrupts.
 * @note See page 782 of the reference manual.
 * @param i2c_instance I2C instance.
 */
static inline void i2c_enable_event_interrupts(I2C_TypeDef *i2c_instance)
{
  i2c_instance->CR2 |= (I2C_CR2_ITEVTEN);
}

/**
 * Disables events interrupts.
 * @param i2c_instance I2C instance
 */
static inline void i2c_disable_event_interrupts(I2C_TypeDef *i2c_instance)
{
  i2c_instance->CR2 &= ~(I2C_CR2_ITEVTEN);
}

/**
 * Clears address sent flag.
 * @param i2c_instance I2C instance
 */
static inline void i2c_clear_address_flag(I2C_TypeDef *i2c_instance)
{
  __IO uint32_t tmpreg;
  tmpreg = i2c_instance->SR1; // Read SR1 to clear ADDR flag
  (void) tmpreg;
  tmpreg = i2c_instance->SR2; // Read SR2 to clear ADDR flag
  (void) tmpreg;
}

/**
 * Enables TXE/RXNE interrupt.
 * @note See reference manual page 782.
 * @param i2c_instance I2C instance
 */
static inline void i2c_enable_rx_tx_interrupt(I2C_TypeDef *i2c_instance)
{
  i2c_instance->CR2 |= (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
}

/**
 * Disables TXE/RXNE interrupt.
 * @param i2c_instance I2C instance
 */
static inline void i2c_disable_rx_tx_interrupt(I2C_TypeDef *i2c_instance)
{
  i2c_instance->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);
}

/**
 * Checks if TXE/RXNE interrupts are enabled.
 * @param I2Cx I2C instance
 */
static inline uint32_t i2c_is_rx_tx_interrupt_enabled(I2C_TypeDef *i2c_instance)
{
  return (((i2c_instance->CR2 & (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN)) == (I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN)) ? 1UL : 0UL);
}

/**
 * Initializes I2C as per the application requirements for I2C1, I2C2, and I2C3.
 * @param I2C_Instance_e enumerated I2C instance.
 */
void i2c_init(i2c_instance_e i2c_instance_enum);

/**
 * Transmits in master mode the specified amount of data with interrupt and semaphore synchronization.
 * @param i2c_instance I2C instance.
 * @param slave_address address of the slave.
 * @param tx_buffer pointer to the transmission data buffer.
 * @param data_length amount of data to send.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
error_t i2c_master_transmit(I2C_TypeDef *i2c_instance, uint8_t slave_address, uint8_t *tx_buffer, uint16_t data_length);

/**
 * Receives in master mode the specified amount of data with interrupt and semaphore synchronization.
 * @param i2c_instance I2C instance.
 * @param device_address address of the slave.
 * @param rx_buffer pointer to the receive data buffer.
 * @param data_length amount of data to receive.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
error_t i2c_master_receive(I2C_TypeDef *i2c_instance, uint8_t device_address, uint8_t *rx_buffer, uint16_t data_length);


#endif /* INC_I2C_H_ */
