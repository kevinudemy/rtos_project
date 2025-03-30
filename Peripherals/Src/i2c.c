/*
 * i2c.c
 *
 *  Contains function definitions required for I2C initialization, transmitting
 *  and receiving data using interrupt and semaphore synchronization.
 */

#include "i2c.h"
#include "rcc.h"
#include "rcc_clock_defs.h"
#include "semphr.h"

// I2C semaphore handles
static SemaphoreHandle_t i2c_semaphore_handle[I2C_SEMAPHORE_ARRAY_SIZE] = {NULL};

// Initialize synch flags
i2c_synch_flags_t i2c_synch_flags = {0};

/**
 * Gets the Semaphore handle for an I2C instance.
 * @param i2c_instance I2C instance.
 * @return i2c_event_group_handle, otherwise NULL
 */
static SemaphoreHandle_t i2c_get_semaphore_handle(I2C_TypeDef *i2c_instance)
{
  uint8_t i2c_index;

  switch ((uint32_t)i2c_instance)
  {
      case (uint32_t)I2C1:
          i2c_index = i2c1;
          break;

      case (uint32_t)I2C2:
          i2c_index = i2c2;
          break;

      case (uint32_t)I2C3:
          i2c_index = i2c3;
          break;

      default:
          return NULL;  // return NULL directly if the I2C instance is not recognized
  }

  // Check that the index is within the valid range
  if (i2c_index < sizeof(i2c_semaphore_handle) / sizeof(i2c_semaphore_handle[0]))
  {
    return i2c_semaphore_handle[i2c_index];
  }

  return NULL;
}

/**
 * Sets the peripheral clock frequency for the specified I2C instance.
 * @param i2c_instance I2C instance.
 */
static void i2c_set_pclk_freq(I2C_TypeDef *i2c_instance)
{
    // Get the APB1 clock frequency in MHz
    uint32_t i2c_apb1_clock_mhz = (rcc_get_pclk1_freq() / 1000000);

    // Set the PCLK1 frequency in MHz to the FREQ field of the CR2 register
    i2c_instance->CR2 |= (i2c_apb1_clock_mhz << I2C_CR2_FREQ_Pos);
}

/**
 * Calculates the CCR value for standard mode I2C communication based on the provided PCLK1 frequency.
 * This function calculates the CCR value necessary for generating a 100 kHz SCL clock,
 * assuming a 50% duty cycle (high and low times are each 5,000 ns).
 *
 * @note See reference manual (page 791) for the formula used to calculate CCR:
 *       T_high = CCR * T_PCLK1 and T_low = CCR * T_PCLK1
 *       The total period is divided evenly between high and low times for a 100 kHz clock.
 *
 * @param pclk1_freq The frequency of the PCLK1 in Hz.
 * @return The CCR value for standard mode.
 */
static uint16_t i2c_calculate_standard_mode_ccr(uint32_t pclk1_freq)
{
    // The high (or low) time for standard mode I2C is 5,000 ns (for a 100 kHz clock).
    uint32_t scl_half_period_ns = 5000;

    // Convert the PCLK1 frequency (Hz) to a period in nanoseconds (ns)
    uint32_t pclk1_period_ns = (1000000000UL / pclk1_freq);

    // Calculate the CCR value based on the high (or low) time
    return (uint16_t)(scl_half_period_ns / pclk1_period_ns);
}

/**
 * Calculates the TRISE value for standard mode I2C communication based on the provided PCLK1 frequency.
 * TRISE represents the maximum rise time of the SCL signal in standard mode, expressed in PCLK1 clock cycles.
 * This value is used to configure the corresponding register in the I2C peripheral.
 *
 * @note The I2C standard mode rise time is 1000 ns (1 µs), as per the I2C specification.
 *       The TRISE value is calculated as (1000 ns / PCLK1 period in ns) + 1.
 *
 * @param pclk1_freq The frequency of the PCLK1 in Hz.
 * @return The TRISE value for standard mode.
 */
static uint16_t i2c_calculate_standard_mode_trise(uint32_t pclk1_freq)
{
    // Convert the PCLK1 frequency to a period in nanoseconds (ns)
    uint32_t pclk1_period_ns = 1000000000UL / pclk1_freq;

    // The rise time for I2C standard mode is 1000 ns (1 µs).
    // Calculate the number of PCLK1 cycles that fit into the rise time
    uint32_t trise_cycles = 1000 / pclk1_period_ns;

    // Add 1 to the TRISE value as required by the I2C specification
    return (uint16_t)(trise_cycles + 1);
}

/**
 * Waits until the busy flag clears for the specified I2C instance.
 * @param i2c_instance I2C instance.
 */
static error_t i2c_wait_for_busy_clear(I2C_TypeDef *i2c_instance)
{
  error_t status = ERR_OK;

  TickType_t i2c_timeout = I2C_TIMEOUT_TICKS;
  TickType_t start_tick;

  // Get the starting tick count and wait until I2Cx not busy
  start_tick = xTaskGetTickCount();
  while ((i2c_instance->SR2 & I2C_SR2_BUSY) == I2C_SR2_BUSY)
  {
    if ((xTaskGetTickCount() - start_tick) >= i2c_timeout)
    {
      status = ERR_TIMEOUT;
      break;
    }
  }

  return status;
}

/**
 * Enables the event interrupt and pends until the TXE flag is set.
 * @param i2c_instance I2C intance.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_synch_txe_interrupt(I2C_TypeDef *i2c_instance)
{
  error_t status = ERR_OK;

  // Set the TXE synch flag and enable interrupt
  i2c_synch_flags.I2C1_WAIT_TXE = true;
  i2c_enable_rx_tx_interrupt(i2c_instance);

  // Wait until TXE is set
  if (xSemaphoreTake(i2c_get_semaphore_handle(i2c_instance), I2C_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Enables the event interrupt and pends until the RXNE flag is set.
 * @param i2c_instance I2C intance.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_synch_rxne_interrupt(I2C_TypeDef *i2c_instance)
{
  error_t status = ERR_OK;

  // Set the RXNE synch flag and enable interrupt
  i2c_synch_flags.I2C1_WAIT_RXNE = true;
  i2c_enable_rx_tx_interrupt(i2c_instance);

  // Wait on the Semaphore until the RXNE flag is set
  if (xSemaphoreTake(i2c_get_semaphore_handle(i2c_instance), I2C_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Enables the event interrupt and pends until the BTF flag is set.
 * @param i2c_instance I2C intance.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_synch_btf_interrupt(I2C_TypeDef *i2c_instance)
{
  error_t status = ERR_OK;

  // Set the BTF synch flag and enable interrupt
  i2c_synch_flags.I2C1_WAIT_BTF = true;
  i2c_enable_event_interrupts(i2c_instance);

  // Wait until the BTF bit is set
  if (xSemaphoreTake(i2c_get_semaphore_handle(i2c_instance), I2C_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Sends the start condition after enabling the event interrupt and pends on the semaphore.
 * @param i2c_instance I2C intance.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_send_start(I2C_TypeDef *i2c_instance)
{
  error_t status = ERR_OK;

  // Send the start condition
  i2c_instance->CR1 |= (I2C_CR1_START);
  (void)i2c_instance->DR;

  // Set the SB synch flag and enable event interrupts
  i2c_synch_flags.I2C1_WAIT_SB = true;
  i2c_enable_event_interrupts(i2c_instance);

  // Wait until the SB bit is set
  if (xSemaphoreTake(i2c_get_semaphore_handle(i2c_instance), I2C_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Sends the slave address and pends on the semaphore until the ADDR bit is set.
 * @param i2c_instance I2C intance.
 * @param slave_address the device address to communicate with.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_send_slave_address(I2C_TypeDef *i2c_instance, uint8_t slave_address)
{
  error_t status = ERR_OK;

  // Send the slave address with write/read
  i2c_instance->DR = slave_address;

  // Set the ADDR synch flag and enable event interrupts
  i2c_synch_flags.I2C1_WAIT_ADDR = true;
  i2c_enable_event_interrupts(i2c_instance);

  // Wait until the ADDR bit is set
  if (xSemaphoreTake(i2c_get_semaphore_handle(i2c_instance), I2C_TIMEOUT_TICKS) != pdTRUE)
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

/**
 * Sends the I2C start condition and slave address.
 * @param i2c_instance I2C intance.
 * @param slave_address the device address to communicate with.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_start_transmit(I2C_TypeDef *i2c_instance, uint8_t slave_address)
{
  error_t status = ERR_OK;

  // Send START only if the Event Group handle is not NULL
  if (i2c_get_semaphore_handle(i2c_instance))
  {
    // Send the start condition
    if (i2c_send_start(i2c_instance) != ERR_OK)
    {
      status = ERR_TIMEOUT;
    }
    else  /* Send the slave address */
    {
      status = i2c_send_slave_address(i2c_instance, I2C_7BIT_ADDR_WRITE(slave_address));
    }
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

/**
 * Enables acknowledge and sends the start condition and slave address.
 * @param i2c_instance I2C intance.
 * @param slave_address the device address to communicate with.
 * @return ERR_OK if successful, otherwise ERR_TIMEOUT.
 */
static error_t i2c_start_receive(I2C_TypeDef *i2c_instance, uint8_t slave_address)
{
  error_t status = ERR_OK;

  // Send START only if the I2Cx Semaphore handle is not NULL
  if (i2c_get_semaphore_handle(i2c_instance))
  {
    // Enable ACK
    i2c_instance->CR1 |= (I2C_CR1_ACK);

    /* Send the start condition */
    if (i2c_send_start(i2c_instance) != ERR_OK)
    {
      status = ERR_TIMEOUT;
    }
    else  /* Send the slave address */
    {
      status = i2c_send_slave_address(i2c_instance, I2C_7BIT_ADDR_READ(slave_address));
    }
  }
  else
  {
    status = ERR_TIMEOUT;
  }

  return status;
}

void i2c_init(i2c_instance_e i2c_instance_enum)
{
  switch (i2c_instance_enum)
  {
    case i2c1:
      // Create the Binary Semaphore for I2C interrupt synchronization
      i2c_semaphore_handle[i2c1] = xSemaphoreCreateBinary();
      configASSERT(i2c_semaphore_handle[i2c1] != NULL);

      // Configure I2C1 clock
      RCC_I2C1_CLK_ENABLE();

      // Disable I2C1 prior to modifying configuration registers
      i2c_disable(I2C1);

      // Reset I2C
      i2c_reset(I2C1);

      /* I2C CR2 configuration */
      // I2C_CR2_FREQ:    45 MHz
      i2c_set_pclk_freq(I2C1);

      // I2C_CR2_ITERREN: Error interrupts enabled
      i2c_enable_error_interrupts(I2C1);

      // Clock control and TRISE configurations
      uint32_t pclk1_freq = rcc_get_pclk1_freq();
      I2C1->CCR |= (i2c_calculate_standard_mode_ccr(pclk1_freq) << I2C_CCR_CCR_Pos);
      I2C1->TRISE = i2c_calculate_standard_mode_trise(pclk1_freq);

      // Enable I2C
      i2c_enable(I2C1);

      // Enable the required IRQs
      NVIC_EnableIRQ(I2C1_EV_IRQn);
      NVIC_EnableIRQ(I2C1_ER_IRQn);
      break;

    case i2c2:
      // Add initialization code for I2C2 here
      break;

    case i2c3:
      // Add initialization code for I2C3 here
      break;

    default:
      break;
  }
}

error_t i2c_master_transmit(I2C_TypeDef *i2c_instance, uint8_t device_address, uint8_t *tx_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;

  if (i2c_wait_for_busy_clear(i2c_instance) == ERR_OK)
  {
    if (i2c_start_transmit(i2c_instance, device_address) == ERR_OK)
    {
      uint16_t tx_count = data_length;
      uint16_t tx_byte_num = 0;

      // Send the data
      while (tx_count)
      {
        // Wait for TXE
        if (i2c_synch_txe_interrupt(i2c_instance) != ERR_OK)
        {
          status = ERR_TIMEOUT;
          break;
        }

        // Send the byte
        *(__IO uint8_t *)&i2c_instance->DR = tx_buffer[tx_byte_num++];

        tx_count--;

        // Wait for BTF
        if (i2c_synch_btf_interrupt(i2c_instance) != ERR_OK)
        {
          status = ERR_TIMEOUT;
          break;
        }
      }

      // Send STOP condition
      i2c_instance->CR1 |= (I2C_CR1_STOP);
    }
    else
    {
      status = ERR_FAIL;
    }
  }
  else
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t i2c_master_receive(I2C_TypeDef *i2c_instance, uint8_t device_address, uint8_t *rx_buffer, uint16_t data_length)
{
  error_t status = ERR_OK;

  if (i2c_wait_for_busy_clear(i2c_instance) == ERR_OK)
  {
    if (i2c_start_receive(i2c_instance, device_address) == ERR_OK)
    {
      /* Reception scheme */
      if (data_length == 1)
      {
        // Disable ACK
        i2c_instance->CR1 &= ~(I2C_CR1_ACK);

        // Clear the ADDR flag
        i2c_clear_address_flag(i2c_instance);

        // Generate STOP
        i2c_instance->CR1 |= (I2C_CR1_STOP);
      }
      else if (data_length == 2)
      {
        // Disable ACK
        i2c_instance->CR1 &= ~(I2C_CR1_ACK);

        // Enable POS
        i2c_instance->CR1 |= (I2C_CR1_POS);

        // Clear the ADDR flag
        i2c_clear_address_flag(i2c_instance);
      }
      else
      {
        // Enable ACK
        i2c_instance->CR1 |= (I2C_CR1_ACK);

        // Clear ADDR flag
        i2c_clear_address_flag(i2c_instance);
      }

      /* Receive the data */
      uint16_t rx_count = data_length;
      uint16_t rx_byte_num = (uint16_t) 0;

      while (rx_count)
      {
        if (rx_count <= 3U)
        {
          if (rx_count == 1U)  /* Recieve one byte */
          {
            // Wait for RXNE
            if (i2c_synch_rxne_interrupt(i2c_instance) != ERR_OK)
            {
              status = ERR_TIMEOUT;
              break;
            }

            // Read the data
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;
          }
          else if (rx_count == 2U) /* Recieve two bytes */
          {
            // Wait for BTF
            if (i2c_synch_btf_interrupt(i2c_instance) != ERR_OK)
            {
              status = ERR_TIMEOUT;
              break;
            }

            // Generate STOP
            i2c_instance->CR1 |= (I2C_CR1_STOP);

            // Read a byte
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;

            // Read last byte
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;
          }
          else  /* Three bytes to receive */
          {
            // Wait for BTF
            if (i2c_synch_btf_interrupt(i2c_instance) != ERR_OK)
            {
              status = ERR_TIMEOUT;
              break;
            }

            // Disable ACK
            i2c_instance->CR1 &= ~(I2C_CR1_ACK);

            // Read data
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;

            // Wait for BTF
            if (i2c_synch_btf_interrupt(i2c_instance) != ERR_OK)
            {
              status = ERR_TIMEOUT;
              break;
            }

            // Generate STOP
            i2c_instance->CR1 |= (I2C_CR1_STOP);

            // Read another byte
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;

            // Read the last byte
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;
          }
        }
        else /* More than 3 bytes */
        {
          // Wait for RXNE
          if (i2c_synch_rxne_interrupt(i2c_instance) != ERR_OK)
          {
            status = ERR_TIMEOUT;
          }

          // Read the DR
          rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
          rx_count--;

          // Read another byte if BTF is set
          if ((I2C1->SR1 & I2C_SR1_BTF) == I2C_SR1_BTF)
          {
            rx_buffer[rx_byte_num++] = *((__IO uint8_t *)&(i2c_instance->DR));
            rx_count--;
          }
        }
      }
    }
    else /* i2c_start_receive() FAIL */
    {
      status = ERR_FAIL;
    }
  }
  else  /* i2c_wait_for_busy_clear() FAIL */
  {
    status = ERR_FAIL;
  }

  return status;
}

/**
 * I2C1 Event Interrupt Handler.
 * Handles various I2C events such as Start Bit (SB), Address Sent (ADDR), Transmit Buffer Empty (TXE),
 * Receive Buffer Not Empty (RXNE), and Byte Transfer Finished (BTF).
 * This handler is responsible for managing the state of I2C communication, synchronizing with the main
 * program flow using semaphores, and handling data transmission and reception.
 *
 * The handler manages synchronization flags for each stage of I2C communication, ensuring that
 * the main program can proceed only when the appropriate I2C event has occurred.
 *
 * Note: This handler is designed for I2C1. Similar handlers should be implemented for other I2C
 *       instances if used.
 */
void I2C1_EV_IRQHandler(void)
{
  portBASE_TYPE higher_priority_task_woken = pdFALSE;

  bool I2C1_Is_BTF_Cleared = false;

  /* Check for start bit */
  if (I2C1_SB_SET)
  {
    if (i2c_synch_flags.I2C1_WAIT_SB == true)
    {
      // Event interrupt disable
      i2c_disable_event_interrupts(I2C1);

      i2c_synch_flags.I2C1_WAIT_SB = false;

      xSemaphoreGiveFromISR(i2c_semaphore_handle[i2c1], &higher_priority_task_woken);
    }
  }

  /* Check if address was sent */
  if (I2C1_ADDR_SET)
  {
    // Clear the address sent flag
    i2c_clear_address_flag(I2C1);

    if (i2c_synch_flags.I2C1_WAIT_ADDR == true)
    {
      i2c_synch_flags.I2C1_WAIT_ADDR = false;

      // Event interrupt disable
      i2c_disable_event_interrupts(I2C1);

      xSemaphoreGiveFromISR(i2c_semaphore_handle[i2c1], &higher_priority_task_woken);
    }
  }

  /* Check for TXE/RXNE */
  if (i2c_is_rx_tx_interrupt_enabled(I2C1))
  {
    // TXE/RXNE interrupt disable
    i2c_disable_rx_tx_interrupt(I2C1);

    if (I2C1_TXE_SET)
    {
      if (i2c_synch_flags.I2C1_WAIT_TXE == true)
      {
        i2c_synch_flags.I2C1_WAIT_TXE = false;

        xSemaphoreGiveFromISR(i2c_semaphore_handle[i2c1], &higher_priority_task_woken);
      }

      // Check for TXE with BTF
      if (I2C1_BTF_SET)
      {
        if (!I2C1_Is_BTF_Cleared)
        {
          (void)I2C1->DR;
          I2C1_Is_BTF_Cleared = true;
        }
      }
    }
    else if (I2C1_RXNE_SET)
    {
      if (i2c_synch_flags.I2C1_WAIT_RXNE == true)
      {
        i2c_synch_flags.I2C1_WAIT_RXNE = false;

        xSemaphoreGiveFromISR(i2c_semaphore_handle[i2c1], &higher_priority_task_woken);
      }

      // Check RXNE with BTF
      if (I2C1_BTF_SET)
      {
        if (!I2C1_Is_BTF_Cleared)
        {
          (void)I2C1->DR;
          I2C1_Is_BTF_Cleared = true;
        }
      }
    }
  }

  /* Check for BTF */
  if (I2C1_BTF_SET)
  {
    if (i2c_synch_flags.I2C1_WAIT_BTF == true)
    {
      i2c_synch_flags.I2C1_WAIT_BTF = false;

      // Event interrupt disable
      i2c_disable_event_interrupts(I2C1);

      xSemaphoreGiveFromISR(i2c_semaphore_handle[i2c1], &higher_priority_task_woken);
    }
    else
    {
      if (!I2C1_Is_BTF_Cleared)
      {
        (void)I2C1->DR;
      }
    }
  }

  /* Immediately switch to the higher priority task */
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

/**
 * I2C1 Error Interrupt Handler.
 * Handles error conditions during I2C communication for I2C1. This handler
 * checks for various error flags such as Bus Error (BERR), Arbitration Lost (ARLO),
 * Acknowledge Failure (AF), Overrun/Underrun (OVR), Timeout, and SMBus Alert.
 */
void I2C1_ER_IRQHandler(void)
{
  /* Check for BERR */
  if (I2C1_BERR_ERROR)
  {
    I2C1->SR1 &= ~(I2C_SR1_BERR);
  }

  /* Check for ARLO */
  if (I2C1_ARLO_ERROR)
  {
    I2C1->SR1 &= ~(I2C_SR1_ARLO);
  }

  /* Check for AF */
  if (I2C1_AF_ERROR)
  {
    I2C1->SR1 &= ~(I2C_SR1_AF);
  }

  /* Check for OVR */
  if (I2C1_OVR_ERROR)
  {
    I2C1->SR1 &= ~(I2C_SR1_OVR);
  }

  /* Check for Timeout Error */
  if (I2C1_TIMEOUT_ERROR)
  {
    I2C1->SR1 &= ~(I2C_SR1_TIMEOUT);
  }

  /* Check for Alert Error */
  if (I2C1_ALERT_ERROR)
  {
    I2C1->SR1 &= ~(I2C_SR1_SMBALERT);
  }
}




