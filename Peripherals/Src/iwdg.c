/*
 * iwdg.c
 *
 * Contains the necessary functions for implementing the Independent Watchdog (IWDG).
 */

#include "iwdg.h"
#include "rcc.h"

/**
 * Sets the IWDG prescaler and IWDG reload value based on the desired timeout and prescaler value.
 * @param timeout_ms Desired timeout in milliseconds.
 * @param prescaler Desired IWDG prescaler value.
 */
static void iwdg_config(uint16_t timeout_ms, iwdg_prescaler_e prescaler)
{
  uint16_t calculated_val;
  uint16_t prescaler_val;
  uint16_t max_timeout;

  // Convert the enumeration value to the actual prescaler divisor and set the max timeout
  switch (prescaler)
  {
    case IWDG_PRESCALER_4:
      prescaler_val = 4;
      max_timeout = 512;
      break;
    case IWDG_PRESCALER_8:
      prescaler_val = 8;
      max_timeout = 1024;
      break;
    case IWDG_PRESCALER_16:
      prescaler_val = 16;
      max_timeout = 2048;
      break;
    case IWDG_PRESCALER_32:
      prescaler_val = 32;
      max_timeout = 4096;
      break;
    case IWDG_PRESCALER_64:
      prescaler_val = 64;
      max_timeout = 8192;
      break;
    case IWDG_PRESCALER_128:
      prescaler_val = 128;
      max_timeout = 16384;
      break;
    case IWDG_PRESCALER_256:
      prescaler_val = 256;
      max_timeout = 32768;
      break;
    default:
      // Handle error or set default prescaler
      prescaler_val = 32;
      max_timeout = 4096;
      break;
  }

  // Ensure the desired timeout does not exceed the maximum allowed timeout for the chosen prescaler
  if (timeout_ms > max_timeout)
  {
    timeout_ms = max_timeout;
  }

  // Calculate the raw reload value based on the desired timeout, LSI frequency, and chosen prescaler
  calculated_val = (timeout_ms * RCC_LSI_FREQ) / (1000 * prescaler_val);

  // Ensure the calculated value does not exceed the maximum reload value (0xFFF)
  if (calculated_val > 0xFFF)
  {
    calculated_val = 0xFFF;
  }

  // Set the prescaler
  IWDG->PR = prescaler;

  // Set the reload value
  IWDG->RLR = calculated_val;
}

void iwdg_init(void)
{
  rcc_lsi_enable();
  iwdg_enable_write_access();
  iwdg_config(IWDG_TIMEOUT, IWDG_PRESCALER_32);
  iwdg_enable();
}





