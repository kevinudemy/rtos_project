/*
 * iwdg.c
 *
 * Contains the necessary functions for implementing the Independent Watchdog (IWDG).
 */

#include "iwdg.h"
#include "rcc.h"

/**
 * Configures the IWDG prescaler and reload value for the requested timeout.
 *
 * The smallest prescaler capable of representing the requested timeout
 * is selected automatically.
 *
 * @param timeout_ms Desired timeout in milliseconds.
 */
static void iwdg_config(uint16_t timeout_ms)
{
  iwdg_prescaler_e prescaler = IWDG_PRESCALER_256;
  uint32_t prescaler_val = 256U;
  uint32_t required_ticks = 0U;
  uint32_t reload = 0U;

  /*
   * Select the smallest prescaler that allows the requested timeout
   * to fit within the 12-bit watchdog counter.
   */
  for (uint32_t candidate = (uint32_t) IWDG_PRESCALER_4;
       candidate <= (uint32_t) IWDG_PRESCALER_256;
       candidate++)
  {
    uint32_t candidate_prescaler = 4U << candidate;

    uint32_t numerator = (uint32_t) timeout_ms * RCC_LSI_FREQ;

    uint32_t denominator = 1000U * candidate_prescaler;

    /*
     * Round up so that the configured watchdog timeout is not
     * shorter than the requested timeout.
     */
    required_ticks = (numerator + denominator - 1U) / denominator;

    if (required_ticks <= IWDG_MAX_COUNTER_TICKS)
    {
      prescaler = (iwdg_prescaler_e) candidate;
      prescaler_val = candidate_prescaler;
      break;
    }
  }

  /*
   * Calculate the watchdog ticks for the selected prescaler.
   */
  {
    uint32_t numerator = (uint32_t) timeout_ms * RCC_LSI_FREQ;

    uint32_t denominator = 1000U * prescaler_val;

    required_ticks = (numerator + denominator - 1U) / denominator;
  }

  /*
   * The watchdog period is based on RLR + 1 counter ticks.
   */
  if (required_ticks > 0U)
  {
    reload = required_ticks - 1U;
  }

  /*
   * Clamp the reload value to the 12-bit hardware limit.
   */
  if (reload > IWDG_MAX_RELOAD_VALUE)
  {
    reload = IWDG_MAX_RELOAD_VALUE;
  }

  IWDG->PR = prescaler;
  IWDG->RLR = reload;
}

void iwdg_init(void)
{
  rcc_lsi_enable();
  iwdg_enable_write_access();
  iwdg_config(IWDG_TIMEOUT);
  iwdg_enable();
}




