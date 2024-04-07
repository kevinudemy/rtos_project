/*
 * rcc.c
 *
 * Contains function definitions required for RCC initialization and validation.
 */

#include "rcc.h"
#include "flash.h"

/**
 * Calculates and returns the optimal PLLP value based on the target system clock frequency.
 * @param mcu_freq_mhz Target system clock frequency in MHZ.
 * @return The PLLP division factor, which can be 2, 4, 6, or 8.
 */
static uint8_t rcc_get_pllp(uint8_t mcu_freq_mhz)
{
  if ((mcu_freq_mhz * 8) <= RCC_MAX_VCO_INPUT_FREQ)
  {
    return 8;
  }
  else if ((mcu_freq_mhz * 6) <= RCC_MAX_VCO_INPUT_FREQ)
  {
    return 6;
  }
  else if ((mcu_freq_mhz * 4) <= RCC_MAX_VCO_INPUT_FREQ)
  {
    return 4;
  }
  else
  {
    return 2;
  }
}

/**
 * Configures the PLLM, PLLN, and PLLP values for the PLL. PLLM is set based on the predefined HSE
 * frequency. PLLN and PLLP are calculated to achieve the desired system clock frequency based on
 * the PLL input frequency.
 * @param sysclk_freq_mhz Desired system clock frequency in MHZ.
 */
static void rcc_pll_config(uint8_t sysclk_freq_mhz)
{
  uint8_t PLLP = 0;
  uint16_t PLLN = 0;

  if (sysclk_freq_mhz > RCC_MAX_SYSCLK_MHZ)
  {
    sysclk_freq_mhz = RCC_MAX_SYSCLK_MHZ;
  }

  PLLP = rcc_get_pllp(sysclk_freq_mhz);
  PLLN = sysclk_freq_mhz * PLLP;

  // Set PLLM
  // For the PLLM parameter, use the RCC_HSE_MHZ definition
  uint8_t PLLM = RCC_HSE_MHZ;
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
  RCC->PLLCFGR |= PLLM << RCC_PLLCFGR_PLLM_Pos;

  // Set the PLLN
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN);
  RCC->PLLCFGR |= PLLN << RCC_PLLCFGR_PLLN_Pos;

  // Set the PLLP
  PLLP = (PLLP / 2) - 1;
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLP);
  RCC->PLLCFGR |= PLLP << RCC_PLLCFGR_PLLP_Pos;
}

void rcc_init(void)
{
  // RCC Initialization:
  // set the system clock frequency, configure wait states,
  // enable HSE, configure PLL as the system clock source,
  // wait for the PLL clock to stabilize before disabling the HSI, and set the clock prescalers.

  uint8_t sysclk_freq_mhz = RCC_MAX_SYSCLK_MHZ;

  // Configure flash wait states (using sysclk_freq_mhz --> assuming AHB division by 1)
  flash_config_wait_states(sysclk_freq_mhz);

  rcc_hse_enable();

  rcc_pll_source(RCC_PLL_SRC_HSE);

  rcc_pll_config(sysclk_freq_mhz);

  rcc_pll_enable();

  rcc_ahb_set_prescaler(RCC_SYSCLK_DIV_1);

  rcc_sysclk_set_source(RCC_SYSCLK_SRC_PLLP);

  while (RCC_CFGR_SWS_PLL != rcc_sysclk_get_source());

  rcc_hsi_disable();

  rcc_apb1_set_prescaler(RCC_APB1_DIV_4);
  rcc_apb2_set_prescaler(RCC_APB2_DIV_4);
}

uint32_t rcc_get_sysclk_freq(void)
{
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U;
  uint32_t sysclck_freq = 0u;

  switch (RCC->CFGR & RCC_CFGR_SWS)
  {
    case RCC_CFGR_SWS_HSI:
      sysclck_freq = RCC_HSI_FREQ;    /* HSI used as system clock source */
      break;

    case RCC_CFGR_SWS_HSE:
      sysclck_freq = RCC_HSE_FREQ;    /* HSE used as system clock  source */
      break;

    case RCC_CFGR_SWS_PLL:            /* PLL used as system clock  source */
      // PLL_VCO = (HSE_VALUE or HSI_VALUE / PLLM) * PLLN
      // SYSCLK  = PLL_VCO / PLLP
      pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;

      if (RCC_GET_PLL_OSCSOURCE() != RCC_PLLCFGR_PLLSRC_HSI)
      {
        /* HSE used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) RCC_HSE_FREQ * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t) pllm);
      }
      else
      {
        /* HSI used as PLL clock source */
        pllvco = (uint32_t) ((((uint64_t) RCC_HSI_FREQ *((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t) pllm);
      }
      pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) * 2U);

      sysclck_freq = pllvco / pllp;
      break;

    default:
      sysclck_freq = RCC_HSI_FREQ;
      break;
  }

  return sysclck_freq;
}

uint32_t rcc_get_hclk_freq(void)
{
  uint32_t sysclk = rcc_get_sysclk_freq();
  uint32_t hclk_div = RCC_GET_HCLK_DIV();

  /* Interpret hclk_div as actual divisor */
  if (hclk_div >= 8)
  {
    hclk_div = 1 << (hclk_div - 7);
  }
  else
  {
      hclk_div = 1; // prescaler is 1
  }

  return sysclk / hclk_div;
}

uint32_t rcc_get_pclk1_freq(void)
{
  uint32_t hclk = rcc_get_hclk_freq();
  uint32_t pclk1_div = RCC_GET_PCLK1_DIV();

  /* Interpret pclk1_div as actual divisor */
  if (pclk1_div >= 4)
  {
    pclk1_div = 1 << (pclk1_div - 3);
  }
  else
  {
    pclk1_div = 1;
  }

  return hclk / pclk1_div;
}

uint32_t rcc_get_pclk2_freq(void)
{
  uint32_t hclk = rcc_get_hclk_freq();
  uint32_t pclk2_div = RCC_GET_PCLK2_DIV();

  /* Interpret pclk2_div as actual divisor */
  if (pclk2_div >= 4)
  {
    pclk2_div = 1 << (pclk2_div - 3);
  }
  else
  {
    pclk2_div = 1;
  }

  return hclk / pclk2_div;
}




