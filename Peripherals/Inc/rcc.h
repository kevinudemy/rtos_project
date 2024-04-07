/*
 * rcc.h
 *
 * Provides macro definitions and function prototypes for RCC (Reset and Clock Control)
 * initialization and management within the MCU. This includes oscillator configurations,
 * system clock source selection, and bus prescaler options.
 */

#ifndef INC_RCC_H_
#define INC_RCC_H_

#include <stdbool.h>
#include "mcu.h"

/**
 * Defines the High Speed External (HSE) oscillator frequency in megahertz (MHz).
 *
 * On Nucleo boards, the HSE oscillator signal is derived from the ST-LINK's microcontroller
 * oscillator.
 */
#define RCC_HSE_MHZ		          (8)

/**
 * Defines the maximum system clock (SYSCLK) frequency of the microcontroller in megahertz (MHz).
 *
 * The system clock drives the CPU, memory and peripherals. It is often derived from one of the
 * on-board oscillators (HSI, HSE) or an output from the phase-locked loop (PLL). The maximum
 * SYSCLK frequency may vary between different microcontroller models.
 */
#define RCC_MAX_SYSCLK_MHZ      (180)

/**
 * Defines the maximum input frequency that the voltage-controlled oscillator (VCO) can handle
 * within the phase-locked loop (PLL).
 *
 * The VCO generates a high-frequency signal which can be used to drive the system clock (SYSCLK)
 * or other on-chip peripherals. The frequency of this signal is influenced by the input signal
 * to the VCO. This defined value is the maximum allowable frequency for this input signal,
 * and it is expressed in megahertz (MHz).
 */
#define RCC_MAX_VCO_INPUT_FREQ  (432)

/**
 * Defines the High Speed Internal (HSI) oscillator frequency in hertz.
 *
 * The HSI oscillator is an internal RC oscillator typically used as a clock
 * source when the external crystal (HSE) is not available or as a backup clock source.
 */
#define RCC_HSI_FREQ            ((uint32_t) 16000000ul)

/**
 * Defines the High Speed External (HSE) oscillator frequency in hertz.
 *
 * The HSE oscillator relies on an external crystal or ceramic resonator to generate
 * a stable frequency. On Nucleo boards, the HSE signal is derived from the ST-LINK
 * microcontroller's oscillator circuitry. This value should match the actual frequency
 * of the external crystal used in the hardware design.
 */
#define RCC_HSE_FREQ            ((uint32_t)  8000000ul)

/**
 * Defines the Low Speed Internal (LSI) osciallator frequency in hertz.
 */
#define RCC_LSI_FREQ            ((uint32_t)  32000ul)

/**
 * PLL clock source options.
 */
#define RCC_PLL_SRC_HSI		  RCC_PLLCFGR_PLLSRC_HSI
#define RCC_PLL_SRC_HSE		  RCC_PLLCFGR_PLLSRC_HSE

/**
 * AHB prescaler options.
 */
#define RCC_SYSCLK_DIV_1	  RCC_CFGR_HPRE_DIV1
#define RCC_SYSCLK_DIV_2	  RCC_CFGR_HPRE_DIV2
#define RCC_SYSCLK_DIV_4	  RCC_CFGR_HPRE_DIV4
#define RCC_SYSCLK_DIV_8	  RCC_CFGR_HPRE_DIV8
#define RCC_SYSCLK_DIV_16	  RCC_CFGR_HPRE_DIV16
#define RCC_SYSCLK_DIV_64	  RCC_CFGR_HPRE_DIV64
#define RCC_SYSCLK_DIV_128	RCC_CFGR_HPRE_DIV128
#define RCC_SYSCLK_DIV_256	RCC_CFGR_HPRE_DIV256
#define RCC_SYSCLK_DIV_512	RCC_CFGR_HPRE_DIV512

/**
 * System clock source options.
 */
#define RCC_SYSCLK_SRC_HSI	RCC_CFGR_SW_HSI
#define RCC_SYSCLK_SRC_HSE	RCC_CFGR_SW_HSE
#define RCC_SYSCLK_SRC_PLLP	RCC_CFGR_SW_PLL
#define RCC_SYSCLK_SRC_PLLR	RCC_CFGR_SW_PLLR

/**
 * APB1 prescaler options.
 */
#define RCC_APB1_DIV_1		  RCC_CFGR_PPRE1_DIV1
#define RCC_APB1_DIV_2		  RCC_CFGR_PPRE1_DIV2
#define RCC_APB1_DIV_4		  RCC_CFGR_PPRE1_DIV4
#define RCC_APB1_DIV_8		  RCC_CFGR_PPRE1_DIV8
#define RCC_APB1_DIV_16		  RCC_CFGR_PPRE1_DIV16

/**
 * APB2 prescaler options.
 */
#define RCC_APB2_DIV_1		  RCC_CFGR_PPRE2_DIV1
#define RCC_APB2_DIV_2		  RCC_CFGR_PPRE2_DIV2
#define RCC_APB2_DIV_4		  RCC_CFGR_PPRE2_DIV4
#define RCC_APB2_DIV_8		  RCC_CFGR_PPRE2_DIV8
#define RCC_APB2_DIV_16		  RCC_CFGR_PPRE2_DIV16

/**
 * Macro to extract the AHB prescaler value (HCLK) from the CFGR register.
 *
 * The AHB (Advanced High-performance Bus) prescaler is used to derive the HCLK (High-speed Clock)
 * frequency from the system clock (SYSCLK). This macro retrieves the prescaler value by masking and
 * bit-shifting the contents of the Clock configuration register (CFGR).
 */
#define RCC_GET_HCLK_DIV() ((RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos)

/**
 * Macro to extract the APB1 prescaler value (PCLK1) from the CFGR register.
 *
 * The APB1 (Advanced Peripheral Bus 1) prescaler is used to derive the PCLK1 frequency from the HCLK.
 * This macro retrieves the prescaler value by masking and bit-shifting the contents of the Clock
 * configuration register (CFGR).
 */
#define RCC_GET_PCLK1_DIV() ((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos)

/**
 * Macro to extract the APB2 prescaler value (PCLK2) from the CFGR register.
 *
 * The APB2 (Advanced Peripheral Bus 2) prescaler is used to derive the PCLK2 frequency from the HCLK.
 * This macro retrieves the prescaler value by masking and bit-shifting the contents of the Clock
 * configuration register (CFGR).
 */
#define RCC_GET_PCLK2_DIV() ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos)

/**
 * Macro to extract the PLL source oscillator.
 *
 * The Phase-locked loop (PLL) is a control system that generates an output signal
 * whose phase is related to the phase of an input signal. The source of this input
 * signal is specified in the PLL configuration register (PLLCFGR). This macro
 * retrieves the current PLL source oscillator by masking the PLLCFGR register.
 */
#define RCC_GET_PLL_OSCSOURCE() ((uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC))

/**
 * Enables the LSI oscillator and waits until ready.
 */
static inline void rcc_lsi_enable(void)
{
   RCC->CR |= (RCC_CSR_LSION);

   // Wait until the LSI oscillator is stable
   while (!(RCC->CR & RCC_CSR_LSIRDY));
}

/**
 * Disables the LSI oscillator.
 */
 static inline void rcc_lsi_disable(void)
 {
    RCC->CR &= ~(RCC_CSR_LSION);
 }

/**
 * Enables the HSE crystal oscillator as the clock source and waits until ready.
 */
static inline void rcc_hse_enable(void)
{
  RCC->CR |= (RCC_CR_HSEON);

  // Wait until the HSE oscillator is stable
  while (!(RCC->CR & RCC_CR_HSERDY));
}

/**
 * Disables the HSI.
 */
static inline void rcc_hsi_disable(void)
{
  RCC->CR &= ~(RCC_CR_HSION);
}

/**
 * Sets the PLL clock source (HSE or HSI).
 * @param ClkSrc can be one of the following values:
 * 	RCC_PLL_SRC_HSI
 * 	RCC_PLL_SRC_HSE
 * @note can only be set when PLL is disabled.
 */
static inline void rcc_pll_source(uint32_t ClkSrc)
{
  RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
  RCC->PLLCFGR |= (ClkSrc);
}

/**
 * Gets the system clock source.
 * @return system clock switch status SWS in RCC_CFGR.
 */
static inline uint32_t rcc_sysclk_get_source(void)
{
  return (uint32_t)(RCC->CFGR & RCC_CFGR_SWS);
}

/**
 * Enables PLL and waits until ready.
 */
static inline void rcc_pll_enable(void)
{
  RCC->CR |= (RCC_CR_PLLON);

  // Wait until main PLL is ready and stable.
  while (!(RCC->CR & RCC_CR_PLLRDY));
}

/**
 * Sets the AHB prescaler.
 * @param Prescaler can be one of the following values:
 * 	RCC_SYSCLK_DIV_1
 *	RCC_SYSCLK_DIV_2
 *	RCC_SYSCLK_DIV_4
 *	RCC_SYSCLK_DIV_8
 *	RCC_SYSCLK_DIV_16
 *	RCC_SYSCLK_DIV_64
 *	RCC_SYSCLK_DIV_128
 *	RCC_SYSCLK_DIV_256
 *	RCC_SYSCLK_DIV_512
 */
static inline void rcc_ahb_set_prescaler(uint32_t Prescaler)
{
  RCC->CFGR &= ~(RCC_CFGR_HPRE);
  RCC->CFGR |= (Prescaler);
}

/**
 * Sets the system clock as HSI, HSE, PLLP, or PLLR.
 * @param Source can be one of the following values:
 * 	RCC_SYSCLK_SRC_HSI
 * 	RCC_SYSCLK_SRC_HSE
 * 	RCC_SYSCLK_SRC_PLLP
 * 	RCC_SYSCLK_SRC_PLLR
 */
static inline void rcc_sysclk_set_source(uint32_t Source)
{
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= (Source);
}

/**
 * Sets the APB1 prescaler.
 * @param Prescaler can be one of the following values:
 * 	RCC_APB1_DIV_1
 * 	RCC_APB1_DIV_2
 * 	RCC_APB1_DIV_4
 * 	RCC_APB1_DIV_8
 * 	RCC_APB1_DIV_16
 */
static inline void rcc_apb1_set_prescaler(uint32_t Prescaler)
{
  RCC->CFGR &= ~(RCC_CFGR_PPRE1);
  RCC->CFGR |= (Prescaler);
}

/**
 * Sets the APB2 prescaler.
 * @param prescaler can be one of the following values:
 * 	RCC_APB2_DIV_1
 * 	RCC_APB2_DIV_2
 * 	RCC_APB2_DIV_4
 * 	RCC_APB2_DIV_8
 * 	RCC_APB2_DIV_16
 */
static inline void rcc_apb2_set_prescaler(uint32_t Prescaler)
{
  RCC->CFGR &= ~(RCC_CFGR_PPRE2);
  RCC->CFGR |= (Prescaler);
}

/**
 * Checks if the last reset was caused by the Independent Watchdog.
 * @return true if the IWDG reset flag is active, false otherwise.
 */
static inline bool rcc_iwdg_flag_active(void)
{
  return ((RCC->CSR & RCC_CSR_IWDGRSTF) == (RCC_CSR_IWDGRSTF));
}

/**
 * Clears reset flags, e.g., event caused by the IWDG.
 */
static inline void rcc_clear_reset_flags(void)
{
  RCC->CSR |= RCC_CSR_RMVF;
}

/**
 * Initializes the clock configuration.
 * Clocks configuration after initialization:
 * 	System Clock Source 	= PLL (HSE)
 * 	SYSCLK (Hz)		  = 180000000
 * 	HCLK (Hz)		    = 180000000
 * 	AHB Prescaler		= 1
 * 	APB1 Prescaler	= 4
 * 	APB2 Prescaler	= 4
 */
void rcc_init(void);

/**
 * Returns the system clock frequency.
 * @note Referenced from the HAL_RCC_GetSysClockFreq() function.
 * Copyright (c) 2017 STMicroelectronics.
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
   disclaimer in the documentation and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * @return System clock (SYSCLK) frequency
 */
uint32_t rcc_get_sysclk_freq(void);

/**
 * Returns the HCLK (core clock) frequency.
 * @return HCLK frequency.
 */
uint32_t rcc_get_hclk_freq(void);

/**
 * Returns the PCLK1 (APB1) frequency.
 * @return PCLK1 frequency.
 */
uint32_t rcc_get_pclk1_freq(void);

/**
 * Returns the PCLK2 (APB2) frequency
 * @return PCLK2 frequency
 */
uint32_t rcc_get_pclk2_freq(void);

#endif /* INC_RCC_H_ */
