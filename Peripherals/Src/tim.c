/*
 * tim.c
 *
 * Contains the initialization and configuration of various timers used in the application.
 */

#include "tim.h"
#include "rcc_clock_defs.h"

void tim2_init(void)
{
  // Enable the clock for TIM2
  RCC_TIM2_CLK_ENABLE();

  // Timer frequency = APB1 TIM2 clock frequency /(1 + PSC) = 90 MHz/(1 + 8999) = 10 KHz
  // Trigger frequency = 10,000 Hz / (1 + ARR) = 10,000 Hz/(1 + 9999) = 1 Hz
  TIM2->PSC = 8999;
  TIM2->ARR = 9999;

  // Set the MMS bits to "Update" mode
  TIM2->CR2 |= (TIM_CR2_MMS_1);

  // Set the UG bit to generate an update event
  TIM2->EGR |= (TIM_EGR_UG);

  // Enable the TIM2 counter
  TIM2->CR1 |= (TIM_CR1_CEN);
}




