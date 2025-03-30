/*
 * irq.c
 *
 * Configures IRQ priority levels for the system by setting priority levels in
 * the NVIC (Nested Vectored Interrupt Controller) for specific interrupts.
 */

#include "irq.h"

void irq_set_priorities(void)
{
  // Internal temperature sensor
  NVIC_SetPriority(ADC_IRQn, IRQ_ADC_PRIORITY);

  // Button EXTI
  NVIC_SetPriority(EXTI15_10_IRQn, IRQ_EXTI15_10_PRIORITY);

  // Sensirion sensors & related timer
  NVIC_SetPriority(I2C1_EV_IRQn, IRQ_I2C1_EV_PRIORITY);
  NVIC_SetPriority(TIM5_IRQn, IRQ_TIM5_PRIORITY);
}
