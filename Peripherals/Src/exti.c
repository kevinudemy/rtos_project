/*
 * exti.c
 *
 * Implements function definitions for configuring and managing External Interrupts and Events (EXTI).
 */

#include "exti.h"

void exti_set_source(exti_port_e port, gpio_num_e pin)
{
  // First clear, then set within the correct EXTICR[0...3]
  SYSCFG->EXTICR[pin >> 2] &= ~(0x0F << ((pin % 4) * 4));
  SYSCFG->EXTICR[pin >> 2] |= (port << ((pin % 4) * 4));
}

void exti_set_trigger_edge(exti_source_e source, exti_trigger_e trigger)
{
  if (EXTI_RISING_EDGE == trigger)
  {
    EXTI->RTSR |= (1 << source);
    EXTI->FTSR &= ~(1 << source);
  }
  else if (EXTI_FALLING_EDGE == trigger)
  {
    EXTI->FTSR |= (1 << source);
    EXTI->RTSR &= ~(1 << source);
  }
  else if (EXTI_RISING_AND_FALLING == trigger)
  {
    EXTI->RTSR |= (1 << source);
    EXTI->FTSR |= (1 << source);
  }
}

void exti_enable_irq(exti_source_e source, IRQn_Type irq_num)
{
  EXTI->IMR |= (1 << source);
  NVIC_EnableIRQ(irq_num);
}

void exti_disable_irq(exti_source_e source, IRQn_Type irq_num)
{
  EXTI->IMR &= ~(1 << source);
  NVIC_DisableIRQ(irq_num);
}




