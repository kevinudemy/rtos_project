/*
 * exti.h
 *
 * Provides enumerations and function prototypes for configuring and managing EXTI.
 */

#ifndef INC_EXTI_H_
#define INC_EXTI_H_

#include "gpio.h"     // This includes mcu.h indirectly

/**
 * Represents the GPIO ports that can be used as sources for external interrupts.
 * @Note See reference manual page 198, etc.
 */
typedef enum
{
  EXTI_PORT_A = 0,
  EXTI_PORT_B,
  EXTI_PORT_C,
  EXTI_PORT_D,
  EXTI_PORT_E,
  EXTI_PORT_F,
  EXTI_PORT_G,
  EXTI_PORT_H
} exti_port_e;

/**
 * Represents the specific lines that can trigger an external interrupt.
 * @Note: See reference manual page 246.
 */
typedef enum
{
  EXTI_PIN_0 = 0,
  EXTI_PIN_1,
  EXTI_PIN_2,
  EXTI_PIN_3,
  EXTI_PIN_4,
  EXTI_PIN_5,
  EXTI_PIN_6,
  EXTI_PIN_7,
  EXTI_PIN_8,
  EXTI_PIN_9,
  EXTI_PIN_10,
  EXTI_PIN_11,
  EXTI_PIN_12,
  EXTI_PIN_13,
  EXTI_PIN_14,
  EXTI_PIN_15,
  // Specific system handlers
  EXTI_VOLTAGE_DETECTION,
  EXTI_RTC_ALARM,
  EXTI_USB_OTG_FS,
  EXTI_DONT_USE,
  EXTI_USB_OTG_HS,
  EXTI_RTC_TAMPER,
  EXTI_RTC_WAKEUP
} exti_source_e;

/**
 * Used to define the edge sensitivity for the external interrupt triggers.
 */
typedef enum
{
  EXTI_RISING_EDGE = 0,
  EXTI_FALLING_EDGE,
  EXTI_RISING_AND_FALLING
} exti_trigger_e;

/**
 * Sets the EXTI line source for EXTI0-EXTI15 in the SYSCFG external interrupt configuration register.
 * @param port value to write to the EXTI control register.
 * @param pin pin number to be configured for external interrupts.
 */
void exti_set_source(exti_port_e port, gpio_num_e pin);

/**
 * Sets the trigger edge of the specified EXTI source by modifying the EXTI RTSR and/or FTSR registers.
 * @param source one of the external interrupt/event lines.
 * @param trigger rising edge, falling edge or rising and falling edge.
 */
void exti_set_trigger_edge(exti_source_e source, exti_trigger_e trigger);

/**
 * Enables the specified EXTI interrupt by unmasking it in the EXTI IMR register and calls NVIC_EnableIRQ().
 * @param source one of the external interrupt/event lines.
 * @param irq_num the IRQ number to pass to NVIC_EnableIRQ().
 */
void exti_enable_irq(exti_source_e source, IRQn_Type irq_num);

/**
 * Disables the specified EXTI interrupt by masking it in the EXTI IMR register and calls NVIC_DisableIRQ().
 * @param source one of the external interrupt/event lines.
 * @param irq_num the IRQ number to pass to NVIC_DisableIRQ().
 */
void exti_disable_irq(exti_source_e source, IRQn_Type irq_num);


#endif /* INC_EXTI_H_ */
