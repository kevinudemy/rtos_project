/*
 * irq.h
 *
 * Contains IRQ priority definitions and initialization function prototype.
 */

#ifndef INC_IRQ_H_
#define INC_IRQ_H_

#include "mcu.h"
#include "FreeRTOSConfig.h"

// ADC interrupt priority, critical for temperature sensing
#define IRQ_ADC_PRIORITY    ((uint32_t) (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 0))

// External interrupt priority for the user button
#define IRQ_EXTI15_10_PRIORITY  ((uint32_t) (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 0))

// I2C event and timer interrupt priorities for Sensirion sensor operations.
#define IRQ_I2C1_EV_PRIORITY          ((uint32_t) (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1))
#define IRQ_TIM5_PRIORITY             ((uint32_t) (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2))

/**
 * Configures the priority levels for various IRQs in the system by setting the
 * priority levels in the NVIC (Nested Vectored Interrupt Controller) for specific interrupts.
 */
void irq_set_priorities(void);


#endif /* INC_IRQ_H_ */
