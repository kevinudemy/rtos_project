/*
 * tim.h
 *
 * Contains function prototypes for initializing various timers used in the application.
 */

#ifndef INC_TIM_H_
#define INC_TIM_H_

#include "mcu.h"

/**
 * Initializes TIM2 to trigger ADC1 conversions for the internal temperature sensor at 1 Hz.
 *
 * Configuration steps:
 * 1. Enable TIM2 clock.
 * 2. Set prescaler (PSC) and auto-reload register (ARR) for 1 Hz frequency.
 * 3. Configure Master Mode Selection (MMS) to use update event as trigger output.
 * 4. Force an update event to reload prescaler immediately.
 * 5. Enable TIM2 counter.
 */
void tim2_init(void);

/**
 * Initializes TIM5 to create microsecond delays for Sensirion sensor communication.
 *
 * Configuration steps:
 * 1. Enable TIM5 clock.
 * 2. Reset timer settings: disable counter enable, clear status and count registers.
 * 3. Set prescaler (PSC) to achieve 1 MHz frequency for 1 microsecond resolution.
 * 4. Force an update event to apply prescaler immediately.
 * 5. Set One Pulse Mode (OPM) to stop automatically after the delay.
 * 6. Enable TIM5 interrupt in NVIC.
 */
void tim5_init(void);

/**
 * Generates a delay in microseconds using TIM5.
 *
 * Steps performed:
 * 1. Disable the timer to allow configuration.
 * 2. Set auto-reload register (ARR) for desired delay.
 * 3. Clear counter and update flag to ensure a clean start.
 * 4. Enable update interrupts.
 * 5. Enable the counter to start the delay.
 *
 * @param microseconds The desired delay in microseconds.
 */
void tim5_delay_microseconds(uint32_t microseconds);


#endif /* INC_TIM_H_ */
