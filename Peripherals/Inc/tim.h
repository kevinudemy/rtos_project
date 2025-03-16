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


#endif /* INC_TIM_H_ */
