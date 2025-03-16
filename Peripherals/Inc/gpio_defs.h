/*
 * gpio_defs.h
 *
 * Defines GPIO ports and pins for all hardware components used in the system.
 *
 */

#ifndef INC_GPIO_DEFS_H_
#define INC_GPIO_DEFS_H_

#include "mcu.h"
#include "gpio.h"

// User interface
#define USER_LED_PORT     GPIOA
#define USER_LED_PIN      GPIO_PIN_5
#define USER_BUTTON_PORT  GPIOC
#define USER_BUTTON_PIN   GPIO_PIN_13


#endif /* INC_GPIO_DEFS_H_ */
