/*
 * gpio.h
 *
 * Provides macro definitions, data type enumerations, and function prototypes related to
 * General-Purpose Input/Output (GPIO) operations.
 *
 */

#ifndef INC_GPIO_H_
#define INC_GPIO_H_

#include "mcu.h"
#include "gpio_defs.h"

/**
 * Defines the pin numbers for use in GPIO operations. Each pin number
 * corresponds to a specific pin on the GPIO port, ranging from 0 to 15.
 */
typedef enum
{
  GPIO_PIN_0 = 0,
  GPIO_PIN_1,
  GPIO_PIN_2,
  GPIO_PIN_3,
  GPIO_PIN_4,
  GPIO_PIN_5,
  GPIO_PIN_6,
  GPIO_PIN_7,
  GPIO_PIN_8,
  GPIO_PIN_9,
  GPIO_PIN_10,
  GPIO_PIN_11,
  GPIO_PIN_12,
  GPIO_PIN_13,
  GPIO_PIN_14,
  GPIO_PIN_15
} gpio_num_e;

/**
 * Specifies the operating mode for GPIO pins, including
 * input, output, alternate function, and analog.
 */
typedef enum
{
  GPIO_MODE_INPUT = 0,
  GPIO_MODE_OUTPUT,
  GPIO_MODE_ALTFUNC,
  GPIO_MODE_ANALOG
} gpio_mode_e;

/**
 * Defines the alternate function mappings for GPIO pins.
 * Each value corresponds to a specific alternate function, ranging
 * from AF0 to AF15, with a special value for no alternate function.
 */
typedef enum
{
  GPIO_AF_0 = 0,
  GPIO_AF_1,
  GPIO_AF_2,
  GPIO_AF_3,
  GPIO_AF_4,
  GPIO_AF_5,
  GPIO_AF_6,
  GPIO_AF_7,
  GPIO_AF_8,
  GPIO_AF_9,
  GPIO_AF_10,
  GPIO_AF_11,
  GPIO_AF_12,
  GPIO_AF_13,
  GPIO_AF_14,
  GPIO_AF_15,
  GPIO_AF_NONE
} gpio_alt_e;

/**
 * Specifies the speed of GPIO output operations.
 * Includes options for low, medium, fast, and high speeds,
 * with a special value for input mode where speed setting is not applicable.
 */
typedef enum
{
  GPIO_SPEED_LOW = 0,
  GPIO_SPEED_MEDIUM,
  GPIO_SPEED_FAST,
  GPIO_SPEED_HIGH,
  GPIO_SPEED_NONE_INPUT
} gpio_speed_e;

/**
 * Defines the output type for GPIO pins,
 * including push-pull and open-drain configurations.
 */
typedef enum
{
  GPIO_OUTPUT_PUSHPULL = 0,
  GPIO_OUTPUT_OPENDRAIN,
  GPIO_OUTPUT_NONE
} gpio_out_type_e;

/**
 * Specifies whether a GPIO pin should use an internal
 * pull-up resistor, pull-down resistor, or no pull resistor (floating).
 */
typedef enum
{
  GPIO_PULL_NONE = 0,
  GPIO_PULL_UP,
  GPIO_PULL_DOWN
} gpio_pull_e;

/**
 * Used to specify or read the state of a GPIO pin.
 * Can be either set (high) or reset (low).
 */
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
} gpio_pin_state_e;

/**
 * Macro to set a GPIO pin.
 * Sets the specified GPIO pin to a high state (1).
 * It writes to the Bit Set Reset Register (BSRR) of the GPIO port,
 * which allows atomic pin setting without affecting other pins.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin to be set, using GPIO_Num_e for reference.
 */
#define GPIO_SET_PIN(port, pin)   (((port)->BSRR) = (0x1UL << (pin)))

/**
 * Macro to reset a GPIO pin.
 * Resets the specified GPIO pin to a low state (0).
 * It writes to the high-order 16 bits of the Bit Set Reset Register (BSRR)
 * of the GPIO port, allowing atomic pin resetting without affecting other pins.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin to be reset, using GPIO_Num_e for reference.
 */
#define GPIO_RESET_PIN(port, pin) (((port)->BSRR) = (0x1UL << ((pin) + 16)))

/**
 * Macro to read the current state of a GPIO pin.
 * Reads the state of the specified GPIO pin.
 * It checks the Input Data Register (IDR) of the GPIO port and returns
 * a non-zero value if the pin is high, and zero if the pin is low.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin to be read, using GPIO_Num_e for reference.
 */
#define GPIO_READ_PIN(port, pin)  (((port)->IDR) & (0x1UL << (pin)))

/**
 * Initializes the GPIO configuration for various components of the system.
 * This function sets up GPIO pins for different functionalities including
 * input, output, alternate functions, and analog modes. It includes configurations
 * for the user LED, user button, test pin, internal temperature sensor, FRAM interface,
 * Sensirion temperature & humidity sensor, and Modbus interface.
 *
 * - Enables clocks for all GPIO ports and the SYSCFG module.
 * - Configures the user LED pin as an output.
 * - Sets the user button pin as an input.
 * - Initializes a test pin for output.
 * - Configures the internal temperature sensor pin in analog mode.
 * - Sets up GPIO pins for the FRAM interface with SPI alternate functions.
 * - Configures GPIO pins for the Sensirion sensor with I2C alternate functions.
 * - Initializes GPIO pins for Modbus communication with appropriate alternate functions.
 */
void gpio_init(void);

/**
 * Sets or resets the state of a specified GPIO pin in a specified GPIO port.
 * Used to turn on or off a pin if it's configured as an output.
 *
 * @Note: This function uses GPIOx_BSRR register to allow atomic read/modify accesses.
 * In this way, there is no risk of an IRQ occurring between the read and the modify access.
 *
 * @param port Pointer to the GPIOx port where the pin is located.
 * @param pin The number of the pin in the specified port to be set or reset.
 *            Should be a value from 0 to 15, corresponding to GPIO_PIN_0 to GPIO_PIN_15.
 * @param pin_state Specifies the desired state of the pin.
 *                  This parameter can be GPIO_PIN_SET to set the pin or GPIO_PIN_RESET to reset the pin.
 *                  The pin is set by writing to the BSRR register (set bits), and reset by writing
 *                  to the higher half of BSRR (bits 16-31, corresponding to reset bits).
 */
void gpio_write_pin(GPIO_TypeDef *port, gpio_num_e pin, gpio_pin_state_e pin_state);

/**
 * Reads the current state of a specified GPIO pin.
 * Checks the status of a pin in a GPIO port, returning whether it is set or reset.
 * It can be used to read the level of a pin configured as an input.
 *
 * @param port Pointer to the GPIOx port where the pin is located.
 * @param pin The number of the pin in the specified port to be read.
 *            Should be a value from 0 to 15, corresponding to GPIO_PIN_0 to GPIO_PIN_15.
 * @return GPIO_PinState_e The current state of the pin.
 *         Returns GPIO_PIN_SET if the pin is in a high state, or GPIO_PIN_RESET if in a low state.
 *         This is determined by reading the IDR (Input Data Register) of the port.
 */
gpio_pin_state_e gpio_read_pin(GPIO_TypeDef *port, gpio_num_e pin);

/**
 * Toggles the state of a specified GPIO pin.
 * If the pin is currently set (high), it will be reset (set to low). If it is reset (low),
 * it will be set (set to high). This is useful for flipping the state of a pin, such as
 * turning an LED on and off.
 *
 * @param port Pointer to the GPIOx port where the pin is located.
 * @param pin The number of the pin in the specified port to be toggled.
 *            Should be a value from 0 to 15, corresponding to GPIO_PIN_0 to GPIO_PIN_15.
 *
 * Note: This function reads the current state of the pin from the IDR (Input Data Register)
 *       and then writes the opposite state back to the pin.
 */
void gpio_toggle_pin(GPIO_TypeDef *port, gpio_num_e pin);


#endif /* INC_GPIO_H_ */
