/*
 * gpio.c
 *
 * Implements function definitions for initializing and managing General-Purpose Input/Output (GPIO) pins. 
 * This file provides a comprehensive set of functionalities to configure GPIO pins including setting their 
 * modes (Input, Output, Alternate Function, Analog), output types (Push-Pull, Open-Drain), speed, and pull-up/
 * pull-down settings. It also includes functions for reading, writing, and toggling pin states.
 *
 */

#include "gpio.h"
#include "rcc_clock_defs.h"

/**
 * Sets the GPIO mode (Input, Output, AltFunc, Analog) for the specified port and pin.
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin in the specified port to be configured.
 *            Should be a value from 0 to 15 for GPIO pins GPIO_PIN_0 to GPIO_PIN_15.
 * @param mode Specifies the operating mode for the selected pin.
 *             This parameter can be a value of GPIO_Mode_e enumeration:
 *               - GPIO_MODE_INPUT:   Set pin as Input
 *               - GPIO_MODE_OUTPUT:  Set pin as General Purpose Output
 *               - GPIO_MODE_ALTFUNC: Set pin to Alternate Function
 *               - GPIO_MODE_ANALOG:  Set pin as Analog mode
 */
static void gpio_set_pin_mode(GPIO_TypeDef *port, gpio_num_e pin, gpio_mode_e mode)
{
  // Clear the current mode configuration for the pin
  port->MODER &= ~(0x3UL << (pin * 2));

  // Apply the new mode configuration if not INPUT
  if (mode != GPIO_MODE_INPUT)
  {
    port->MODER |= (mode << (pin * 2));
  }
}

/**
 * Sets the GPIO alternate function for a specific port and pin.
 * GPIO pins can be configured to serve alternate functions, such as peripheral mappings.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin in the specified port to be configured.
 *            Should be a value from 0 to 15 for GPIO pins GPIO_PIN_0 to GPIO_PIN_15.
 * @param alternate Specifies the alternate function to be assigned to the selected pin.
 *                  This parameter can be a value of GPIO_Alt_e enumeration, which corresponds
 *                  to various peripheral functionalities the pin can be assigned to, like
 *                  UART, SPI, I2C, etc.
 */
static void gpio_set_alt_function(GPIO_TypeDef *port, gpio_num_e pin, gpio_alt_e alternate)
{
  // Configure the lower half of the AF register array (AFR[0] for pins 0-7)
  if (pin <= GPIO_PIN_7)
  {
    port->AFR[0] &= ~(0xFUL << (pin * 4));    // Clear the alternate function bits for this pin
    port->AFR[0] |= (alternate << (pin * 4)); // Set new alternate function
  }
  else  // Configure the upper half of the AF register array (AFR[1] for pins 8-15)
  {
    // Subtract 8 from the pin number to start at bit 0 of the AFR[1] register
    port->AFR[1] &= ~(0xFUL << ((pin - 8) * 4));    // Clear the alternate function bits for this pin
    port->AFR[1] |= (alternate << ((pin - 8) * 4)); // Set new alternate function
  }
}

/**
 * Sets the operating speed of a GPIO output pin.
 * The speed setting controls the rate at which the pin can change state,
 * affecting the rise and fall times of the signal. This is particularly
 * important for high-frequency digital interfaces.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin in the specified port to be configured.
 *            Should be a value from 0 to 15 for GPIO pins GPIO_PIN_0 to GPIO_PIN_15.
 * @param speed Specifies the speed for the GPIO pin.
 *              This parameter can be a value of GPIO_Speed_e enumeration:
 *                - GPIO_SPEED_LOW: Low speed
 *                - GPIO_SPEED_MEDIUM: Medium speed
 *                - GPIO_SPEED_FAST: Fast speed
 *                - GPIO_SPEED_HIGH: High speed
 */
static void gpio_set_pin_speed(GPIO_TypeDef *port, gpio_num_e pin, gpio_speed_e speed)
{
  // Clear the current speed configuration for the pin
  port->OSPEEDR &= ~(0x3UL << (pin * 2));

  // Apply the new speed configuration
  if (speed == GPIO_SPEED_MEDIUM)
  {
    port->OSPEEDR |= (GPIO_SPEED_MEDIUM << (pin * 2));
  }
  else if (speed == GPIO_SPEED_FAST)
  {
    port->OSPEEDR |= (GPIO_SPEED_FAST << (pin * 2));
  }
  else if (speed == GPIO_SPEED_HIGH)
  {
    port->OSPEEDR |= (GPIO_SPEED_HIGH << (pin * 2));
  }
}

/**
 * Sets the output type of the GPIO pin as either Push-Pull or Open Drain.
 * The output type determines how the pin drives its output value. Push-Pull represents
 * a typical output driving high and low, while Open Drain can only pull the line low,
 * allowing the line to be pulled high by an external source.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin in the specified port to be configured.
 *            Should be a value from 0 to 15 for GPIO pins GPIO_PIN_0 to GPIO_PIN_15.
 * @param output_type Specifies the output type for the GPIO pin.
 *                    This parameter can be a value of GPIO_OutputType_e enumeration:
 *                      - GPIO_OUTPUT_PUSHPULL: Set the pin in push-pull mode.
 *                      - GPIO_OUTPUT_OPENDRAIN: Set the pin in open-drain mode.
 */
static void gpio_set_output_type(GPIO_TypeDef *port, gpio_num_e pin, gpio_out_type_e output_type)
{
  // Clear the output type (Push-Pull)
  port->OTYPER &= ~(0x1UL << pin);

  if (output_type == GPIO_OUTPUT_OPENDRAIN)
  {
    port->OTYPER |= (1 << pin);
  }
}

/**
 * Sets the push/pull resistor type (No Pull, Pull-Up, Pull-Down) for a specific GPIO pin.
 * This configuration determines whether the pin has an internal pull-up or pull-down resistor,
 * or none. This is useful for ensuring a stable logic level when the pin is floating (not actively driven).
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin in the specified port to be configured.
 *            Should be a value from 0 to 15, corresponding to GPIO pins GPIO_PIN_0 to GPIO_PIN_15.
 * @param pull Specifies the pull-up or pull-down activation for the selected pin.
 *             This parameter can be a value of the GPIO_Pull_e enumeration:
 *               - GPIO_PULL_NONE: No pull-up or pull-down resistor (floating)
 *               - GPIO_PULL_UP:   Enable internal pull-up resistor
 *               - GPIO_PULL_DOWN: Enable internal pull-down resistor
 */
static void gpio_set_pin_pull(GPIO_TypeDef *port, gpio_num_e pin, gpio_pull_e pull)
{
  // Clear the current Pin Pull (No PU/PD) configuration
  port->PUPDR &= ~(0x03 << (pin * 2));

  // Apply the new Pull-Up or Pull-Down configuration
  if (pull == GPIO_PULL_UP)
  {
    port->PUPDR |= (GPIO_PULL_UP << (pin *2));
  }
  else if (pull == GPIO_PULL_DOWN)
  {
    port->PUPDR |= (GPIO_PULL_DOWN << (pin *2));
  }
}

/**
 * Configures a GPIO pin with specified settings including mode, speed, output type, etc.
 * This is a comprehensive function that covers all basic aspects of GPIO pin configuration.
 *
 * @param port Pointer to the GPIOx port.
 * @param pin The number of the pin in the specified port to be configured.
 *            Should be a value from 0 to 15, corresponding to GPIO_PIN_0 to GPIO_PIN_15.
 * @param mode Specifies the operating mode of the pin (e.g., Input, Output, Alt Function, Analog).
 * @param alternate Specifies the alternate function to be assigned to the pin, if needed.
 *                  This is relevant only when the mode is set to Alternate Function.
 * @param out_type Specifies the output type (Push-Pull or Open Drain) for the pin.
 * @param speed Specifies the speed for the pin if it's configured as an output.
 * @param pull Specifies whether to enable internal pull-up or pull-down resistors.
 *
 * Note: This function combines several individual GPIO configuration functions into one,
 *       making it convenient to configure a pin with multiple settings in a single call.
 */
static void gpio_pin_config(GPIO_TypeDef *port, gpio_num_e pin, gpio_mode_e mode, gpio_alt_e alternate,
                            gpio_out_type_e out_type, gpio_speed_e speed, gpio_pull_e pull)
{
  // Set IO direction mode (Input, Output, Alternate, or Analog)
  gpio_set_pin_mode(port, pin, mode);

  // In case Mode is GPIO_MODE_ALTFUNC, set the Alternate Function
  if ((mode == GPIO_MODE_ALTFUNC) && (alternate != GPIO_AF_NONE))
  {
    gpio_set_alt_function(port, pin, alternate);
  }

  // In case of Output or Alternate Function, configure the Speed & Output Type
  if ((mode == GPIO_MODE_OUTPUT) || (mode == GPIO_MODE_ALTFUNC))
  {
    // Configure Speed
    gpio_set_pin_speed(port, pin, speed);

    // Configure Output Type
    gpio_set_output_type(port, pin, out_type);
  }

  // In case Mode is not Analog, activate Pull-up or Pull down resistor
  if (mode != GPIO_MODE_ANALOG)
  {
    gpio_set_pin_pull(port, pin, pull);
  }
}

void gpio_init(void)
{
  // Enable clocks for GPIOs
  RCC_GPIOA_CLK_ENABLE();
  RCC_GPIOB_CLK_ENABLE();
  RCC_GPIOC_CLK_ENABLE();

  // Enable SYSCFG module to enable GPIO ISRs
  RCC_SYSCFG_CLK_ENABLE();

  // USER LED
  gpio_pin_config(USER_LED_PORT,
                  USER_LED_PIN,
                  GPIO_MODE_OUTPUT,
                  GPIO_AF_NONE,
                  GPIO_OUTPUT_PUSHPULL,
                  GPIO_SPEED_LOW,
                  GPIO_PULL_NONE);

  // USER Button
  gpio_pin_config(USER_BUTTON_PORT,
                  USER_BUTTON_PIN,
                  GPIO_MODE_INPUT,
                  GPIO_AF_NONE,
                  GPIO_OUTPUT_NONE,
                  GPIO_SPEED_LOW,
                  GPIO_PULL_UP);

  // Test pin
  gpio_pin_config(TEST_PORT,
                  TEST_PIN,
                  GPIO_MODE_OUTPUT,
                  GPIO_AF_NONE,
                  GPIO_OUTPUT_PUSHPULL,
                  GPIO_SPEED_FAST,
                  GPIO_PULL_DOWN);

  // Sensirion sensors
  gpio_pin_config(SENSIRION_SCL_PORT,
                  SENSIRION_SCL_PIN,
                  GPIO_MODE_ALTFUNC,
                  GPIO_AF_4,
                  GPIO_OUTPUT_OPENDRAIN,
                  GPIO_SPEED_FAST,
                  GPIO_PULL_NONE);
  gpio_pin_config(SENSIRION_SDA_PORT,
                  SENSIRION_SDA_PIN,
                  GPIO_MODE_ALTFUNC,
                  GPIO_AF_4,
                  GPIO_OUTPUT_OPENDRAIN,
                  GPIO_SPEED_FAST,
                  GPIO_PULL_NONE);
}

void gpio_write_pin(GPIO_TypeDef *port, gpio_num_e pin, gpio_pin_state_e pin_state)
{
  // Set or reset the specified pin
  if (pin_state != GPIO_PIN_RESET)
  {
    // Set the pin (turn on)
    port->BSRR = (0x1UL << pin);
  }
  else
  {
    // Reset the pin (turn off)
    port->BSRR = (0x1UL << (pin + 16));
  }
}

gpio_pin_state_e gpio_read_pin(GPIO_TypeDef *port, gpio_num_e pin)
{
  gpio_pin_state_e pin_state;

  // Check the state of the pin
  if ((port->IDR & (0x1UL << pin)) != GPIO_PIN_RESET)
  {
    pin_state = GPIO_PIN_SET;   // Pin is in high state
  }
  else
  {
    pin_state = GPIO_PIN_RESET; // Pin is in low state
  }

  return pin_state;
}

void gpio_toggle_pin(GPIO_TypeDef *port, gpio_num_e pin)
{
  // Use the gpio_read_pin function to check the current state of the pin
  if (gpio_read_pin(port, pin) == GPIO_PIN_SET)
  {
    gpio_write_pin(port, pin, GPIO_PIN_RESET);
  }
  else
  {
    gpio_write_pin(port, pin, GPIO_PIN_SET);
  }
}




