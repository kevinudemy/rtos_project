/*
 * flash.h
 *
 * Provides function prototypes for configuring the internal flash memory of the MCU.
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "mcu.h"

/**
 * Configures the number of wait states for accessing the internal flash memory based on the CPU clock frequency (HCLK).
 * @param hclk The CPU clock frequency in MHz.
 */
void flash_config_wait_states(uint8_t hclk);

#endif /* INC_FLASH_H_ */
