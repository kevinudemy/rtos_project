/*
 * modbus_data.h
 *
 * Header file for Modbus data storage and access functions,
 * and contains default Settings for Holding Register data.
 */

#ifndef MODBUS_INC_MODBUS_DATA_H_
#define MODBUS_INC_MODBUS_DATA_H_

#include <stdint.h>
#include "modbus_nvs.h"
#include "modbus_regs.h"
#include "error.h"

/**
 * Default settings used for Holding Register data.
 * @Note: Can be moved to a file specific to handling default data if necessary.
 */
#define DEFAULT_SAMPLING_INTERVAL     (2000)
#define DEFAULT_ALARM_MAX_VOC_INDEX   (250)
#define DEFAULT_ALARM_MAX_AMB_TEMP    (50)
#define DEFAULT_ALARM_MIN_AMB_TEMP    (0)
#define DEFAULT_ALARM_MAX_HUM         (75)

/**
 * Retrieves the value of a specified holding register.
 * @param index the index of the holding register.
 * @param value pointer to store the retrieved value.
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_get_holding_register(uint16_t index, uint16_t *value);

/**
 * Sets the value of a specified holding register.
 * @param index the index of the holding register.
 * @param value the value to be set.
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_set_holding_register(uint16_t index, uint16_t value);

/**
 * Sets the value of the specified holding register structure member and
 * holding registers by calling modbus_data_set_holding_register().
 * @param index the index of the holding register.
 * @param value the value to be set.
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_set_holding_register_and_data(holding_registers_index_e index, uint16_t value);

/**
 * Retrieves the current state of the holding registers in NVS.
 * @return input_registers_t The current input register values.
 */
modbus_nvs_holding_registers_t modbus_get_nvs_holding_registers(void);

/**
 * Retrieves the value of a specified input register.
 * @param index the index of the input register.
 * @param value pointer to store the retrieved value.
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_get_input_register(uint16_t index, uint16_t *value);

/**
 * Sets the value of a specified input register.
 * @param index the index of the input register.
 * @param value the value to be set.
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_set_input_register(uint16_t index, uint16_t value);

/**
 * Sets the value of the specified input register structure member and
 * input registers by calling modbus_data_set_input_register().
 * @param index the index of the input register.
 * @param value the value to be set.
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_set_input_register_and_data(input_registers_index_e index, uint16_t value);

/**
 * Retrieves the current state of the input registers in NVS.
 * @return input_registers_t The current input register values.
 */
modbus_nvs_input_registers_t modbus_get_nvs_input_registers(void);

/**
 * Retrieves the value of a specified coil.
 * @param index the index of the coil.
 * @param value pointer to store the retrieved coil value (0 or 1).
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_get_coil(uint16_t index, uint8_t *value);

/**
 * Sets the value of a specified coil.
 * @param index the index of the coil.
 * @param value the value to be set (0 or 1).
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_set_coil(uint16_t index, uint8_t value);

/**
 * Sets the value of a specified discrete input.
 * @param The address or index of the discrete input to be set.
 * @param The value to set (0 or 1).
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_set_discrete_input(uint16_t index, uint8_t value);

/**
 * Retrieves the value of a specific discrete input.
 * @param index the index of the discrete input.
 * @param value pointer to store the retrieved discrete input value (0 or 1).
 * @return ERR_OK if successful, otherwise relevant error code.
 */
error_t modbus_data_get_discrete_input(uint16_t index, uint8_t *value);

/**
 * Initializes the Modbus holding registers and its data structure from FRAM if previously stored.
 * If it's the first run or the FRAM was cleared, it initializes the holding
 * registers and its data structure with default values and stores them back to FRAM.
 *
 * @return error_t Returns ERR_OK if initialization was successful.
 *                 Returns appropriate error codes otherwise.
 */
error_t modbus_data_init_holding_registers(void);

/**
 * Initialize the Input Registers based on the data in FRAM.
 *
 * Reads from FRAM and updates the corresponding input registers
 * regardless of the value. This ensures that on a first run or post-clear,
 * the registers will be initialized to zero, and for subsequent runs,
 * they will be initialized to the stored historical values.
 *
 * @return error_t Returns ERR_OK if initialization was successful.
 *                 Returns appropriate error codes otherwise.
 */
error_t modbus_data_init_input_registers(void);

/**
 * Updates the FRAM storage for the Holding Register structure.
 * Ensures that the current values are written back to non-volatile memory.
 *
 * @return ERR_OK on success, otherwise appropriate error code.
 */
error_t modbus_data_update_nvs_holding_regs(void);

/**
 * Updates the FRAM storage for the Input Register structure.
 * Ensures that the current values are written back to non-volatile memory.
 *
 * @return ERR_OK on success, otherwise appropriate error code.
 */
error_t modbus_data_update_nvs_input_regs(void);


#endif /* MODBUS_INC_MODBUS_DATA_H_ */




