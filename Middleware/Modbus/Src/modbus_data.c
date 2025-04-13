/*
 * modbus_data.c
 *
 * Implementation for Modbus data storage and access functions.
 */

#include "fram.h"
#include "modbus_data.h"

/**
 * Initialize the Holding Register NVS structure.
 */
static modbus_nvs_holding_registers_t modbus_nvs_holding_reg_updated_data;

/**
 * Initialize the Input Register NVS structure.
 */
static modbus_nvs_input_registers_t modbus_nvs_input_reg_updated_data;

error_t modbus_data_get_holding_register(uint16_t index, uint16_t *value)
{
  error_t status = ERR_OK;

  if (index >= sizeof(modbus_holding_regs) / sizeof(modbus_holding_regs[0]))
  {
      status = MODBUS_INVALID_REG_ADDRESS;
  }
  else
  {
    *value = modbus_holding_regs[index];
  }

  return status;
}

error_t modbus_data_set_holding_register(uint16_t index, uint16_t value)
{
  error_t status = ERR_OK;

  if (index >= sizeof(modbus_holding_regs) / sizeof(modbus_holding_regs[0]))
  {
    status = MODBUS_INVALID_REG_ADDRESS;
  }
  else
  {
    modbus_holding_regs[index] = value;
  }

  return status;
}

error_t modbus_data_set_holding_register_and_data(holding_registers_index_e index, uint16_t value)
{
  error_t status = ERR_OK;

  switch (index)
  {
    case HOLDING_SENSORS_SAMPLING_INTERVAL:
        modbus_nvs_holding_reg_updated_data.holding_sensors_sampling_interval = value;
        break;
    case HOLDING_ALARM_MAX_VOC_INDEX:
        modbus_nvs_holding_reg_updated_data.holding_alarm_max_voc_index = value;
        break;
    case HOLDING_ALARM_MAX_AMB_TEMP:
        modbus_nvs_holding_reg_updated_data.holding_alarm_max_amb_temp = value;
        break;
    case HOLDING_ALARM_MIN_AMB_TEMP:
        modbus_nvs_holding_reg_updated_data.holding_alarm_min_amb_temp = value;
        break;
    case HOLDING_ALARM_MAX_HUM:
        modbus_nvs_holding_reg_updated_data.holding_alarm_max_hum = value;
        break;
    default:
        return ERR_INVALID_PARAM;
  }

  if (status == ERR_OK)
  {
    // Set the Modbus register in case the Modbus slave task didn't already do it (update according to your needs)
    status = modbus_data_set_holding_register(index, value);
  }

  return status;
}

modbus_nvs_holding_registers_t modbus_get_nvs_holding_registers(void)
{
  return modbus_nvs_holding_reg_updated_data;
}

error_t modbus_data_get_input_register(uint16_t index, uint16_t *value)
{
  error_t status = ERR_OK;

  if (index >= sizeof(modbus_input_regs) / sizeof(modbus_input_regs[0]))
  {
    status = MODBUS_INVALID_REG_ADDRESS;
  }
  else
  {
    *value = modbus_input_regs[index];
  }

  return status;
}

error_t modbus_data_set_input_register(uint16_t index, uint16_t value)
{
  error_t status = ERR_OK;

  if (index >= sizeof(modbus_input_regs) / sizeof(modbus_input_regs[0]))
  {
    status = MODBUS_INVALID_REG_ADDRESS;
  }
  else
  {
    modbus_input_regs[index] = value;
  }

  return status;
}

error_t modbus_data_set_input_register_and_data(input_registers_index_e index, uint16_t value)
{
  error_t status = ERR_OK;

  switch (index)
  {
    case INPUT_ALARM_COUNT_VOC:
      modbus_nvs_input_reg_updated_data.input_alarm_count_voc = value;
      break;
    case INPUT_ALARM_COUNT_AMB_TEMP_MAX:
      modbus_nvs_input_reg_updated_data.input_alarm_count_amb_temp_max = value;
      break;
    case INPUT_ALARM_COUNT_AMB_TEMP_MIN:
      modbus_nvs_input_reg_updated_data.input_alarm_count_amb_temp_min = value;
      break;
    case INPUT_ALARM_COUNT_HUM:
      modbus_nvs_input_reg_updated_data.input_alarm_count_hum = value;
      break;
    default:
      status = ERR_INVALID_PARAM;
  }

  if (status == ERR_OK)
  {
    // Set the Modbus register
    status = modbus_data_set_input_register(index, value);
  }

  return status;
}

modbus_nvs_input_registers_t modbus_get_nvs_input_registers(void)
{
  return modbus_nvs_input_reg_updated_data;
}

error_t modbus_data_get_coil(uint16_t index, uint8_t *value)
{
  error_t status = ERR_OK;

  if (index >= (sizeof(modbus_coils) * 8))  // 8 bits per byte
  {
      status = MODBUS_INVALID_COIL_ADDRESS;
  }
  else
  {
    uint8_t byte_index = index / 8;
    uint8_t bitIndex = index % 8;
    *value = (modbus_coils[byte_index] >> bitIndex) & 0x01;
  }

  return status;
}

error_t modbus_data_set_coil(uint16_t index, uint8_t value)
{
  error_t status = ERR_OK;

  if (index >= (sizeof(modbus_coils) * 8))
  {
    status = MODBUS_INVALID_COIL_ADDRESS;
  }
  else if (value != 0 && value != 1)
  {
    status = MODBUS_INVALID_COIL_VALUE;
  }
  else
  {
    uint8_t byte_index = index / 8;
    uint8_t bit_index = index % 8;
    if (value)
    {
      modbus_coils[byte_index] |= (1 << bit_index);  // Set the bit
    }
    else
    {
      modbus_coils[byte_index] &= ~(1 << bit_index); // Clear the bit
    }
  }

  return status;
}

error_t modbus_data_set_discrete_input(uint16_t index, uint8_t value)
{
  error_t status = ERR_OK;

  if (index >= (sizeof(modbus_discrete_inputs) * 8))
  {
    status = MODBUS_INVALID_DISC_ADDRESS;
  }
  else if (value != 0 && value != 1)
  {
    status = MODBUS_INVALID_DISC_VALUE;
  }
  else
  {
    uint8_t byte_index = index / 8;
    uint8_t bit_index = index % 8;
    if (value)
    {
      modbus_discrete_inputs[byte_index] |= (1 << bit_index);  // Set the bit
    }
    else
    {
      modbus_discrete_inputs[byte_index] &= ~(1 << bit_index); // Clear the bit
    }
  }

  return status;
}

error_t modbus_data_get_discrete_input(uint16_t index, uint8_t *value)
{
  error_t status = ERR_OK;

  if (index >= (sizeof(modbus_discrete_inputs) * 8))
  {
    status = MODBUS_INVALID_DISC_ADDRESS;
  }
  else
  {
    uint8_t byte_index = index / 8;
    uint8_t bit_index = index % 8;
    *value = (modbus_discrete_inputs[byte_index] >> bit_index) & 0x01;
  }

  return status;
}

error_t modbus_data_init_holding_registers(void)
{
  error_t status = ERR_OK;

  // Try to read the holding register data from FRAM
  status = fram_read(HOLDING_REGS_FRAM_ADDR, (uint8_t*)&modbus_nvs_holding_reg_updated_data, sizeof(modbus_nvs_holding_registers_t));

  // Check if the read operation was successful and if the data is valid
  if (status == ERR_OK && modbus_nvs_holding_reg_updated_data.holding_sensors_sampling_interval != 0x0000)  // 0x0000, would indicate uninitialized FRAM data
  {
    // Update the actual Modbus holding registers
    modbus_data_set_holding_register(HOLDING_SENSORS_SAMPLING_INTERVAL, modbus_nvs_holding_reg_updated_data.holding_sensors_sampling_interval);
    modbus_data_set_holding_register(HOLDING_ALARM_MAX_VOC_INDEX, modbus_nvs_holding_reg_updated_data.holding_alarm_max_voc_index);
    modbus_data_set_holding_register(HOLDING_ALARM_MAX_AMB_TEMP, modbus_nvs_holding_reg_updated_data.holding_alarm_max_amb_temp);
    modbus_data_set_holding_register(HOLDING_ALARM_MIN_AMB_TEMP, modbus_nvs_holding_reg_updated_data.holding_alarm_min_amb_temp);
    modbus_data_set_holding_register(HOLDING_ALARM_MAX_HUM, modbus_nvs_holding_reg_updated_data.holding_alarm_max_hum);
  }
  else
  {
    modbus_data_set_holding_register_and_data(HOLDING_SENSORS_SAMPLING_INTERVAL, DEFAULT_SAMPLING_INTERVAL);
    modbus_data_set_holding_register_and_data(HOLDING_ALARM_MAX_VOC_INDEX, DEFAULT_ALARM_MAX_VOC_INDEX);
    modbus_data_set_holding_register_and_data(HOLDING_ALARM_MAX_AMB_TEMP, DEFAULT_ALARM_MAX_AMB_TEMP);
    modbus_data_set_holding_register_and_data(HOLDING_ALARM_MIN_AMB_TEMP, DEFAULT_ALARM_MIN_AMB_TEMP);
    modbus_data_set_holding_register_and_data(HOLDING_ALARM_MAX_HUM, DEFAULT_ALARM_MAX_HUM);

    // Store the initialized structure back to FRAM
    status = fram_write(HOLDING_REGS_FRAM_ADDR,
                        (uint8_t*)&modbus_nvs_holding_reg_updated_data,
                        sizeof(modbus_nvs_holding_registers_t));
  }

  return status;
}

error_t modbus_data_init_input_registers(void)
{
  error_t status = ERR_OK;

  // Read the data from FRAM
  status = fram_read(INPUT_REGS_FRAM_ADDR, (uint8_t*)&modbus_nvs_input_reg_updated_data, sizeof(modbus_nvs_input_registers_t));

  if (status == ERR_OK)
  {
    // Update input registers with values from FRAM
    modbus_data_set_input_register(INPUT_ALARM_COUNT_VOC, modbus_nvs_input_reg_updated_data.input_alarm_count_voc);
    modbus_data_set_input_register(INPUT_ALARM_COUNT_AMB_TEMP_MAX, modbus_nvs_input_reg_updated_data.input_alarm_count_amb_temp_max);
    modbus_data_set_input_register(INPUT_ALARM_COUNT_AMB_TEMP_MIN, modbus_nvs_input_reg_updated_data.input_alarm_count_amb_temp_min);
    modbus_data_set_input_register(INPUT_ALARM_COUNT_HUM, modbus_nvs_input_reg_updated_data.input_alarm_count_hum);
  }

  return status;
}

error_t modbus_data_update_nvs_holding_regs(void)
{
    return fram_write(HOLDING_REGS_FRAM_ADDR,
                      (uint8_t*)&modbus_nvs_holding_reg_updated_data,
                      sizeof(modbus_nvs_holding_registers_t));
}

error_t modbus_data_update_nvs_input_regs(void)
{
    return fram_write(INPUT_REGS_FRAM_ADDR,
                      (uint8_t*)&modbus_nvs_input_reg_updated_data,
                      sizeof(modbus_nvs_input_registers_t));
}



