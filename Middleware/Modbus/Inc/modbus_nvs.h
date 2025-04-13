/*
 * modbus_nvs.h
 *
 * Contains Modbus structure members which require storage in FRAM and their respective addresses.
 */

#ifndef MODBUS_INC_MODBUS_NVS_H_
#define MODBUS_INC_MODBUS_NVS_H_

#include <stdint.h>
#include "fram.h"

/**
 * Holding register data structure used for updating to/from FRAM.
 */
typedef struct
{
  uint16_t holding_sensors_sampling_interval;
  uint16_t holding_alarm_max_voc_index;
  uint16_t holding_alarm_max_amb_temp;
  uint16_t holding_alarm_min_amb_temp;
  uint16_t holding_alarm_max_hum;
} modbus_nvs_holding_registers_t;

/**
 * Input register data structure used for updating to/from FRAM.
 */
typedef struct
{
  uint16_t input_alarm_count_voc;
  uint16_t input_alarm_count_amb_temp_max;
  uint16_t input_alarm_count_amb_temp_min;
  uint16_t input_alarm_count_hum;
} modbus_nvs_input_registers_t;

/**
 * FRAM addresses where each Modbus Structure will be stored.
 */
#define HOLDING_REGS_FRAM_ADDR      (FRAM_ADDR_START)
#define INPUT_REGS_FRAM_ADDR        (HOLDING_REGS_FRAM_ADDR + sizeof(modbus_nvs_holding_registers_t))


#endif /* MODBUS_INC_MODBUS_NVS_H_ */
