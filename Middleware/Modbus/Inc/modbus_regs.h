/*
 * modbus_regs.h
 *
 * Contains modbus register definitions, i.e., Coils, Discrete Inputs, Input & Holding Registers,
 * and declares their respective data structures.
 */

#ifndef MODBUS_INC_MODBUS_REGS_H_
#define MODBUS_INC_MODBUS_REGS_H_

/**
 * Enumeration representing the available Modbus coil functions.
 */
typedef enum
{
  COIL_NO_OPERATION = 0,              // Placeholder for future coil functions
  COIL_MAX                            // Maximum number of Coils, serves as an upper bound
} coil_index_e;

/**
 * Enumeration representing alarms accessible through Modbus discrete inputs.
 */
typedef enum
{

  DISC_VOC_ALARM = 0,                 // Alarm indicating abnormal VOC levels
  DISC_AMB_TEMP_MAX_ALARM,            // Alarm for temperature exceeding max threshold
  DISC_AMB_TEMP_MIN_ALARM,            // Alarm for temperature falling below min threshold
  DISC_HUM_ALARM,                     // Alarm indicating abnormal humidity levels
  DISC_MAX                            // Maximum number of Discrete Inputs, serves as an upper bound
} discrete_inputs_index_e;

/**
 * Enumeration representing the available Modbus input registers for reading sensor data.
 */
typedef enum
{
  // Measurement data
  INPUT_SENSOR_VOC_INDEX = 0,         // VOC index
  INPUT_SENSOR_AMB_TEMP,              // Current ambient temperature
  INPUT_SENSOR_HUM,                   // Current humidity level

  // Alarm counts
  INPUT_ALARM_COUNT_VOC,              // Counter for VOC Index alarms
  INPUT_ALARM_COUNT_AMB_TEMP_MAX,     // Counter for Ambient Temp MAX alarms
  INPUT_ALARM_COUNT_AMB_TEMP_MIN,     // Counter for Ambient Temp MIN alarms
  INPUT_ALARM_COUNT_HUM,              // Counter for Humidity alarms

  INPUT_MAX                           // Maximum number of Input Registers, serves as an upper bound
} input_registers_index_e;

/**
 * Enumeration representing the available Modbus holding registers for configuration and settings.
 */
typedef enum
{
  HOLDING_SENSORS_SAMPLING_INTERVAL,  // Sampling interval for sensors
  HOLDING_ALARM_MAX_VOC_INDEX,        // Threshold for maximum VOC level before triggering an alarm
  HOLDING_ALARM_MAX_AMB_TEMP,         // Threshold for maximum ambient temperature before triggering an alarm
  HOLDING_ALARM_MIN_AMB_TEMP,         // Threshold for minimum ambient temperature before triggering an alarm
  HOLDING_ALARM_MAX_HUM,              // Threshold for maximum humidity level before triggering an alarm
  HOLDING_MAX                         // Maximum number of Holding Registers, serves as an upper bound
} holding_registers_index_e;

/**
 * Coils Database.
 * Represent binary states that can be read from and written to by the Modbus Master.
 * They may include controls or other binary conditions.
 */
extern uint8_t modbus_coils[(COIL_MAX + 7) / 8];

/**
 * Discrete Inputs Database.
 * Like Coils, but they are read-only for the Modbus Master.
 * They provide binary status information, such as data readiness, device status, or alarms.
 */
extern uint8_t modbus_discrete_inputs[(DISC_MAX + 7) / 8];

/**
 * Holding Registers Database.
 * These registers are used to store configuration and control settings.
 * Modbus Master can read from and write to these registers to modify sensor behavior.
 */
extern uint16_t modbus_holding_regs[HOLDING_MAX];

/**
 * Input Registers Database.
 * These registers provide a snapshot of the latest readings from the sensors.
 * They are read-only for the Modbus Master, representing various environmental metrics like temperature, humidity, etc.
 */
extern uint16_t modbus_input_regs[INPUT_MAX];


#endif /* MODBUS_INC_MODBUS_REGS_H_ */
