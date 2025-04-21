/*
 * modbus_data_mgr_task.c
 *
 * Contains the Modbus Data Manager task implementation.
 */

#include "modbus_data_mgr_task.h"
#include "modbus_slave_task.h"
#include "modbus_sync.h"
#include "error_handler_task.h"
#include "sensors_task.h"

#include "queue.h"

/**
 * Queue handle for the Modbus Data Manager task.
 */
static QueueHandle_t modbus_data_mgr_queue_handle;

/**
 * Processes and handles actions based on modified Modbus coils. Loops through
 * the changed coils, checks their status, and performs the corresponding actions.
 *
 * @param data_update_msg Modbus data update details for the modified coils.
 * @return ERR_OK if all coils are processed successfully, or an error code otherwise.
 */
static error_t process_modified_coils(modbus_data_mgr_processing_msg_t data_update_msg)
{
  error_t status = ERR_OK;
  uint16_t start_address = data_update_msg.address;
  uint16_t num_coils = data_update_msg.quantity;

  for (uint16_t i = 0; i < num_coils && status == ERR_OK; i++)
  {
    uint8_t coil_value;
    uint16_t current_coil_address = start_address + i;

    if (modbus_data_get_coil(current_coil_address, &coil_value) == ERR_OK)
    {
      // Take action based on the specific coil that was modified
      switch (current_coil_address)
      {
        case COIL_NO_OPERATION:
          if (coil_value)  // If this coil has been set
          {
            // TODO: Implement your functionality for when this coil is set.
            // Example: Turn on a specific feature or start a process.
          }
          else  // If this coil has been reset
          {
            // TODO: Implement your functionality for when this coil is reset.
            // Example: Turn off the feature or stop the process.
          }
          break;

        // Add new coil functionality here in the future...

        default:
          // Handle any unknown coil address or add any other necessary actions.
          status = MODBUS_INVALID_COIL_ADDRESS; // Set error code if coil address is unknown.
          break;
      }
    }
    else
    {
        status = MODBUS_INVALID_COIL_ADDRESS; // Set error code if unable to get the coil value.
    }
  }

  return status;
}

/**
 * Updates data structures and FRAM based on modified holding registers.
 * Handles additional actions such as updating the sensors' sampling interval if required.
 *
 * @param data_update_msg Modbus data update details.
 * @return ERR_OK if successful, error code otherwise.
 */
static error_t process_modified_holding_registers(modbus_data_mgr_processing_msg_t data_udpate_msg)
{
  error_t status = ERR_OK;
  uint16_t start_address = data_udpate_msg.address;
  uint16_t num_regs = data_udpate_msg.quantity;

  for (uint16_t i = 0; i < num_regs && status == ERR_OK; i++)
  {
    uint16_t register_value;
    uint16_t current_reg_address = start_address + i;

    if (modbus_data_get_holding_register(current_reg_address, &register_value) == ERR_OK)
    {
      // Update the data structure based on the modified holding register
      switch (current_reg_address)
      {
        case HOLDING_SENSORS_SAMPLING_INTERVAL:
          status = modbus_data_set_holding_register_and_data(HOLDING_SENSORS_SAMPLING_INTERVAL, register_value);
          if (status == ERR_OK)
          {
              status = sensors_task_send_msg(SAMPLING_INTERVAL_UPDATE, register_value);
          }
          break;

        case HOLDING_ALARM_MAX_VOC_INDEX:
            status = modbus_data_set_holding_register_and_data(HOLDING_ALARM_MAX_VOC_INDEX, register_value);
            break;

        case HOLDING_ALARM_MAX_AMB_TEMP:
            status = modbus_data_set_holding_register_and_data(HOLDING_ALARM_MAX_AMB_TEMP, register_value);
            break;

        case HOLDING_ALARM_MIN_AMB_TEMP:
            status = modbus_data_set_holding_register_and_data(HOLDING_ALARM_MIN_AMB_TEMP, register_value);
            break;

        case HOLDING_ALARM_MAX_HUM:
            status = modbus_data_set_holding_register_and_data(HOLDING_ALARM_MAX_HUM, register_value);
            break;

          // Expand for other holding registers as needed...

        default:
          // Handle any unknown register address or add any other necessary actions
          status = MODBUS_INVALID_REG_ADDRESS;
      }
    }
    else
    {
        status = MODBUS_INVALID_REG_ADDRESS;
    }
  }

  // Write the updated data back to FRAM once, after all modifications
  if (status == ERR_OK)
  {
    status = modbus_data_update_nvs_holding_regs();
  }

  return status;
}

/**
 * Handles the updates for sensor data and alarms.
 * - Checks if the current value exceeds the max threshold or falls below the min threshold.
 * - Updates the corresponding discrete input alarm.
 * - Increments the input alarm count if an alarm is set.
 * - Resets the alarm if the value is within the threshold.
 *
 * @param current_value The current sensor value.
 * @param threshold_value The threshold value for the alarm.
 * @param input_alarm_count Pointer to the input alarm count value.
 * @param input_index The input register index for alarm counts.
 * @param discrete_index The discrete input index for the alarm.
 * @param is_max_threshold Boolean flag to indicate max (true) or min (false) threshold check.
 * @return ERR_OK on success, appropriate error code otherwise.
 */
static error_t handle_sensor_update(uint16_t current_value,
                                    uint16_t threshold_value,
                                    uint16_t *input_alarm_count,
                                    input_registers_index_e input_index,
                                    discrete_inputs_index_e discrete_index,
                                    bool is_max_threshold)
{
  error_t status = ERR_OK;
  uint8_t discrete_alarm;

  // Get the current state of the discrete input for the alarm
  status = modbus_data_get_discrete_input(discrete_index, &discrete_alarm);

  if (status == ERR_OK)
  {
    bool alarm_triggered = (is_max_threshold) ? (current_value > threshold_value)
                                              : (current_value < threshold_value);

    // Set alarm if condition is met
    if (alarm_triggered)
    {
      if (discrete_alarm == 0)  // Alarm was not set previously
      {
        status = modbus_data_set_discrete_input(discrete_index, 1);
        if (status == ERR_OK)
        {
          // Increment the alarm count and update input register data
          (*input_alarm_count)++;
          status = modbus_data_set_input_register_and_data(input_index, *input_alarm_count);

          // Update FRAM with the current input register values
          if (status == ERR_OK)
          {
            status = modbus_data_update_nvs_input_regs();
          }
        }
      }
    }
    else if (discrete_alarm == 1) // Reset alarm if condition is back to normal
    {
      status = modbus_data_set_discrete_input(discrete_index, 0);
    }
  }

  return status;
}

/**
 * Handles updates for VOC index.
 * - Checks if the VOC index exceeds the maximum allowed value.
 * - Updates the VOC alarm and alarm count accordingly.
 *
 * @param voc_index The current VOC index.
 * @return ERR_OK on success, appropriate error code otherwise.
 */
static error_t handle_voc_index_update(uint16_t voc_index)
{
  error_t status = ERR_OK;
  uint16_t max_voc_index;

  // 1. Get max VOC index from holding registers
  status = modbus_data_get_holding_register(HOLDING_ALARM_MAX_VOC_INDEX, &max_voc_index);

  if (status == ERR_OK)
  {
    // 2. Handle the update using the helper function
    modbus_nvs_input_registers_t modbus_nvs_input_regs = modbus_get_nvs_input_registers();
    status = handle_sensor_update(voc_index,
                                  max_voc_index,
                                  &modbus_nvs_input_regs.input_alarm_count_voc,
                                  INPUT_ALARM_COUNT_VOC,
                                  DISC_VOC_ALARM,
                                  true);
  }

  return status;
}

/**
 * Handles updates for ambient temperature.
 * - Checks if the ambient temperature exceeds max or falls below min.
 * - Updates the temperature alarm and alarm count accordingly.
 *
 * @param amb_temp The current ambient temperature.
 * @return ERR_OK on success, appropriate error code otherwise.
 */
static error_t handle_amb_temp_update(uint16_t amb_temp)
{
  error_t status = ERR_OK;
  uint16_t max_amb_temp, min_amb_temp;

  // 1. Get max and min temperature from holding registers
  status = modbus_data_get_holding_register(HOLDING_ALARM_MAX_AMB_TEMP, &max_amb_temp);
  if (status == ERR_OK)
  {
    status = modbus_data_get_holding_register(HOLDING_ALARM_MIN_AMB_TEMP, &min_amb_temp);
  }

  if (status == ERR_OK)
  {
    // 2. Get the Modbus NVS structure containing stored alarm counts
    modbus_nvs_input_registers_t modbus_nvs_input_regs = modbus_get_nvs_input_registers();

    // Check for high temperature alarm
    status = handle_sensor_update(amb_temp,
                                  max_amb_temp,
                                  &modbus_nvs_input_regs.input_alarm_count_amb_temp_max,
                                  INPUT_ALARM_COUNT_AMB_TEMP_MAX,
                                  DISC_AMB_TEMP_MAX_ALARM,
                                  true);  // Max threshold

    // Check for low temperature alarm (Reversed logic for min)
    if (status == ERR_OK)
    {
      status = handle_sensor_update(amb_temp,
                                    min_amb_temp,
                                    &modbus_nvs_input_regs.input_alarm_count_amb_temp_min,
                                    INPUT_ALARM_COUNT_AMB_TEMP_MIN,
                                    DISC_AMB_TEMP_MIN_ALARM,
                                    false);  // Min threshold
    }

    // Update FRAM with the modified input register data
    if (status == ERR_OK)
    {
      status = modbus_data_update_nvs_input_regs();
    }
  }

  return status;
}

/**
 * Handles updates for humidity data.
 * - Checks if the humidity exceeds the maximum allowed value.
 * - Updates the humidity alarm and alarm count accordingly.
 *
 * @param hum_value The current humidity value.
 * @return ERR_OK on success, appropriate error code otherwise.
 */
static error_t handle_hum_update(uint16_t hum_value)
{
  error_t status = ERR_OK;
  uint16_t max_hum_index;

  // 1. Get max humidity from holding registers
  status = modbus_data_get_holding_register(HOLDING_ALARM_MAX_HUM, &max_hum_index);

  if (status == ERR_OK)
  {
    // 2. Handle the update using the helper function
    modbus_nvs_input_registers_t modbus_nvs_input_regs = modbus_get_nvs_input_registers();
    status = handle_sensor_update(hum_value,
                                  max_hum_index,
                                  &modbus_nvs_input_regs.input_alarm_count_hum,
                                  INPUT_ALARM_COUNT_HUM,
                                  DISC_HUM_ALARM,
                                  true);
  }

  return status;
}

/**
 * Processes modified Modbus input registers.
 * - Iterates through updated input registers.
 * - Updates values and takes action based on the specific register.
 *
 * @param data_udpate_msg The Modbus data update message.
 * @return ERR_OK if all registers were processed successfully, appropriate error code otherwise.
 */
static error_t process_input_registers_update(modbus_data_mgr_processing_msg_t data_udpate_msg)
{
  error_t status = ERR_OK;
  uint16_t start_address = data_udpate_msg.address;
  uint16_t num_regs = data_udpate_msg.quantity;

  for (uint16_t i = 0; i < num_regs && status == ERR_OK; i++)
  {
    uint16_t current_reg_address = start_address + i;
    uint16_t register_value = *((uint16_t*)data_udpate_msg.data + i);

    status = modbus_data_set_input_register(current_reg_address, register_value);

    if (status == ERR_OK)
    {
      switch (current_reg_address)
      {
        case INPUT_SENSOR_VOC_INDEX:
          status = handle_voc_index_update(register_value);
          break;

        case INPUT_SENSOR_AMB_TEMP:
          status = handle_amb_temp_update(register_value);
          break;

        case INPUT_SENSOR_HUM:
          status = handle_hum_update(register_value);
          break;

        default:
          // Handle any unknown register address or add any other necessary actions
          break;
      }
    }
  }

  return status;
}

/**
 * Handles incoming messages to update various Modbus data types,
 * including coils, holding registers, and input registers.
 * 1. Waits for data update messages in the queue.
 * 2. Processes updates based on the data type and takes appropriate actions.
 * 3. Sends feedback if required and handles errors during processing.
 * 4. Maintains synchronization with a Mutex to ensure data consistency.
 */
static void modbus_data_mgr_task(void *param)
{
  modbus_data_mgr_processing_msg_t data_update_msg;
  modbus_data_mgr_feedback_msg_t feedback_msg;

  while (1)
  {
    if (xQueueReceive(modbus_data_mgr_queue_handle, &data_update_msg, portMAX_DELAY))
    {
      error_t lock_status = modbus_sync_lock();
      if (lock_status == ERR_OK)
      {
        feedback_msg.status = ERR_OK;

        switch (data_update_msg.msg_type)
        {
          case COIL_COMMAND:
            feedback_msg.status = process_modified_coils(data_update_msg);
            if (feedback_msg.status != ERR_OK)
            {
              error_handler_send_msg(EVT_MODBUS_DATA_UPDATE_COILS_FAIL);
            }
            break;

          case HOLDING_REGS_UPDATE:
            feedback_msg.status = process_modified_holding_registers(data_update_msg);
            if (feedback_msg.status != ERR_OK)
            {
              error_handler_send_msg(EVT_MODBUS_DATA_UPDATE_HOLDING_REGS_FAIL);
            }
            break;

          case INPUT_REGS_UPDATE:
            feedback_msg.status = process_input_registers_update(data_update_msg);
            if (feedback_msg.status != ERR_OK)
            {
              error_handler_send_msg(EVT_MODBUS_DATA_UPDATE_INPUT_REGS_FAIL);
            }
            break;

          default:
            feedback_msg.status = ERR_FAIL;
            break;
        }

        if (data_update_msg.requires_feedback)
        {
          modbus_data_mgr_send_feedback_msg(feedback_msg.status);
        }

        modbus_sync_unlock();
      }
      else
      {
        error_handler_send_msg(EVT_MODBUS_MUTEX_TIMEOUT);
      }
    }
  }
}

error_t modbus_data_mgr_send_processing_msg(modbus_data_mgr_msg_e msg_type,
                                            void *data,
                                            uint16_t addr,
                                            uint16_t qty,
                                            bool req_feedback)
{
  error_t status = ERR_OK;

  modbus_data_mgr_processing_msg_t data_update_msg;
  data_update_msg.msg_type = msg_type;
  data_update_msg.data = data;
  data_update_msg.address = addr;
  data_update_msg.quantity = qty;
  data_update_msg.requires_feedback = req_feedback;

  if (xQueueSend(modbus_data_mgr_queue_handle, &data_update_msg, portMAX_DELAY) != pdTRUE)
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t modbus_data_mgr_send_feedback_msg(error_t feedback_status)
{
  error_t status = ERR_OK;

  modbus_data_mgr_feedback_msg_t feedback_msg;
  feedback_msg.status = feedback_status;

  if (xQueueSend(modbus_feedback_queue_handle, &feedback_msg, portMAX_DELAY) != pdTRUE)
  {
    status = ERR_FAIL;
  }

  return status;
}

void modbus_data_mgr_start(void)
{
  modbus_data_mgr_queue_handle = xQueueCreate((UBaseType_t) 10,
                                              sizeof(modbus_data_mgr_processing_msg_t));
  configASSERT(modbus_data_mgr_queue_handle != NULL);

  // Add the Modbus Data Manager Queue object to the FreeRTOS Queue registery
  vQueueAddToRegistry(modbus_data_mgr_queue_handle, "Modbus Data Mgr Queue");

  configASSERT(xTaskCreate(modbus_data_mgr_task,
                           "Modbus Data Manager Task",
                           MODBUS_DATA_MGR_TASK_STACK_SIZE,
                           NULL,
                           MODBUS_DATA_MGR_TASK_PRIORITY,
                           NULL) == pdPASS);
}




