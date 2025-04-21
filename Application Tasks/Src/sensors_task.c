/*
 * sensors_task.c
 *
 * Contains the implementation of the Sensors Task, which is responsible for acquiring data
 * from various sensors at a specified interval and communicates with the Modbus Data Manager.
 */

#include "sensors_task.h"
#include "error_handler_task.h"
#include "gpio.h"
#include "modbus_data_mgr_task.h"
#include "queue.h"
#include "sys_health_monitor_task.h"
#include "sgp40_voc_index.h"
#include "system_events.h"

/**
 * Queue handle for the Sensors Task.
 */
static QueueHandle_t sensors_task_queue_handle;

/**
 * Checks if the sensors task queue has a message.
 * @return pdTRUE if there is a message in the queue, otherwise pdFALSE.
 */
static BaseType_t sensors_task_has_message(void)
{
  // Check if there are any messages in the queue without actually reading them
  if (uxQueueMessagesWaiting(sensors_task_queue_handle) > 0)
  {
    return pdTRUE;
  }
  else
  {
    return pdFALSE;
  }
}

/**
 * Manages the sensor data sampling process.
 * It initializes the sensors and sets the sampling interval based on the holding register value.
 * It also handles messages for updating the sampling interval dynamically and can be extended to handle other types of messages.
 */
static void sensors_task(void *params)
{
  // Wait for Modbus to be initialized before proceeding
  xEventGroupWaitBits(system_event_group,
                      MODBUS_INITIALIZED_BIT,
                      pdFALSE,
                      pdTRUE,
                      portMAX_DELAY);

  sensors_msg_t received_msg;

  TickType_t xLastWakeTime;
  TickType_t xFrequency;

  int32_t voc_index_sens = 0;
  int32_t amb_temp_sens = 0;
  int32_t hum_sens = 0;

  sensors_task_data_t sensors_task_data = {0};

  // Initialize the sensors and let the Error Handler know if there was an issue
  error_t sensors_task_status = sensirion_init_sensors();
  if (sensors_task_status != ERR_OK)
  {
    error_handler_send_msg(EVT_SENSOR_READ_FAIL);
  }


  // Get the sampling interval from the Modbus Holding registers and update the xFrequency variable
  uint16_t sampling_interval;
  modbus_data_get_holding_register(HOLDING_SENSORS_SAMPLING_INTERVAL, &sampling_interval);
  xFrequency = pdMS_TO_TICKS(sampling_interval);

  // Set the USER LED to ON to signify VOC sensor warmup
  gpio_write_pin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_SET);

  // Initialize the xLastWakeTime variable with the current tick count
  xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Set the flag indicating this task is running correctly
    g_sensors_task_ok = true;

    // Check if there is a message in the queue
    if (sensors_task_has_message() == pdTRUE)
    {
      if (xQueueReceive(sensors_task_queue_handle, &received_msg, portMAX_DELAY) == pdTRUE)
      {
        switch (received_msg.msg_id)
        {
          case SAMPLING_INTERVAL_UPDATE:
            xFrequency = pdMS_TO_TICKS(received_msg.sampling_interval);
            break;

          // Add cases for other message types as necessary...

          default:
            // Handle unknown message id here if necessary
            break;
        }
      }
    }

    //        1) Read VOC index, temperature and humidity
    //        2) If sensor read successful;
    //            Check if VOC sensor is warmed up (value not 0), toggle LED, update sensor task data structure, update Modbus Data Manager w/sensor data.
    //        3) Else; let the Error Handler know by sending EVT_SENSOR_READ_FAIL.

    // Read VOC index, temperature and humidity
    sensors_task_status = sensirion_measure_voc_index_with_rh_t(&voc_index_sens,
                                                                &hum_sens,
                                                                &amb_temp_sens);
    if (sensors_task_status == ERR_OK)
    {
      if (voc_index_sens != 0)
      {
        gpio_toggle_pin(USER_LED_PORT, USER_LED_PIN);

        sensors_task_data.voc_index = voc_index_sens;
        sensors_task_data.amb_temp = amb_temp_sens / 1000;
        sensors_task_data.hum = hum_sens / 1000;

        // Update the Modbus input registers with the sensor data
        sensors_task_status = modbus_data_mgr_send_processing_msg(INPUT_REGS_UPDATE,
                                                                  &sensors_task_data,
                                                                  INPUT_SENSOR_VOC_INDEX,
                                                                  3,
                                                                  false);
      }
    }
    else
    {
      error_handler_send_msg(EVT_SENSOR_READ_FAIL);
    }
  }
}

error_t sensors_task_send_msg(sensors_msg_e msg_id, uint16_t sampling_interval)
{
  error_t status = ERR_OK;

  sensors_msg_t sensors_msg;
  sensors_msg.msg_id = msg_id;
  sensors_msg.sampling_interval = sampling_interval;

  if (xQueueSend(sensors_task_queue_handle, &sensors_msg, portMAX_DELAY) != pdTRUE)
  {
    status = ERR_FAIL;
  }

  return status;
}

void sensors_task_start(void)
{
  sensors_task_queue_handle = xQueueCreate((UBaseType_t) 10, sizeof(sensors_msg_t));
  configASSERT(sensors_task_queue_handle != NULL);

  // Add the Sensors Task Queue object to the FreeRTOS Queue registery
  vQueueAddToRegistry(sensors_task_queue_handle, "Sensors Task Queue");

  configASSERT(pdPASS == xTaskCreate(sensors_task,
                                     "Sensors Task",
                                     SENSORS_TASK_STACK_SIZE,
                                     NULL,
                                     SENSORS_TASK_PRIORITY,
                                     NULL));
}




