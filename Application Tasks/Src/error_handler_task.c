/*
 * error_handler_task.c
 *
 * Contains the function definitions and task logic for the error handler.
 */

#include <stdbool.h>
#include "error_handler_task.h"
#include "gpio.h"
#include "queue.h"

/**
 * Queue handle used by the Error Handler task.
 */
static QueueHandle_t error_handler_queue_handle = NULL;

/**
 * Error counter used for debugging purposes.
 */
static uint32_t error_counts[EVT_MAX] = {0};

/**
 * Tracks active state of LED blinking.
 */
static bool is_blinking_active = false;

/**
 * Blinks the USER LED to indicate an error.
 */
static void error_handler_led_blink(void)
{
  if (!is_blinking_active)
  {
    is_blinking_active = true;

    for (int i = 0; i < EVT_BLINK_NUMBER; i++)
    {
      gpio_toggle_pin(USER_LED_PORT, USER_LED_PIN);
      vTaskDelay(pdMS_TO_TICKS(EVT_BLINK_DELAY_MS));
      gpio_toggle_pin(USER_LED_PORT, USER_LED_PIN);
      vTaskDelay(pdMS_TO_TICKS(EVT_BLINK_DELAY_MS));
    }

    // Leave the LED ON after blinking to indicate that we need to check the error
    gpio_write_pin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_SET);
    is_blinking_active = false;
  }
}

/**
 * The Error Handler Task awaits error messages and responds accordingly.
 */
static void error_handler_task(void *param)
{
  event_id_e received_error;

  while (1)
  {
    if (xQueueReceive(error_handler_queue_handle, &received_error, portMAX_DELAY))
    {
      // Increment the count for the received error
      if (received_error < EVT_MAX)
      {
        error_counts[received_error]++;
      }

      switch (received_error)
      {
        // Analog watchdog threshold error
        case EVT_SYS_HEALTH_AWDG_THRESHOLD_EXCEEDED:

        // Sensor read error
        case EVT_SENSOR_READ_FAIL:

        // FRAM initialization error
        case EVT_FRAM_INIT_FAIL:

        // Modbus Mutex errors
        case EVT_MODBUS_MUTEX_NOT_CREATED:
        case EVT_MODBUS_MUTEX_TIMEOUT:

        // Modbus Middleware UART Tx error
        case EVT_MODBUS_UART_TX_ERROR:

        // Modbus Slave errors
        case EVT_MODBUS_SLAVE_WRITE_HOLDING_REGS_FAIL:
        case EVT_MODBUS_SLAVE_WRITE_COILS_FAIL:
        case EVT_MODBUS_SLAVE_CRC_MISMATCH:

        // Modbus internal data update errors
        case EVT_MODBUS_DATA_UPDATE_COILS_FAIL:
        case EVT_MODBUS_DATA_UPDATE_HOLDING_REGS_FAIL:
        case EVT_MODBUS_DATA_UPDATE_INPUT_REGS_FAIL:

          // Blink the USER LED in a specific pattern for all of the above error types
          error_handler_led_blink();
          break;

        default:
          break;
      }
    }
  }
}

void error_handler_send_msg(event_id_e error)
{
  xQueueSend(error_handler_queue_handle, &error, portMAX_DELAY);
}

void error_handler_send_msg_from_isr(event_id_e error)
{
  BaseType_t pxHigherPriorityTaskWoken = pdFALSE; // This will get set to pdTRUE if a context switch is required

  xQueueSendFromISR(error_handler_queue_handle, &error, &pxHigherPriorityTaskWoken);

  // Force a context switch if pxHigherPriorityTaskWoken was set to true inside of xQueueSendFromISR()
  portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

void error_handler_task_start(void)
{
  // Create the message queue that can store up to 10 error_id_e's
  error_handler_queue_handle = xQueueCreate((UBaseType_t) 10, sizeof(event_id_e));

  // Add the Error Handler Queue object to the FreeRTOS Queue registery
  vQueueAddToRegistry(error_handler_queue_handle, "Error Handler Queue");

  // If queue creation failed, handle as needed, maybe halt the system here!
  if (error_handler_queue_handle == NULL)
  {
    while (1);
  }

  // Create the task
  configASSERT(pdPASS == xTaskCreate(error_handler_task,
                                     "Error Handler Task",
                                     ERROR_HANDLER_TASK_STACK_SIZE,
                                     NULL,
                                     ERROR_HANDLER_TASK_PRIORITY,
                                     NULL));
}







