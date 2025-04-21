/*
 * modbus_slave_task.c
 *
 * Contains the Modbus Slave application task implementation.
 */

#include "modbus_slave_task.h"
#include "error_handler_task.h"
#include "modbus_data_mgr_task.h"
#include "modbus_slave.h"
#include "modbus_sync.h"
#include "system_events.h"
#include "uart.h"

// Initialize Modbus Slave buffers
modbus_buffers_t modbus_buffers = {0};

// Queue handle for the feedback queue, shared with the Modbus Data Manager Task
QueueHandle_t modbus_feedback_queue_handle = NULL;

// Global task handle for the Modbus Slave task
static TaskHandle_t modbus_slave_task_handle;

/**
 * Handles the status of a Modbus operation, unlocking the mutex if successful,
 * notifying the Modbus Data Manager, and waiting for feedback before sending a response.
 * @param status Result of the Modbus operation (success or error).
 * @param msg_type Type of Modbus data update to process.
 * @param address Modbus address involved.
 * @param quantity Number of registers or coils affected.
 *
 * @return ERR_OK if successful, otherwise ERR_FAIL.
 */
static error_t handle_modbus_status_and_send_data_update(error_t status,
                                                         modbus_data_mgr_msg_e msg_type,
                                                         uint16_t address,
                                                         uint16_t quantity)
{
  error_t result;

  if (status == ERR_OK)
  {
    modbus_sync_unlock();

    result = modbus_data_mgr_send_processing_msg(msg_type, NULL, address, quantity, true);

    // Wait for feedback after sending the data update
    if (result == ERR_OK)
    {
      modbus_data_mgr_feedback_msg_t feedback;
      if (xQueueReceive(modbus_feedback_queue_handle, &feedback, pdMS_TO_TICKS(100)) && feedback.status != ERR_OK)
      {
        modbus_slave_exception(SLAVE_DEVICE_FAILURE);
        result = ERR_FAIL;
      }
      else
      {
        result = modbus_slave_prepare_and_send_response(&modbus_buffers);
      }
    }
  }
  else  // Handle error scenarios for different message types
  {
    switch (msg_type)
    {
      case HOLDING_REGS_UPDATE:
        error_handler_send_msg(EVT_MODBUS_SLAVE_WRITE_HOLDING_REGS_FAIL);
        break;

      case COIL_COMMAND:
        error_handler_send_msg(EVT_MODBUS_SLAVE_WRITE_COILS_FAIL);
        break;

      default:
        break;
    }
    result = ERR_FAIL;
  }

  return result;
}

/**
 * Modbus Slave FreeRTOS task.
 * Handles Modbus RTU requests by processing commands from the master device.
 * This task initializes required components, waits for notifications to process incoming data,
 * validates requests, and executes appropriate actions based on function codes.
 */
static void modbus_slave_task(void *param)
{
  // Modbus operations return status
  error_t modbus_slave_task_status;

  // Initialize UART2 for the Modbus Slave
  uart_init(uart2);

  // Initialize the Holding Registers
  modbus_slave_task_status = modbus_data_init_holding_registers();
  if (modbus_slave_task_status != ERR_OK)
  {
    error_handler_send_msg(EVT_FRAM_INIT_FAIL);
  }


  // Initialize the Input Registers
  modbus_slave_task_status = modbus_data_init_input_registers();
  if (modbus_slave_task_status != ERR_OK)
  {
    error_handler_send_msg(EVT_FRAM_INIT_FAIL);
  }

  // Enable RXNE interrupt
  uart_enable_rxne_interrupt(USART2);

  // Set Modbus initialized bit for synchronization with the Sensors Task
  xEventGroupSetBits(system_event_group, MODBUS_INITIALIZED_BIT);


  // Address and quantity to pass to the Modbus Data Manager Task
  uint16_t changed_address, number_of_regs_changed;

  while (1)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
    {
      uint16_t message_length = modbus_buffers.rx_byte_num - 2;
      uint16_t received_crc = (modbus_buffers.rx_data[message_length + 1] << 8) |
                              modbus_buffers.rx_data[message_length];

      uint16_t calculated_crc = modbus_crc16(modbus_buffers.rx_data, message_length);

      if ((received_crc == calculated_crc) && (modbus_buffers.rx_byte_num >= MODBUS_MIN_MSG_LEN))
      {
        if (modbus_buffers.rx_data[SLAVE_ID_IDX] == SLAVE_ID)
        {
          error_t lock_status = modbus_sync_lock();
          if (lock_status == ERR_OK)
          {
            switch (modbus_buffers.rx_data[FUNC_CODE_IDX])
            {
              case READ_HOLDING_REGS:
                modbus_slave_task_status = modbus_slave_read_holding_regs(&modbus_buffers);
                break;

              case READ_INPUT_REGS:
                modbus_slave_task_status = modbus_slave_read_input_regs(&modbus_buffers);
                break;

              case READ_COILS:
                modbus_slave_task_status = modbus_slave_read_coils(&modbus_buffers);
                break;

              case READ_DISCRETE_INPUTS:
                modbus_slave_task_status = modbus_slave_read_discrete_inputs(&modbus_buffers);
                break;

              case WRITE_SINGLE_REG:
                modbus_slave_task_status = modbus_slave_write_single_reg(&modbus_buffers, &changed_address);
                handle_modbus_status_and_send_data_update(modbus_slave_task_status,
                                                          HOLDING_REGS_UPDATE,
                                                          changed_address,
                                                          1);
                break;

              case WRITE_HOLDING_REGS:
                modbus_slave_task_status = modbus_slave_write_holding_regs(&modbus_buffers,
                                                                           &changed_address,
                                                                           &number_of_regs_changed);
                handle_modbus_status_and_send_data_update(modbus_slave_task_status,
                                                          HOLDING_REGS_UPDATE,
                                                          changed_address,
                                                          number_of_regs_changed);
                break;

              case WRITE_SINGLE_COIL:
                modbus_slave_task_status = modbus_slave_write_single_coil(&modbus_buffers, &changed_address);
                handle_modbus_status_and_send_data_update(modbus_slave_task_status,
                                                          COIL_COMMAND,
                                                          changed_address,
                                                          1);
                break;

              case WRITE_MULTI_COILS:
                modbus_slave_task_status = modbus_slave_write_multi_coils(&modbus_buffers,
                                                                          &changed_address,
                                                                          &number_of_regs_changed);
                handle_modbus_status_and_send_data_update(modbus_slave_task_status,
                                                          COIL_COMMAND,
                                                          changed_address,
                                                          number_of_regs_changed);
                break;

              default:
                modbus_slave_task_status = modbus_slave_exception(ILLEGAL_FUNCTION);
                break;
            }

            modbus_sync_unlock();
          }
          else
          {
            error_handler_send_msg(EVT_MODBUS_MUTEX_TIMEOUT);
          }
        }
      }

      modbus_buffers.rx_byte_num = 0;
      uart_enable_rxne_interrupt(USART2);
    }
  }
}

void modbus_slave_task_send_notification_from_isr(BaseType_t *xHigherPriorityTaskWoken)
{
  vTaskNotifyGiveFromISR(modbus_slave_task_handle, xHigherPriorityTaskWoken);
}

void modbus_slave_tasks_start(void)
{
  // Create the synchronization mutex
  configASSERT(modbus_sync_create() == ERR_OK);

  // Create the feedback queue
  modbus_feedback_queue_handle = xQueueCreate(10, sizeof(modbus_data_mgr_feedback_msg_t));
  configASSERT(modbus_feedback_queue_handle != NULL);

  // Add the Modbus Feedback Queue object to the FreeRTOS Queue registery
  vQueueAddToRegistry(modbus_feedback_queue_handle, "Modbus Slave Feedback Queue");

  // Start the Modbus Data Manager Task
  modbus_data_mgr_start();

  // Create the Modbus Slave Task
  configASSERT(xTaskCreate(modbus_slave_task,
                           "Modbus Slave Task",
                           MODBUS_SLAVE_TASK_STACK_SIZE,
                           NULL,
                           MODBUS_SLAVE_TASK_PRIORITY,
                           &modbus_slave_task_handle) == pdPASS);
}




















