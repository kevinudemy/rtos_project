/*
 * modbus_slave_task.c
 *
 * Contains the Modbus Slave application task implementation.
 */

#include "modbus_slave_task.h"
#include "error_handler_task.h"
// Todo: include the below once integrated/implemented
//#include "modbus_data_mgr_task.h"
#include "modbus_slave.h"
#include "modbus_sync.h"
//#include "system_events.h"
#include "uart.h"

// Initialize Modbus Slave buffers
modbus_buffers_t modbus_buffers = {0};

// Queue handle for the feedback queue, shared with the Modbus Data Manager Task
QueueHandle_t modbus_feedback_queue_handle = NULL;

// Global task handle for the Modbus Slave task
static TaskHandle_t modbus_slave_task_handle;

/**
 * Modbus Slave FreeRTOS task.
 * Handles Modbus RTU requests by processing commands from the master device.
 * This task initializes required components, waits for notifications to process incoming data,
 * validates requests, and executes appropriate actions based on function codes.
 * `Todo` comments mark sections for further development as the task is implemented step-by-step.
 */
static void modbus_slave_task(void *param)
{
  // Modbus operations return status
  error_t modbus_slave_task_status;

  // Initialize UART2 for the Modbus Slave
  uart_init(uart2);

  // Todo: Initialize the Holding Registers


  // Todo: Initialize the Input Registers


  // Enable RXNE interrupt
  uart_enable_rxne_interrupt(USART2);

  // Todo: Set Modbus initialized bit for synchronization with the Sensors Task


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
                break;

              case WRITE_HOLDING_REGS:
                modbus_slave_task_status = modbus_slave_write_holding_regs(&modbus_buffers,
                                                                           &changed_address,
                                                                           &number_of_regs_changed);
                break;

              case WRITE_SINGLE_COIL:
                modbus_slave_task_status = modbus_slave_write_single_coil(&modbus_buffers, &changed_address);
                break;

              case WRITE_MULTI_COILS:
                modbus_slave_task_status = modbus_slave_write_multi_coils(&modbus_buffers,
                                                                          &changed_address,
                                                                          &number_of_regs_changed);
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

  // Todo: Create the feedback queue


  // Todo: Start the Modbus Data Manager Task


  // Create the Modbus Slave Task
  configASSERT(xTaskCreate(modbus_slave_task,
                           "Modbus Slave Task",
                           MODBUS_SLAVE_TASK_STACK_SIZE,
                           NULL,
                           MODBUS_SLAVE_TASK_PRIORITY,
                           &modbus_slave_task_handle) == pdPASS);
}




















