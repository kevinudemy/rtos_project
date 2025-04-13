/*
 * modbus_slave_task.h
 *
 * Contains the necessary function prototypes and definitions for the Modbus Slave task.
 */

#ifndef INC_MODBUS_SLAVE_TASK_H_
#define INC_MODBUS_SLAVE_TASK_H_

#include "error.h"
#include "FreeRTOSTasks.h"
#include "queue.h"

/**
 * Enumeration of Function Code definitions.
 */
/**
 * Enumeration of Function Code definitions.
 */
typedef enum
{
  READ_COILS            = 0x01,
  READ_DISCRETE_INPUTS  = 0x02,
  READ_HOLDING_REGS     = 0x03,
  READ_INPUT_REGS       = 0x04,
  WRITE_SINGLE_REG      = 0x06,
  WRITE_HOLDING_REGS    = 0x10,
  WRITE_SINGLE_COIL     = 0x05,
  WRITE_MULTI_COILS     = 0x0F
} modbus_function_codes_e;

/**
 * Handle for the feedback queue used to retrieve the satus of requests processed by the Modbus Data Manager.
 */
extern QueueHandle_t modbus_feedback_queue_handle;

/**
 * Sends a task notification to the Modbus Slave task from an ISR to signal
 * the receipt of a Modbus RTU frame (e.g., detected via UART IDLE line).
 * @param xHigherPriorityTaskWoken Pointer updated by the xTaskNotifyFromISR() to pdTRUE
 *        if sending the notification unblocks a higher-priority task.
 */
void modbus_slave_task_send_notification_from_isr(BaseType_t *xHigherPriorityTaskWoken);

/**
 * Starts the Modbus Slave tasks and related resources.
 */
void modbus_slave_tasks_start(void);


#endif /* INC_MODBUS_SLAVE_TASK_H_ */
