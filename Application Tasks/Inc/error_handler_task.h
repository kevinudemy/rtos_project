/*
 * error_handler_task.h
 *
 * Contains the function prototypes and error enumeration for the error handler task.
 */

#ifndef INC_ERROR_HANDLER_TASK_H_
#define INC_ERROR_HANDLER_TASK_H_

#include "FreeRTOSTasks.h"

/**
 * Defines the number of LED blink cycles for an error event.
 */
#define EVT_BLINK_NUMBER      (3)

/**
 * Delay in milliseconds for each LED blink phase.
 */
#define EVT_BLINK_DELAY_MS    (100)

/**
 * Error event IDs.
 */
typedef enum
{
  // System Health Analog Watchdog threshold error
  EVT_SYS_HEALTH_AWDG_THRESHOLD_EXCEEDED,

  // Sensor read error
  EVT_SENSOR_READ_FAIL,

  // FRAM initialization error
  EVT_FRAM_INIT_FAIL,

  // Modbus Mutex errors
  EVT_MODBUS_MUTEX_NOT_CREATED,
  EVT_MODBUS_MUTEX_TIMEOUT,

  // Modbus Middleware UART Tx error
  EVT_MODBUS_UART_TX_ERROR,

  // Modbus Slave errors
  EVT_MODBUS_SLAVE_WRITE_HOLDING_REGS_FAIL,
  EVT_MODBUS_SLAVE_WRITE_COILS_FAIL,
  EVT_MODBUS_SLAVE_CRC_MISMATCH,

  // Modbus internal data update errors
  EVT_MODBUS_DATA_UPDATE_COILS_FAIL,
  EVT_MODBUS_DATA_UPDATE_HOLDING_REGS_FAIL,
  EVT_MODBUS_DATA_UPDATE_INPUT_REGS_FAIL,

  // Total number of error codes
  EVT_MAX
} event_id_e;

/**
 * Sends an error message to the Error Handler by passing the Error ID.
 * @param error The enumerated error ID code.
 */
void error_handler_send_msg(event_id_e error);

/**
 * Sends an error message from an ISR to the Error Handler by passing the Error ID.
 * @param error The enumerated error ID code.
 */
void error_handler_send_msg_from_isr(event_id_e error);

/**
 * Starts the Error Handler Task.
 */
void error_handler_task_start(void);


#endif /* INC_ERROR_HANDLER_TASK_H_ */
