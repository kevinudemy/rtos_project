/*
 * modbus_data_mgr_task.h
 *
 * Contains the necessary function prototypes and definitions for the Modbus Data Manager task.
 */

#ifndef INC_MODBUS_DATA_MGR_TASK_H_
#define INC_MODBUS_DATA_MGR_TASK_H_

#include <stdbool.h>
#include "error.h"
#include "FreeRTOSTasks.h"
#include "modbus_data.h"

/**
 * Modbus Data Manager message IDs enum.
 */
typedef enum
{
  COIL_COMMAND,
  HOLDING_REGS_UPDATE,
  INPUT_REGS_UPDATE
} modbus_data_mgr_msg_e;

/**
 * Modbus Data Manager message queue structure.
 */
typedef struct
{
  modbus_data_mgr_msg_e msg_type;
  void *data;
  uint16_t address;
  uint16_t quantity;
  bool requires_feedback;
} modbus_data_mgr_processing_msg_t;

/**
 * Feedback Message for communication from the Modbus Data Manager task to the Modbus Slave task.
 */
typedef struct
{
  error_t status;
} modbus_data_mgr_feedback_msg_t;

/**
 * Enqueues a Modbus data message for processing by the Modbus Data Manager task.
 *
 * @param msg_type Message ID from the modbus_data_mgr_msg_e enum indicating the type of update.
 * @param data Pointer to data associated with the message. Can be NULL.
 * @param addr The starting address for the Modbus data operation.
 * @param qty The number of registers or coils involved in the operation.
 * @param req_feedback Indicates whether feedback is required from the Modbus Data Manager.
 *
 * @return ERR_OK if the message was successfully enqueued to the queue, otherwise ERR_FAIL.
 */
error_t modbus_data_mgr_send_processing_msg(modbus_data_mgr_msg_e msg_type,
                                            void *data,
                                            uint16_t addr,
                                            uint16_t qty,
                                            bool req_feedback);

/**
 * Enqueues a modbus feedback message for the Modbus Slave task to receive.
 *
 * @param feedback_status Status of the operation (e.g., ERR_OK, ERR_FAIL).
 * @return ERR_OK if the message was successfully sent/enqueued to the queue, otherwise ERR_FAIL.
 */
error_t modbus_data_mgr_send_feedback_msg(error_t feedback_status);

/**
 * Starts the Modbus Data Manager task.
 */
void modbus_data_mgr_start(void);


#endif /* INC_MODBUS_DATA_MGR_TASK_H_ */
