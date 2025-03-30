/*
 * sensors_task.h
 *
 * Declares the necessary functions, data structures, and enumerations required by the sensors task.
 */

#ifndef INC_SENSORS_TASK_H_
#define INC_SENSORS_TASK_H_

#include "error.h"
#include "FreeRTOSTasks.h"

/**
 * Sensors data structure to be populated by the sensors task.
 */
typedef struct
{
  uint16_t voc_index;
  uint16_t amb_temp;
  uint16_t hum;
} sensors_task_data_t;

/**
 * Sensors task message IDs enum.
 * Note: expand this based on your needs.
 */
typedef enum
{
  SAMPLING_INTERVAL_UPDATE
} sensors_msg_e;

/**
 * Sensors task message queue structure.
 */
typedef struct
{
  sensors_msg_e msg_id;
  uint16_t sampling_interval;
} sensors_msg_t;

/**
 * Enqueues a message for the Sensors Task.
 * @param msg_id Message ID from the sensors_msg_e enum indicating the type of update.
 * @param sampling_interval The sampling interval of the sensors task.
 * @return ERR_OK if the message was successfully sent to the queue, otherwise ERR_FAIL.
 *         ERR_FAIL indicates that the queue is full, and the message cannot be enqueued at this time.
 */
error_t sensors_task_send_msg(sensors_msg_e msg_id, uint16_t sampling_interval);

/**
 * Starts the Sensors Task.
 */
void sensors_task_start(void);


#endif /* INC_SENSORS_TASK_H_ */
