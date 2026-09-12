/*
 * FreeRTOSTasks.h
 *
 * Contains FreeRTOS task configuration, including stack depths and
 * execution priorities. Centralizing these definitions makes the task
 * configuration easier to manage and review.
 */

#ifndef FREERTOSTASKS_H_
#define FREERTOSTASKS_H_

#include "FreeRTOS.h"
#include "task.h"

/**
 * The Startup Task initializes the system components required before
 * normal application operation begins.
 *
 * It is assigned the highest application task priority so initialization
 * can complete promptly. The task deletes itself after startup is complete.
 *
 * The stack depth is set to 512 for prototyping and can be adjusted later
 * based on the measured requirements of the initialization routines.
 */
#define STARTUP_TASK_STACK_SIZE            (512)
#define STARTUP_TASK_PRIORITY              (configMAX_PRIORITIES - 1)

/**
 * The Error Handler Task processes system error messages and performs
 * the associated error-handling actions.
 *
 * It is assigned a high priority so error conditions can be handled
 * promptly.
 *
 * The stack depth is set to 256 for prototyping and can be adjusted later
 * based on measured usage.
 */
#define ERROR_HANDLER_TASK_STACK_SIZE      (256)
#define ERROR_HANDLER_TASK_PRIORITY        (configMAX_PRIORITIES - 2)

/**
 * The System Health Monitor Task supervises the overall health of the system.
 *
 * This includes monitoring the MCU's internal temperature, checking the
 * status of critical tasks, and refreshing the watchdog when the required
 * conditions are satisfied.
 *
 * Its priority is lower than the Sensors Task and Error Handler Task, while
 * remaining higher than the Modbus tasks so system supervision is performed
 * regularly without unnecessarily delaying higher-priority processing.
 *
 * The stack depth is set to 256 for prototyping and can be adjusted later
 * based on measured usage.
 */
#define SYS_HEALTH_MONITOR_TASK_STACK_SIZE (256)
#define SYS_HEALTH_MONITOR_TASK_PRIORITY   (configMAX_PRIORITIES - 4)

/**
 * The Sensors Task periodically acquires sensor data and forwards the
 * updated values to the Modbus data manager.
 *
 * It is assigned a relatively high priority to support timely sensor
 * acquisition and processing.
 *
 * The stack depth is set to 512 for prototyping and can be adjusted later
 * based on measured usage.
 */
#define SENSORS_TASK_STACK_SIZE            (512)
#define SENSORS_TASK_PRIORITY              (configMAX_PRIORITIES - 3)

/**
 * The Modbus Slave Task handles communication with the Modbus master,
 * including processing requests and managing data transfers.
 *
 * Its priority is lower than the Sensors Task and System Health Monitor Task,
 * allowing sensor acquisition and system supervision to take precedence.
 *
 * The stack depth is set to 512 for prototyping and can be adjusted later
 * based on measured usage.
 */
#define MODBUS_SLAVE_TASK_STACK_SIZE       (512)
#define MODBUS_SLAVE_TASK_PRIORITY         (configMAX_PRIORITIES - 5)

/**
 * The Modbus Data Manager Task processes data updates associated with
 * Modbus registers based on inputs from the Modbus Slave Task and
 * Sensors Task.
 *
 * It is assigned a lower priority than the Modbus Slave Task so the slave
 * task can respond promptly to incoming Modbus requests.
 *
 * The stack depth is set to 512 for prototyping and can be adjusted later
 * based on measured usage.
 */
#define MODBUS_DATA_MGR_TASK_STACK_SIZE    (512)
#define MODBUS_DATA_MGR_TASK_PRIORITY      (configMAX_PRIORITIES - 6)

#endif /* FREERTOSTASKS_H_ */




