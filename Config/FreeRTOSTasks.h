/*
 * FreeRTOSTasks.h
 *
 * Contains definitions of FreeRTOS tasks, prioritizing their stack sizes and
 * execution priorities. This file centralizes task configuration for ease of
 * management and clarity in the overall system design.
 */

#ifndef FREERTOSTASKS_H_
#define FREERTOSTASKS_H_

#include "FreeRTOS.h"
#include "task.h"

/**
 * The startup task is designed to initialize system components and
 * facilitate the transition to normal operational tasks. The stack size
 * of 512 is chosen for prototyping purposes and should be optimized based
 * on the actual requirements of the initialization routines. It is given
 * high priority (the highest among the tasks defined here) to ensure swift
 * system initialization. However, the task deletes itself upon completion.
 */
#define STARTUP_TASK_STACK_SIZE           (512)
#define STARTUP_TASK_PRIORITY             (configMAX_PRIORITIES - 1)

/**
 * The Handler Task handles system errors.
 * Messages are sent to this task for error handling actions, which can be expanded/customized as needed.
 * The task has a high priority due to the need for timely error detection.
 * Stack size is set at 256 for prototyping, optimize later based on usage.
 */
#define ERROR_HANDLER_TASK_STACK_SIZE     (256)
#define ERROR_HANDLER_TASK_PRIORITY       (configMAX_PRIORITIES - 2)

/**
 * The System Health Monitor Task oversees the overall health of the system.
 * Currently, this includes monitoring the MCU's internal temperature and
 * checking the status of critical tasks and resetting the watchdog timer if they are operational.
 * The task has a low priority to ensure that higher priority tasks get CPU time first,
 * preventing them from being starved and causing a system reset if they fail to update.
 * Stack size is set at 256 for prototyping; optimize later based on usage.
 */
#define SYS_HEALTH_MONITOR_TASK_STACK_SIZE  (256)
#define SYS_HEALTH_MONITOR_TASK_PRIORITY    (configMAX_PRIORITIES - 4)

/**
 * The Sensors Task is responsible for periodically acquiring data from
 * the sensors and updating the data manager. It operates at a relatively high
 * priority level to ensure timely data acquisition and processing.
 * The stack size is initially set to 512 for prototyping; optimize later based on usage.
 */
#define SENSORS_TASK_STACK_SIZE               (512)
#define SENSORS_TASK_PRIORITY                 (configMAX_PRIORITIES - 3)


#endif /* FREERTOSTASKS_H_ */
