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

#endif /* FREERTOSTASKS_H_ */
