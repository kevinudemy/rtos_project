/*
 * sys_health_monitor_task.h
 *
 * Provides the interface for the System Health Monitor Task.
 */

#ifndef INC_SYS_HEALTH_MONITOR_TASK_H_
#define INC_SYS_HEALTH_MONITOR_TASK_H_

#include <stdbool.h>
#include "FreeRTOSTasks.h"

/**
 * Temperature thresholds for the analog watchdog.
 */
#define ADC1_AWD_HT_TEMP_CELSIUS  80
#define ADC1_AWD_LT_TEMP_CELSIUS  -20

// Extern health flag for the sensors task
extern volatile bool g_sensors_task_ok;
//... add other flags as necessary

/**
 * Starts the System Health Monitor Task.
 */
void sys_health_monitor_task_start(void);


#endif /* INC_SYS_HEALTH_MONITOR_TASK_H_ */
