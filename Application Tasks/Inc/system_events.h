/*
 * system_events.h
 *
 * Header file containing event group handle and bit definitions which can be expanded as needed.
 */

#ifndef INC_SYSTEM_EVENTS_H_
#define INC_SYSTEM_EVENTS_H_

#include "FreeRTOS.h"
#include "event_groups.h"

extern EventGroupHandle_t system_event_group;

#define MODBUS_INITIALIZED_BIT  (1 << 0)
#define SENSORS_INITIALIZED_BIT (1 << 1)  // Example for further use
#define OTHER_INITIALIZED_BIT   (1 << 2)  // Example for further use


#endif /* INC_SYSTEM_EVENTS_H_ */
