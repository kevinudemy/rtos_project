/*
 * modbus_sync.h
 *
 * Provides synchronization function prototypes for locking and unlocking access
 * to shared Modbus registers, ensuring data consistency during concurrent operations.
 */

#ifndef INC_MODBUS_SYNC_H_
#define INC_MODBUS_SYNC_H_

#include "error.h"
#include "FreeRTOS.h"

/**
 * Modbus mutex timeout in ms.
 */
#define MODBUS_MUTEX_TIMEOUT_MS     100

/**
 * Modbus mutex timeout in FreeRTOS ticks.
 */
#define MODBUS_MUTEX_TIMEOUT_TICKS  pdMS_TO_TICKS(MODBUS_MUTEX_TIMEOUT_MS)

/**
 * Creates the semaphore used for Modbus data synchronization.
 * @return ERR_OK if successful, ERR_FAIL otherwise.
 */
error_t modbus_sync_create(void);

/**
 * Locks the semaphore to safely access Modbus data.
 * Waits for up to MODBUS_MUTEX_TIMEOUT_TICKS for the semaphore to become available.
 * @return ERR_OK if successful, ERR_MUTEX_TIMEOUT if timed out, ERR_MUTEX_NOT_CREATED if the mutex was not created.
 */
error_t modbus_sync_lock(void);

/**
 * Unlocks the semaphore after accessing Modbus data.
 * @return ERR_OK if successful, ERR_MUTEX_NOT_CREATED if mutex is not created.
 */
error_t modbus_sync_unlock(void);


#endif /* INC_MODBUS_SYNC_H_ */
