/*
 * modbus_sync.c
 *
 * Contains the synchronization function definitions for locking and unlocking access
 * to shared Modbus registers, ensuring data consistancy during concurrent operations.
 */

#include "modbus_sync.h"
#include "semphr.h"

static SemaphoreHandle_t modbus_mutex = NULL;

error_t modbus_sync_create(void)
{
  error_t status = ERR_OK;

  modbus_mutex = xSemaphoreCreateMutex();

  if (modbus_mutex == NULL)
  {
    status = ERR_FAIL;
  }

  return status;
}

error_t modbus_sync_lock(void)
{
  error_t status = ERR_OK;

  if (modbus_mutex == NULL)
  {
    status = MODBUS_MUTEX_NOT_CREATED;
  }
  else if (xSemaphoreTake(modbus_mutex, MODBUS_MUTEX_TIMEOUT_TICKS) != pdTRUE)
  {
    status = MODBUS_MUTEX_TIMEOUT;
  }

  return status;
}

error_t modbus_sync_unlock(void)
{
  error_t status = ERR_OK;

  if (modbus_mutex == NULL)
  {
    status = MODBUS_MUTEX_NOT_CREATED;
  }
  else if (xSemaphoreGive(modbus_mutex) != pdTRUE)
  {
    status = MODBUS_MUTEX_UNLOCK_FAIL;
  }

  return status;
}






