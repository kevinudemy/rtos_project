/**
 * Application entry point.
 */

#include "mcu.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOSTasks.h"

// Startup Task prototype
static void startup(void);

int main(void)
{
  // Update the 'SystemCoreClock' variable required by FreeRTOS
  SystemCoreClockUpdate();

  // Create the startup task
  startup();

  // Start the FreeRTOS scheduler
  vTaskStartScheduler();

  while (1)
  {

  }
}

/**
 * Startup task used for the required FreeRTOS initializations on startup.
 */
static void startup_task(void *param)
{
  // Delete the startup task
  vTaskDelete(NULL);
}

/**
 * Creates the startup task.
 */
static void startup(void)
{
  configASSERT(pdPASS == xTaskCreate(startup_task, "Startup Task", STARTUP_TASK_STACK_SIZE, NULL, STARTUP_TASK_PRIORITY, NULL));
}
