/**
 * Application entry point.
 */

#include "gpio.h"
#include "rcc.h"
#include "error_handler_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOSTasks.h"

// Startup Task prototype
static void startup(void);

int main(void)
{
  /* Initial RCC configuration and clock frequency tests */
  uint32_t system_clock = 0;
  uint32_t hclk         = 0;
  uint32_t apb1_clock   = 0;
  uint32_t apb2_clock   = 0;

  rcc_init();

  system_clock = rcc_get_sysclk_freq();
  hclk         = rcc_get_hclk_freq();
  apb1_clock   = rcc_get_pclk1_freq();
  apb2_clock   = rcc_get_pclk2_freq();

  (void)system_clock;
  (void)hclk;
  (void)apb1_clock;
  (void)apb2_clock;

  // Update the 'SystemCoreClock' variable required by FreeRTOS
  SystemCoreClockUpdate();

  // Initialize the required GPIOs
  gpio_init();

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
  // Start the Error Handler Task
  error_handler_task_start();

  // Test for the Error Handler Task
  for (event_id_e error = EVT_SYS_HEALTH_AWDG_THRESHOLD_EXCEEDED; error < EVT_MAX; error++)
  {
    error_handler_send_msg(error);
    vTaskDelay(pdMS_TO_TICKS(1000));  // Add delay to ensure each message is processed
  }
  // After this point, you should check the error_counts array
  // to verify that each error event ID has been counted correctly.

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






























