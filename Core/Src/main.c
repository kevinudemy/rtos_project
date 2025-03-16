/**
 * Application entry point.
 */

#include "button.h"
#include "gpio.h"
#include "irq.h"
#include "rcc.h"
#include "error_handler_task.h"
#include "sys_health_monitor_task.h"
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

  // Set IRQ priorities
  irq_set_priorities();

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
  // Initialize the USER button
  button_init();

  // Check if the IWDG caused a reset
  button_check_and_acknowledge_iwdg_event();

  // Start the Error Handler Task
  error_handler_task_start();

  // Start the System Health Monitor Task
  sys_health_monitor_task_start();

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






























