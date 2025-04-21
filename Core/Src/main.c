/**
 * Application entry point.
 */

#include <string.h>
#include "button.h"
#include "dma.h"
#include "gpio.h"
#include "irq.h"
#include "rcc.h"
#include "tim.h"
#include "error_handler_task.h"
#include "fram.h"
#include "modbus_slave_task.h"
#include "sensors_task.h"
#include "sys_health_monitor_task.h"
#include "system_events.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOSTasks.h"

// System Events Group Handle
EventGroupHandle_t system_event_group;

// FreeRTOS Run Time Stats global variable and function prototypes, used for configGENERATE_RUNTIME_STATS
volatile uint32_t RTOS_RunTimeCounter;

// Startup Task prototype
static void startup(void);

// FRAM test prototype
static void fram_test(void);

// Run Time stats configuration prototype
void configureRunTime(void);

// Run Time stats get counter value prototype
uint32_t getRunTimeCounter(void);

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

  // Initialize DMA
  dma_init();

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
  // Create the Event Group
  system_event_group = xEventGroupCreate();
  configASSERT(system_event_group != NULL);

  // Initialize the USER button
  button_init();

  // Check if the IWDG caused a reset
  button_check_and_acknowledge_iwdg_event();

  // Initialize FRAM
  fram_init();

  // Run FRAM test
  fram_test();

  // Start the Error Handler Task
  error_handler_task_start();

  // Start the Sensors Task
  sensors_task_start();

  // Start the System Health Monitor Task
  sys_health_monitor_task_start();

  // Start the Modbus Tasks
  modbus_slave_tasks_start();

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

/**
 * Basic FRAM validation test.
 *
 * This test writes a predefined pattern to the FRAM, reads it back,
 * and verifies correctness using a simple memory comparison.
 *
 * In a production-ready system, this test could be expanded to include
 * writing to larger portions of memory or testing across the end address.
 * Additionally, an alert or error-handling mechanism (such as logging if available
 * or triggering a system fault response) could be integrated into a startup test sequence.
 */
static void fram_test(void)
{
  uint8_t write_data[] = {0xAB, 0xCD, 0xEF, 0x12};  // Test pattern
  uint8_t read_data[sizeof(write_data)] = {0};
  uint16_t test_address = FRAM_ADDR_START + 500;

  error_t status;

  // Write test data
  status = fram_write(test_address, write_data, sizeof(write_data));
  if (status != ERR_OK)
  {
    while (1);
  }

  // Read back data
  status = fram_read(test_address, read_data, sizeof(read_data));
  if (status != ERR_OK)
  {
    while (1);
  }

  // Validate data
  if (memcmp(write_data, read_data, sizeof(write_data)) != 0)
  {
    while (1);
  }
}

/**
 * RTOS Run time stats function which sets up the timer
 * and initializes the run time counter.
 */
void configureRunTime(void)
{
  tim3_init();
  RTOS_RunTimeCounter = 0;
}

/**
 * RTOS Run time which returns the run time counter value
 * that is updated in the TIMx interrupt handler.
 */
uint32_t getRunTimeCounter(void)
{
  return RTOS_RunTimeCounter;
}

/**
 * TIM3 interrupt handler for FreeRTOS runtime stats.
 * Increments the run-time counter and manages the TIM3 overflow event.
 */
void TIM3_IRQHandler(void)
{
  // Check whether an overflow event has taken place
  if ((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF)
  {
    // Update counter value for runtime stats
    RTOS_RunTimeCounter++;

    gpio_toggle_pin(TEST_PORT, TEST_PIN);

    // Clear the UIF to prevent immediate reetrance
    TIM3->SR &= ~(TIM_SR_UIF);
  }
}






