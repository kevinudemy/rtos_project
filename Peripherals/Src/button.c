/*
 * button.c
 *
 * Implementation for initializing and managing the Nucleo Board USER button for IWDG acknowledgement.
 */

#include "button.h"
#include "exti.h"
#include "gpio.h"
#include "rcc.h"

#include "FreeRTOS.h"
#include "semphr.h"

// Semaphore for synchronizing button press vents with its external interrupt (EXTI)
static SemaphoreHandle_t exti15_10_semaphore_handle = NULL;

/**
 * Waits for user button press to acknowledge a fault event and blocks the calling task until the button is pressed.
 * During this time, the LED blinks at the specified interval to alert the user.
 * @param blink_interval_ms Time interval in milliseconds for LED blinking.
 */
static void button_wait_for_acknowledge(uint32_t blink_interval_ms)
{
  while ((xSemaphoreTake(exti15_10_semaphore_handle, 0) != pdTRUE))
  {
    gpio_toggle_pin(USER_LED_PORT, USER_LED_PIN);
    vTaskDelay(pdMS_TO_TICKS(blink_interval_ms));
  }

  gpio_write_pin(USER_LED_PORT, USER_LED_PIN, GPIO_PIN_RESET);
  exti_disable_irq(EXTI_PIN_13, EXTI15_10_IRQn);
}

void button_init(void)
{
  // Create a semaphore for EXTI
  exti15_10_semaphore_handle = xSemaphoreCreateBinary();
  configASSERT(exti15_10_semaphore_handle != NULL);

  // Configure button interrupts for user interaction
  exti_set_source(EXTI_PORT_C, USER_BUTTON_PIN);
  exti_set_trigger_edge(EXTI_PIN_13, EXTI_FALLING_EDGE);
  exti_enable_irq(EXTI_PIN_13, EXTI15_10_IRQn);
}

void button_check_and_acknowledge_iwdg_event(void)
{
  // Check for IWDG reset event and handle user acknowledgement
  if (rcc_iwdg_flag_active())
  {
    button_wait_for_acknowledge(BUTTON_ACK_BLINK_INTERVAL);
    rcc_clear_reset_flags();
  }
}

/**
 * This handler is triggered by EXTI line 15-10, specifically for the USER button connected to GPIO_PIN_13.
 */
void EXTI15_10_IRQHandler(void)
{
  portBASE_TYPE higher_priority_task_woken = pdFALSE;

  if ((EXTI->PR & EXTI_PR_PR13) == EXTI_PR_PR13)
  {
    EXTI->PR |= EXTI_PR_PR13;

    // Give the EXTI15_10 Semaphore to `button_wait_for_acknowledge`
    xSemaphoreGiveFromISR(exti15_10_semaphore_handle, &higher_priority_task_woken);
  }

  portYIELD_FROM_ISR(higher_priority_task_woken);
}



























