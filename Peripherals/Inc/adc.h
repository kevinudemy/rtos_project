/*
 * adc.h
 *
 * Provides macro definitions and function prototypes related to ADC initialization.
 */

#ifndef INC_ADC_H_
#define INC_ADC_H_

#include "mcu.h"

/**
 * Enumerates ADC channels for clear and easy reference throughout the code.
 */
typedef enum
{
  ADC_CH0 = 0,
  ADC_CH1,
  ADC_CH2,
  ADC_CH3,
  ADC_CH4,
  ADC_CH5,
  ADC_CH6,
  ADC_CH7,
  ADC_CH8,
  ADC_CH9,
  ADC_CH10,
  ADC_CH11,
  ADC_CH12,
  ADC_CH13,
  ADC_CH14,
  ADC_CH15,
  ADC_CH16,
  ADC_CH17,
  ADC_CH18
} adc_channels_e;

/**
 * Initializes ADC for the application by configuring the ADC clock, prescaler,
 * and temperature sensor channel and calls individual configuration functions
 * like adc1_config and enables ADC interrupts.
 * Sets ADC clock to ~11.25 MHz by dividing the APB2 clock (45 MHz) by 4.
 */
void adc_init(void);

/**
 * Sets up the Analog Watchdog (AWD) feature of the specified ADC.
 * AWD is used to monitor the ADC conversion result against set threshold values.
 *
 * @param ADCx          The ADC instance to configure.
 * @param channel       The ADC channel to monitor with AWD.
 * @param high_threshold The high threshold value for the AWD.
 * @param low_threshold  The low threshold value for the AWD.
 */
void adc_awd_init(ADC_TypeDef *ADCx, adc_channels_e channel, uint16_t high_threshold, uint16_t low_threshold);


#endif /* INC_ADC_H_ */
