/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc_handler.h
  * @brief          : ADC operations handler header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __ADC_HANDLER_H
#define __ADC_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <math.h>

/* MUX Control Definitions ---------------------------------------------------*/
#define MUX_CHANNELS    8      // Number of MUX channels (0-7)

// MUX Control Pin Definitions (from IOC file)
#define MUX_SIG1_PIN    GPIO_PIN_1
#define MUX_SIG1_PORT   GPIOB
#define MUX_SIG2_PIN    GPIO_PIN_8  
#define MUX_SIG2_PORT   GPIOA
#define MUX_SIG3_PIN    GPIO_PIN_5
#define MUX_SIG3_PORT   GPIOB

/* Thermistor Calculation Definitions ----------------------------------------*/
#define THERMISTOR_B_VALUE     4300    // B25/85 value of thermistor (4300K per datasheet)
#define THERMISTOR_R25         10000   // Resistance at 25°C (10kΩ typical)
#define REFERENCE_TEMP_K       298.15  // 25°C in Kelvin  
#define PULLUP_RESISTOR        10000   // 10kΩ pullup resistor
#define ADC_RESOLUTION         4095    // 12-bit ADC resolution
#define ADC_VREF               3.3     // ADC reference voltage (STM32 internal 3.3V reference)

/* Function Prototypes -------------------------------------------------------*/
uint16_t ADC_ReadValue(void);
void MUX_SetChannel(uint8_t channel);
uint16_t MUX_ReadChannel(uint8_t channel);
float ADC_CalculateThermistorTemp(uint16_t adc_value);
void ADC_ReadAllThermistors(float *temperatures);
float ADC_ConvertToVoltage(uint16_t adc_value);
void ADC_ReadAllVoltages(float *voltages);

/* External Variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

#ifdef __cplusplus
}
#endif

#endif /* __ADC_HANDLER_H */
