/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc_handler.c
  * @brief          : ADC operations handler implementation
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

/* Includes ------------------------------------------------------------------*/
#include "adc_handler.h"

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Read ADC value from PA0 (ADC channel 5)
  * @retval ADC value (0-4095 for 12-bit resolution)
  */
uint16_t ADC_ReadValue(void)
{
  uint16_t adc_value = 0;
  
  // Start ADC conversion
  if (HAL_ADC_Start(&hadc1) == HAL_OK) {
    // Wait for conversion to complete
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
      // Get ADC value
      adc_value = HAL_ADC_GetValue(&hadc1);
    }
    // Stop ADC
    HAL_ADC_Stop(&hadc1);
  }
  
  return adc_value;
}
