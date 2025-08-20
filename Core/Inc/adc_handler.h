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

/* Function Prototypes -------------------------------------------------------*/
uint16_t ADC_ReadValue(void);

/* External Variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;

#ifdef __cplusplus
}
#endif

#endif /* __ADC_HANDLER_H */
