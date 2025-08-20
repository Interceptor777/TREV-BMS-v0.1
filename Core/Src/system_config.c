/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : system_config.c
  * @brief          : System configuration implementation
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
#include "system_config.h"
#include "can_handler.h"

/* Global Variables ----------------------------------------------------------*/
// Global flag to track CAN RX activity for debugging
volatile uint8_t can_rx_flag = 0;

// Store last received 7FF message data
volatile uint8_t last_7ff_data[8] = {0};
volatile uint8_t new_7ff_message = 0;

// Store read temperature threshold requests
volatile uint8_t new_read_temp_request = 0;

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Send system error message via CAN
  * @param  error_code: Error code to send
  * @retval None
  */
void System_SendErrorMessage(uint8_t error_code) 
{
  CAN_SendErrorMessage(error_code);
}
