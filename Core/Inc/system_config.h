/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : system_config.h
  * @brief          : System configuration definitions and error codes
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

#ifndef __SYSTEM_CONFIG_H
#define __SYSTEM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Error Code Definitions ----------------------------------------------------*/
#define ERROR_TEMP_RANGE_INVALID    0x01
#define ERROR_I2C_COMMUNICATION     0x02  
#define ERROR_DATA_MEMORY_WRITE     0x03
#define ERROR_DATA_MEMORY_READ      0x04

/* CAN ID Definitions --------------------------------------------------------*/
// 0x100-0x11F: Read/Write Request Commands and Responses (10 IDs used of 32 available)
#define CAN_ID_READ_CELL_VOLTAGE_REQ     0x100  // Request specific cell voltage
#define CAN_ID_READ_CELL_VOLTAGE_RESP    0x101  // Response with cell voltage
#define CAN_ID_READ_STACK_VOLTAGE_REQ    0x102  // Request stack voltage  
#define CAN_ID_READ_STACK_VOLTAGE_RESP   0x103  // Response with stack voltage
#define CAN_ID_READ_TEMP_THRESHOLDS_REQ  0x104  // Request current temp thresholds
#define CAN_ID_READ_TEMP_THRESHOLDS_RESP 0x105  // Response with temp thresholds
#define CAN_ID_WRITE_TEMP_THRESHOLDS_REQ  0x106  // Write temperature thresholds
#define CAN_ID_WRITE_TEMP_THRESHOLDS_RESP 0x107  // Response (success/failure)
#define CAN_ID_WRITE_CELL_BALANCE_REQ     0x108  // Write cell balancing config
#define CAN_ID_WRITE_CELL_BALANCE_RESP    0x109  // Response for cell balancing

// 0x600-0x61F: Telemetry Information (4 IDs used of 32 available)
#define CAN_ID_TELEMETRY_ADC             0x600  // ADC thermistor readings
#define CAN_ID_TELEMETRY_STACK_VOLTAGE   0x601  // Stack voltage telemetry
#define CAN_ID_CELL_VOLTAGES             0x602  // Multiplexed all cell voltages
#define CAN_ID_TELEMETRY_HEARTBEAT       0x603  // System heartbeat

// 0x700-0x71F: Debug/Legacy Messages (2 IDs used of 32 available)
#define CAN_ID_ERROR_DEBUG               0x700  // Error debug messages
#define CAN_ID_LEGACY_TEMP_THRESHOLDS    0x701  // Legacy temperature threshold command

/* Global Variables ----------------------------------------------------------*/
extern volatile uint8_t can_rx_flag;
extern volatile uint8_t last_7ff_data[8];
extern volatile uint8_t new_7ff_message;
extern volatile uint8_t new_read_temp_request;

/* Function Prototypes -------------------------------------------------------*/
void System_SendErrorMessage(uint8_t error_code);

#ifdef __cplusplus
}
#endif

#endif /* __SYSTEM_CONFIG_H */
