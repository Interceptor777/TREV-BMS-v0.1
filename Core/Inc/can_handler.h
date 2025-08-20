/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_handler.h
  * @brief          : CAN communication handler header file
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

#ifndef __CAN_HANDLER_H
#define __CAN_HANDLER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* Function Prototypes -------------------------------------------------------*/
void CAN_Config(void);
void CAN_TxData(uint16_t adc_value);
void CAN_TxStackVoltage(uint16_t stack_voltage_mv);
void CAN_TxAllCellVoltagesMux(void);
void CAN_SendErrorMessage(uint8_t error_code);
void CAN_SendHeartbeat(uint32_t counter);
void CAN_ProcessTemperatureRequests(void);
void CAN_TxThermistorData(uint8_t mux_channel, uint16_t adc_value);
void CAN_TxAllThermistorsMux(float *temperatures);
void CAN_TxAllVoltagesMux(float *voltages);

/* External Variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_TxHeaderTypeDef TxHeader;
extern uint32_t TxMailbox;
extern uint8_t TxData[8];
extern uint8_t RxData[8];

#ifdef __cplusplus
}
#endif

#endif /* __CAN_HANDLER_H */
