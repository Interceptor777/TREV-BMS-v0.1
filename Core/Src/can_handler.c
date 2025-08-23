/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : can_handler.c
  * @brief          : CAN communication handler implementation
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
#include "can_handler.h"
#include "main.h"
#include "system_config.h"
#include "bq76952_driver.h"

/* Global Variables ----------------------------------------------------------*/
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t TxData[8];
uint8_t RxData[8];

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Configure CAN filter
  * @retval None
  */
void CAN_Config(void)
{
  CAN_FilterTypeDef sFilterConfig;

  // Configure CAN Filter to accept all messages
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  // Configure TX Header for ADC telemetry messages
  TxHeader.StdId = CAN_ID_TELEMETRY_ADC;
  TxHeader.ExtId = 0x00;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;
}

/**
  * @brief  Send ADC value over CAN
  * @param  adc_value: ADC reading to send
  * @retval None
  */
void CAN_TxData(uint16_t adc_value)
{
  // Clear TX data buffer
  for (int i = 0; i < 8; i++) {
    TxData[i] = 0;
  }
  
  // Pack ADC value into first two bytes (little endian)
  TxData[0] = (uint8_t)(adc_value & 0xFF);        // LSB
  TxData[1] = (uint8_t)((adc_value >> 8) & 0xFF); // MSB
  
  // Send CAN message
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  Send stack voltage over CAN telemetry
  * @param  stack_voltage_mv: Stack voltage in millivolts
  * @retval None
  */
void CAN_TxStackVoltage(uint16_t stack_voltage_mv)
{
  CAN_TxHeaderTypeDef TxHeader_Stack;
  uint8_t TxData_Stack[8] = {0};
  uint32_t TxMailbox_Stack;
  
  // Configure TX Header for stack voltage telemetry message
  TxHeader_Stack.StdId = CAN_ID_TELEMETRY_STACK_VOLTAGE;
  TxHeader_Stack.ExtId = 0x00;
  TxHeader_Stack.RTR = CAN_RTR_DATA;
  TxHeader_Stack.IDE = CAN_ID_STD;
  TxHeader_Stack.DLC = 8;
  TxHeader_Stack.TransmitGlobalTime = DISABLE;
  
  // Pack stack voltage into first two bytes (little endian)
  TxData_Stack[0] = (uint8_t)(stack_voltage_mv & 0xFF);        // LSB
  TxData_Stack[1] = (uint8_t)((stack_voltage_mv >> 8) & 0xFF); // MSB
  
  // Send CAN message
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Stack, TxData_Stack, &TxMailbox_Stack) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  Send all cell voltages using multiplexed format (4 cells per message, 4 mux values)
  * @retval None
  */
void CAN_TxAllCellVoltagesMux(void)
{
  CAN_TxHeaderTypeDef TxHeader_Cell;
  uint8_t TxData_Cell[8] = {0};
  uint32_t TxMailbox_Cell;
  uint16_t cell_voltages[16];
  
  // Read all cell voltages first
  for (uint8_t cell = 1; cell <= BQ76952_MAX_CELLS; cell++) {
    cell_voltages[cell - 1] = BQ76952_ReadCellVoltage(cell);
  }
  
  // Configure TX Header (same ID for all mux messages)
  TxHeader_Cell.StdId = CAN_ID_CELL_VOLTAGES;
  TxHeader_Cell.ExtId = 0x00;
  TxHeader_Cell.RTR = CAN_RTR_DATA;
  TxHeader_Cell.IDE = CAN_ID_STD;
  TxHeader_Cell.DLC = 8;
  TxHeader_Cell.TransmitGlobalTime = DISABLE;
  
  // Send Cells 1-4 (mux index 0)
  // Format: [mux_index(2bits) | cell1_voltage(14bits) | cell2_voltage(14bits) | cell3_voltage(14bits) | cell4_voltage(14bits)]
  // Pack 4 x 14-bit values into 58 bits (7.25 bytes - fits in 8 bytes)
  uint64_t packed_data = 0;
  packed_data |= (uint64_t)0x00;  // Mux index 0 (2 bits)
  packed_data |= ((uint64_t)(cell_voltages[0] & 0x3FFF)) << 2;   // Cell 1 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[1] & 0x3FFF)) << 16;  // Cell 2 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[2] & 0x3FFF)) << 30;  // Cell 3 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[3] & 0x3FFF)) << 44;  // Cell 4 (14 bits)
  
  TxData_Cell[0] = (uint8_t)(packed_data & 0xFF);
  TxData_Cell[1] = (uint8_t)((packed_data >> 8) & 0xFF);
  TxData_Cell[2] = (uint8_t)((packed_data >> 16) & 0xFF);
  TxData_Cell[3] = (uint8_t)((packed_data >> 24) & 0xFF);
  TxData_Cell[4] = (uint8_t)((packed_data >> 32) & 0xFF);
  TxData_Cell[5] = (uint8_t)((packed_data >> 40) & 0xFF);
  TxData_Cell[6] = (uint8_t)((packed_data >> 48) & 0xFF);
  TxData_Cell[7] = (uint8_t)((packed_data >> 56) & 0xFF);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Cell, TxData_Cell, &TxMailbox_Cell);
  HAL_Delay(2);
  
  // Send Cells 5-8 (mux index 1)
  packed_data = 0;
  packed_data |= (uint64_t)0x01;  // Mux index 1 (2 bits)
  packed_data |= ((uint64_t)(cell_voltages[4] & 0x3FFF)) << 2;   // Cell 5 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[5] & 0x3FFF)) << 16;  // Cell 6 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[6] & 0x3FFF)) << 30;  // Cell 7 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[7] & 0x3FFF)) << 44;  // Cell 8 (14 bits)
  
  TxData_Cell[0] = (uint8_t)(packed_data & 0xFF);
  TxData_Cell[1] = (uint8_t)((packed_data >> 8) & 0xFF);
  TxData_Cell[2] = (uint8_t)((packed_data >> 16) & 0xFF);
  TxData_Cell[3] = (uint8_t)((packed_data >> 24) & 0xFF);
  TxData_Cell[4] = (uint8_t)((packed_data >> 32) & 0xFF);
  TxData_Cell[5] = (uint8_t)((packed_data >> 40) & 0xFF);
  TxData_Cell[6] = (uint8_t)((packed_data >> 48) & 0xFF);
  TxData_Cell[7] = (uint8_t)((packed_data >> 56) & 0xFF);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Cell, TxData_Cell, &TxMailbox_Cell);
  HAL_Delay(2);
  
  // Send Cells 9-12 (mux index 2)
  packed_data = 0;
  packed_data |= (uint64_t)0x02;  // Mux index 2 (2 bits)
  packed_data |= ((uint64_t)(cell_voltages[8] & 0x3FFF)) << 2;   // Cell 9 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[9] & 0x3FFF)) << 16;  // Cell 10 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[10] & 0x3FFF)) << 30; // Cell 11 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[11] & 0x3FFF)) << 44; // Cell 12 (14 bits)
  
  TxData_Cell[0] = (uint8_t)(packed_data & 0xFF);
  TxData_Cell[1] = (uint8_t)((packed_data >> 8) & 0xFF);
  TxData_Cell[2] = (uint8_t)((packed_data >> 16) & 0xFF);
  TxData_Cell[3] = (uint8_t)((packed_data >> 24) & 0xFF);
  TxData_Cell[4] = (uint8_t)((packed_data >> 32) & 0xFF);
  TxData_Cell[5] = (uint8_t)((packed_data >> 40) & 0xFF);
  TxData_Cell[6] = (uint8_t)((packed_data >> 48) & 0xFF);
  TxData_Cell[7] = (uint8_t)((packed_data >> 56) & 0xFF);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Cell, TxData_Cell, &TxMailbox_Cell);
  HAL_Delay(2);
  
  // Send Cells 13-16 (mux index 3)
  packed_data = 0;
  packed_data |= (uint64_t)0x03;  // Mux index 3 (2 bits)
  packed_data |= ((uint64_t)(cell_voltages[12] & 0x3FFF)) << 2;  // Cell 13 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[13] & 0x3FFF)) << 16; // Cell 14 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[14] & 0x3FFF)) << 30; // Cell 15 (14 bits)
  packed_data |= ((uint64_t)(cell_voltages[15] & 0x3FFF)) << 44; // Cell 16 (14 bits)
  
  TxData_Cell[0] = (uint8_t)(packed_data & 0xFF);
  TxData_Cell[1] = (uint8_t)((packed_data >> 8) & 0xFF);
  TxData_Cell[2] = (uint8_t)((packed_data >> 16) & 0xFF);
  TxData_Cell[3] = (uint8_t)((packed_data >> 24) & 0xFF);
  TxData_Cell[4] = (uint8_t)((packed_data >> 32) & 0xFF);
  TxData_Cell[5] = (uint8_t)((packed_data >> 40) & 0xFF);
  TxData_Cell[6] = (uint8_t)((packed_data >> 48) & 0xFF);
  TxData_Cell[7] = (uint8_t)((packed_data >> 56) & 0xFF);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Cell, TxData_Cell, &TxMailbox_Cell);
}

/**
  * @brief  Send CAN error message
  * @param  error_code: Error code to send
  * @retval None
  */
void CAN_SendErrorMessage(uint8_t error_code) 
{
  CAN_TxHeaderTypeDef TxHeader_Error;
  uint8_t TxData_Error[8] = {0};
  uint32_t TxMailbox_Error;
  
  // Configure error message header
  TxHeader_Error.StdId = 0x7FE;  // Error message CAN ID
  TxHeader_Error.ExtId = 0;
  TxHeader_Error.RTR = CAN_RTR_DATA;
  TxHeader_Error.IDE = CAN_ID_STD;
  TxHeader_Error.DLC = 1;  // Only send error code
  
  TxData_Error[0] = error_code;
  
  // Send error message
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Error, TxData_Error, &TxMailbox_Error);
}

/**
  * @brief  Send heartbeat message
  * @param  counter: Heartbeat counter value
  * @retval None
  */
void CAN_SendHeartbeat(uint32_t counter)
{
  CAN_TxHeaderTypeDef TxHeader_Heartbeat;
  uint8_t TxData_Heartbeat[8] = {0xAA, 0xBB, 0xCC, 0xDD, 
                                 (uint8_t)(counter & 0xFF), 
                                 (uint8_t)((counter >> 8) & 0xFF), 
                                 can_rx_flag, 0x00};
  uint32_t TxMailbox_Heartbeat;
  
  TxHeader_Heartbeat.StdId = CAN_ID_TELEMETRY_HEARTBEAT;  // New organized heartbeat ID
  TxHeader_Heartbeat.ExtId = 0x00;
  TxHeader_Heartbeat.RTR = CAN_RTR_DATA;
  TxHeader_Heartbeat.IDE = CAN_ID_STD;
  TxHeader_Heartbeat.DLC = 8;
  TxHeader_Heartbeat.TransmitGlobalTime = DISABLE;
  
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Heartbeat, TxData_Heartbeat, &TxMailbox_Heartbeat);
}

/**
  * @brief  Send individual MUX channel thermistor data over CAN (single channel format)
  * @param  mux_channel: MUX channel number (0-7)
  * @param  adc_value: Raw ADC value for this channel
  * @retval None
  */
void CAN_TxThermistorData(uint8_t mux_channel, uint16_t adc_value)
{
  // Clear TX data buffer
  for (int i = 0; i < 8; i++) {
    TxData[i] = 0;
  }
  
  // Update header for individual ADC channel telemetry
  TxHeader.StdId = CAN_ID_TELEMETRY_ADC;
  
  // Pack single channel data with special format (0xFF mux index indicates single channel mode)
  TxData[0] = 0xFF;                                     // Special mux index for single channel mode
  TxData[1] = mux_channel;                              // MUX channel (0-7)
  TxData[2] = (uint8_t)(adc_value & 0xFF);             // ADC value LSB
  TxData[3] = (uint8_t)((adc_value >> 8) & 0xFF);      // ADC value MSB
  // Bytes 4-7 remain 0 (reserved)
  
  // Send CAN message
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  Send all thermistor temperatures in two multiplexed CAN messages
  * @param  temperatures: Array of 8 temperature values in Celsius
  * @retval None
  */
void CAN_TxAllThermistorsMux(float *temperatures)
{
  CAN_TxHeaderTypeDef TxHeader_Therm;
  uint8_t TxData_Therm[8];
  uint32_t TxMailbox_Therm;
  
  // Configure common header settings
  TxHeader_Therm.ExtId = 0x00;
  TxHeader_Therm.RTR = CAN_RTR_DATA;
  TxHeader_Therm.IDE = CAN_ID_STD;
  TxHeader_Therm.DLC = 8;
  TxHeader_Therm.TransmitGlobalTime = DISABLE;
  
  // Send thermistors 1-4 (channels 0-3)
  TxHeader_Therm.StdId = CAN_ID_THERMISTOR_TEMPS_1_4;
  
  // Convert temperatures to signed 16-bit integers (scaled by 10 for 0.1°C resolution)
  int16_t temp1 = (int16_t)(temperatures[0] * 10.0f);
  int16_t temp2 = (int16_t)(temperatures[1] * 10.0f);
  int16_t temp3 = (int16_t)(temperatures[2] * 10.0f);
  int16_t temp4 = (int16_t)(temperatures[3] * 10.0f);
  
  TxData_Therm[0] = (uint8_t)(temp1 & 0xFF);
  TxData_Therm[1] = (uint8_t)((temp1 >> 8) & 0xFF);
  TxData_Therm[2] = (uint8_t)(temp2 & 0xFF);
  TxData_Therm[3] = (uint8_t)((temp2 >> 8) & 0xFF);
  TxData_Therm[4] = (uint8_t)(temp3 & 0xFF);
  TxData_Therm[5] = (uint8_t)((temp3 >> 8) & 0xFF);
  TxData_Therm[6] = (uint8_t)(temp4 & 0xFF);
  TxData_Therm[7] = (uint8_t)((temp4 >> 8) & 0xFF);
  
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Therm, TxData_Therm, &TxMailbox_Therm);
  HAL_Delay(2);  // Small delay between messages
  
  // Send thermistors 5-8 (channels 4-7)
  TxHeader_Therm.StdId = CAN_ID_THERMISTOR_TEMPS_5_8;
  
  int16_t temp5 = (int16_t)(temperatures[4] * 10.0f);
  int16_t temp6 = (int16_t)(temperatures[5] * 10.0f);
  int16_t temp7 = (int16_t)(temperatures[6] * 10.0f);
  int16_t temp8 = (int16_t)(temperatures[7] * 10.0f);
  
  TxData_Therm[0] = (uint8_t)(temp5 & 0xFF);
  TxData_Therm[1] = (uint8_t)((temp5 >> 8) & 0xFF);
  TxData_Therm[2] = (uint8_t)(temp6 & 0xFF);
  TxData_Therm[3] = (uint8_t)((temp6 >> 8) & 0xFF);
  TxData_Therm[4] = (uint8_t)(temp7 & 0xFF);
  TxData_Therm[5] = (uint8_t)((temp7 >> 8) & 0xFF);
  TxData_Therm[6] = (uint8_t)(temp8 & 0xFF);
  TxData_Therm[7] = (uint8_t)((temp8 >> 8) & 0xFF);
  
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Therm, TxData_Therm, &TxMailbox_Therm);
}

/**
  * @brief  Send all ADC voltages in multiplexed format (2 voltages per message, 4 mux values)
  * @param  voltages: Array of 8 voltage values in volts
  * @retval None
  */
void CAN_TxAllVoltagesMux(float *voltages)
{
  CAN_TxHeaderTypeDef TxHeader_Volt;
  uint8_t TxData_Volt[8];
  uint32_t TxMailbox_Volt;
  
  // Configure common header settings for multiplexed ADC voltage telemetry
  TxHeader_Volt.StdId = CAN_ID_TELEMETRY_ADC;  // Single ID for all ADC voltages
  TxHeader_Volt.ExtId = 0x00;
  TxHeader_Volt.RTR = CAN_RTR_DATA;
  TxHeader_Volt.IDE = CAN_ID_STD;
  TxHeader_Volt.DLC = 8;
  TxHeader_Volt.TransmitGlobalTime = DISABLE;
  
  // Send voltages 1-2 (channels 0-1) with mux index 0
  TxData_Volt[0] = 0x00;  // Mux index 0
  uint16_t volt1 = (uint16_t)(voltages[0] * 1000.0f);  // Convert to mV
  uint16_t volt2 = (uint16_t)(voltages[1] * 1000.0f);  // Convert to mV
  TxData_Volt[1] = (uint8_t)(volt1 & 0xFF);
  TxData_Volt[2] = (uint8_t)((volt1 >> 8) & 0xFF);
  TxData_Volt[3] = (uint8_t)(volt2 & 0xFF);
  TxData_Volt[4] = (uint8_t)((volt2 >> 8) & 0xFF);
  TxData_Volt[5] = 0x00;  // Reserved
  TxData_Volt[6] = 0x00;  // Reserved
  TxData_Volt[7] = 0x00;  // Reserved
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Volt, TxData_Volt, &TxMailbox_Volt);
  HAL_Delay(2);
  
  // Send voltages 3-4 (channels 2-3) with mux index 1
  TxData_Volt[0] = 0x01;  // Mux index 1
  uint16_t volt3 = (uint16_t)(voltages[2] * 1000.0f);
  uint16_t volt4 = (uint16_t)(voltages[3] * 1000.0f);
  TxData_Volt[1] = (uint8_t)(volt3 & 0xFF);
  TxData_Volt[2] = (uint8_t)((volt3 >> 8) & 0xFF);
  TxData_Volt[3] = (uint8_t)(volt4 & 0xFF);
  TxData_Volt[4] = (uint8_t)((volt4 >> 8) & 0xFF);
  TxData_Volt[5] = 0x00;  // Reserved
  TxData_Volt[6] = 0x00;  // Reserved
  TxData_Volt[7] = 0x00;  // Reserved
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Volt, TxData_Volt, &TxMailbox_Volt);
  HAL_Delay(2);
  
  // Send voltages 5-6 (channels 4-5) with mux index 2
  TxData_Volt[0] = 0x02;  // Mux index 2
  uint16_t volt5 = (uint16_t)(voltages[4] * 1000.0f);
  uint16_t volt6 = (uint16_t)(voltages[5] * 1000.0f);
  TxData_Volt[1] = (uint8_t)(volt5 & 0xFF);
  TxData_Volt[2] = (uint8_t)((volt5 >> 8) & 0xFF);
  TxData_Volt[3] = (uint8_t)(volt6 & 0xFF);
  TxData_Volt[4] = (uint8_t)((volt6 >> 8) & 0xFF);
  TxData_Volt[5] = 0x00;  // Reserved
  TxData_Volt[6] = 0x00;  // Reserved
  TxData_Volt[7] = 0x00;  // Reserved
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Volt, TxData_Volt, &TxMailbox_Volt);
  HAL_Delay(2);
  
  // Send voltages 7-8 (channels 6-7) with mux index 3
  TxData_Volt[0] = 0x03;  // Mux index 3
  uint16_t volt7 = (uint16_t)(voltages[6] * 1000.0f);
  uint16_t volt8 = (uint16_t)(voltages[7] * 1000.0f);
  TxData_Volt[1] = (uint8_t)(volt7 & 0xFF);
  TxData_Volt[2] = (uint8_t)((volt7 >> 8) & 0xFF);
  TxData_Volt[3] = (uint8_t)(volt8 & 0xFF);
  TxData_Volt[4] = (uint8_t)((volt8 >> 8) & 0xFF);
  TxData_Volt[5] = 0x00;  // Reserved
  TxData_Volt[6] = 0x00;  // Reserved
  TxData_Volt[7] = 0x00;  // Reserved
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader_Volt, TxData_Volt, &TxMailbox_Volt);
}

/**
  * @brief  Process temperature-related CAN requests
  * @retval None
  */
void CAN_ProcessTemperatureRequests(void)
{
  // Check if we received a temperature threshold write command (new or legacy)
  if (new_7ff_message == 1) {
    new_7ff_message = 0; // Clear flag
    
    // Extract temperature values from received message
    int8_t min_temp = (int8_t)last_7ff_data[0];  // Byte 0: Min temperature
    int8_t max_temp = (int8_t)last_7ff_data[1];  // Byte 1: Max temperature
    
    CAN_TxHeaderTypeDef TxHeader_WriteResp;
    uint8_t TxData_WriteResp[8];
    uint32_t TxMailbox_WriteResp;
    
    // Try to set temperature thresholds
    if (BQ76952_SetTemperatureThresholds(min_temp, max_temp) == HAL_OK) {
      // Send success reply message (all 0xFF)
      for (int i = 0; i < 8; i++) {
        TxData_WriteResp[i] = 0xFF;
      }
    } else {
      // Send failure reply message (all 0x00)  
      for (int i = 0; i < 8; i++) {
        TxData_WriteResp[i] = 0x00;
      }
    }
    
    // Configure and send reply message on new organized response ID
    TxHeader_WriteResp.StdId = CAN_ID_WRITE_TEMP_THRESHOLDS_RESP;
    TxHeader_WriteResp.ExtId = 0x00;
    TxHeader_WriteResp.RTR = CAN_RTR_DATA;
    TxHeader_WriteResp.IDE = CAN_ID_STD;
    TxHeader_WriteResp.DLC = 8;
    TxHeader_WriteResp.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader_WriteResp, TxData_WriteResp, &TxMailbox_WriteResp);
  }
  
  // Check if we received a read temperature thresholds request
  if (new_read_temp_request == 1) {
    new_read_temp_request = 0; // Clear flag
    
    CAN_TxHeaderTypeDef TxHeader_ReadResp;
    uint8_t TxData_ReadResp[8] = {0};
    uint32_t TxMailbox_ReadResp;
    
    int8_t current_min_temp = 0;
    int8_t current_max_temp = 0;
    
    // Try to read current temperature thresholds from BMS
    if (BQ76952_ReadTemperatureThresholds(&current_min_temp, &current_max_temp) == HAL_OK) {
      // Pack current thresholds in response
      TxData_ReadResp[0] = (uint8_t)current_min_temp;  // Byte 0: Current min temperature
      TxData_ReadResp[1] = (uint8_t)current_max_temp;  // Byte 1: Current max temperature
      TxData_ReadResp[2] = 0x01;  // Byte 2: Success flag
      // Bytes 3-7: Reserved/unused
    } else {
      // Send error response
      TxData_ReadResp[0] = 0x00;  // Byte 0: Error value
      TxData_ReadResp[1] = 0x00;  // Byte 1: Error value  
      TxData_ReadResp[2] = 0x00;  // Byte 2: Error flag
      // Bytes 3-7: Reserved/unused
    }
    
    // Configure and send response message
    TxHeader_ReadResp.StdId = CAN_ID_READ_TEMP_THRESHOLDS_RESP;
    TxHeader_ReadResp.ExtId = 0x00;
    TxHeader_ReadResp.RTR = CAN_RTR_DATA;
    TxHeader_ReadResp.IDE = CAN_ID_STD;
    TxHeader_ReadResp.DLC = 8;
    TxHeader_ReadResp.TransmitGlobalTime = DISABLE;
    
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader_ReadResp, TxData_ReadResp, &TxMailbox_ReadResp);
  }
}

/**
  * @brief  CAN Receive Callback - handles incoming temperature threshold commands
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader_Temp;
  uint8_t RxData_Temp[8];
  
  // Set debug flag to indicate RX callback was triggered
  can_rx_flag = 1;
  
  // Simply clear the FIFO without any complex processing
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader_Temp, RxData_Temp);
  
  // Check for new organized write temperature threshold command
  if (RxHeader_Temp.StdId == CAN_ID_WRITE_TEMP_THRESHOLDS_REQ) {
    can_rx_flag = 2;  // Special value to indicate write command was received
    // Copy message data for main loop to process
    for (int i = 0; i < 8; i++) {
      last_7ff_data[i] = RxData_Temp[i];
    }
    new_7ff_message = 1;  // Flag for main loop
  }
  // Check for read temperature thresholds request
  else if (RxHeader_Temp.StdId == CAN_ID_READ_TEMP_THRESHOLDS_REQ) {
    can_rx_flag = 3;  // Special value to indicate read command was received
    new_read_temp_request = 1;  // Flag for main loop
  }
  // Legacy support for 7FF temperature threshold command
  else if (RxHeader_Temp.StdId == CAN_ID_LEGACY_TEMP_THRESHOLDS) {
    can_rx_flag = 2;  // Special value to indicate legacy 7FF was received
    // Copy message data for main loop to process
    for (int i = 0; i < 8; i++) {
      last_7ff_data[i] = RxData_Temp[i];
    }
    new_7ff_message = 1;  // Flag for main loop
  }
  
  // Don't try to transmit from within interrupt - let main loop handle it
}
