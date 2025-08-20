/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bq76952_driver.c
  * @brief          : BQ76952 BMS driver implementation
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
#include "bq76952_driver.h"
#include "system_config.h"

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Read stack voltage from BQ76952 via I2C
  * @retval Stack voltage in millivolts (0 if communication failed)
  */
uint16_t BQ76952_ReadStackVoltage(void)
{
  uint8_t reg_addr = BQ76952_STACK_VOLTAGE;
  uint8_t rx_data[2] = {0};
  uint16_t stack_voltage_mv = 0;
  
  // Read 2 bytes from BQ76952 stack voltage register
  if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), reg_addr, 1, rx_data, 2, HAL_MAX_DELAY) == HAL_OK) {
    // Convert to millivolts (BQ76952 returns voltage in 0.01V units)
    // Combine low and high bytes (little endian)
    stack_voltage_mv = ((uint16_t)rx_data[1] << 8) | rx_data[0];
    stack_voltage_mv *= 10; // Convert from 0.01V to mV
  }
  
  return stack_voltage_mv;
}

/**
  * @brief  Read individual cell voltage from BQ76952 via I2C
  * @param  cell_number: Cell number (1-16)
  * @retval Cell voltage in millivolts (0 if communication failed)
  */
uint16_t BQ76952_ReadCellVoltage(uint8_t cell_number)
{
  uint8_t reg_addr;
  uint8_t rx_data[2] = {0};
  uint16_t cell_voltage_mv = 0;
  
  // Validate cell number
  if (cell_number < 1 || cell_number > BQ76952_MAX_CELLS) {
    return 0;
  }
  
  // Calculate register address for the cell (Cell 1 starts at 0x14, increment by 2 for each cell)
  reg_addr = BQ76952_CELL1_VOLTAGE + ((cell_number - 1) * 2);
  
  // Read 2 bytes from BQ76952 cell voltage register
  if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), reg_addr, 1, rx_data, 2, HAL_MAX_DELAY) == HAL_OK) {
    // Convert to millivolts (BQ76952 returns voltage in 1mV units)
    // Combine low and high bytes (little endian)
    cell_voltage_mv = ((uint16_t)rx_data[1] << 8) | rx_data[0];
  }
  
  return cell_voltage_mv;
}

/**
  * @brief  Write data to BQ76952 data memory using block data transfer
  * @param  address: Data memory address (16-bit)
  * @param  data: Pointer to data to write
  * @param  length: Number of bytes to write (max 32)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76952_WriteDataMemory(uint16_t address, uint8_t *data, uint8_t length)
{
  uint8_t subcommand[2];
  uint8_t checksum = 0;
  uint8_t data_length;
  uint8_t verify_bytes[2];
  uint8_t i;
  uint8_t retry_count = 0;
  
  if (length > 32 || data == NULL) {
    return HAL_ERROR;  // Maximum block size is 32 bytes
  }
  
  // Add substantial delay before starting new data memory operation
  HAL_Delay(200);  // Give BMS time to finish any previous operations
  
  // Step 1: Write subcommand address to 0x3E and 0x3F
  subcommand[0] = (uint8_t)(address & 0xFF);        // Low byte to 0x3E
  subcommand[1] = (uint8_t)((address >> 8) & 0xFF); // High byte to 0x3F
  
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &subcommand[0], 1, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  HAL_Delay(10);  // Delay between subcommand writes
  
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &subcommand[1], 1, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  
  // Step 2: Wait for subcommand completion (per datasheet page 13)
  // Read 0x3E and 0x3F until they return original values (not 0xFF)
  HAL_Delay(50);  // Initial delay for subcommand processing
  
  for (retry_count = 0; retry_count < 10; retry_count++) {
    if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &verify_bytes[0], 1, HAL_MAX_DELAY) != HAL_OK) {
      System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
      return HAL_ERROR;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &verify_bytes[1], 1, HAL_MAX_DELAY) != HAL_OK) {
      System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
      return HAL_ERROR;
    }
    
    // Check if subcommand completed (returns original values, not 0xFF)
    if (verify_bytes[0] == subcommand[0] && verify_bytes[1] == subcommand[1]) {
      break;  // Subcommand completed successfully
    }
    
    HAL_Delay(20);  // Wait before retry
  }
  
  if (retry_count >= 10) {
    System_SendErrorMessage(ERROR_DATA_MEMORY_WRITE);
    return HAL_ERROR;  // Subcommand did not complete
  }
  
  // Step 3: Write data to transfer buffer (0x40-0x5F)
  for (i = 0; i < length; i++) {
    if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_BASE + i, 1, &data[i], 1, HAL_MAX_DELAY) != HAL_OK) {
      System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
      return HAL_ERROR;
    }
    HAL_Delay(5);  // Increased delay between each data byte
  }
  
  // Step 4: Calculate checksum (sum of subcommand bytes + data bytes, then bitwise inverted)
  checksum = subcommand[0] + subcommand[1];
  for (i = 0; i < length; i++) {
    checksum += data[i];
  }
  checksum = ~checksum;  // Bitwise invert (per datasheet)
  
  // Step 5: Calculate data length (includes 0x3E, 0x3F, 0x60, 0x61 + buffer length)
  data_length = length + 4;
  
  // Step 6: Write checksum and length together as a word to 0x60 and 0x61
  uint8_t checksum_length[2] = {checksum, data_length};
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_CHECKSUM, 1, checksum_length, 2, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  
  // Step 7: Extended delay to allow BMS to process the complete transaction
  HAL_Delay(250);  // Substantial delay for data memory write completion
  
  return HAL_OK;
}

/**
  * @brief  Set temperature thresholds on BQ76952
  * @param  min_temp_c: Minimum temperature in Celsius (-40 to 120)
  * @param  max_temp_c: Maximum temperature in Celsius (-40 to 120)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76952_SetTemperatureThresholds(int8_t min_temp_c, int8_t max_temp_c)
{
  uint8_t temp_data;
  
  // Validate temperature range
  if (min_temp_c < -40 || min_temp_c > 120 || max_temp_c < -40 || max_temp_c > 120) {
    System_SendErrorMessage(ERROR_TEMP_RANGE_INVALID);
    return HAL_ERROR;
  }
  
  // Debug: Send what we're about to write for minimum temp
  temp_data = (uint8_t)min_temp_c;
  System_SendErrorMessage(temp_data);  // Debug: Show what we're writing for min temp
  
  // Set UTC (Under Temperature Charge) threshold - minimum temperature
  if (BQ76952_WriteDataMemory(BQ76952_UTC_THRESHOLD, &temp_data, 1) != HAL_OK) {
    System_SendErrorMessage(ERROR_DATA_MEMORY_WRITE);
    return HAL_ERROR;
  }
  
  // Extended delay between consecutive data memory writes
  HAL_Delay(500);  // Give BMS substantial time to process first write before second
  
  // Debug: Send what we're about to write for maximum temp (with offset to distinguish)
  temp_data = (uint8_t)max_temp_c;
  System_SendErrorMessage(temp_data + 0x40);  // Debug: Show what we're writing for max temp (offset)
  
  // Set OTC (Over Temperature Charge) threshold - maximum temperature
  if (BQ76952_WriteDataMemory(BQ76952_OTC_THRESHOLD, &temp_data, 1) != HAL_OK) {
    System_SendErrorMessage(ERROR_DATA_MEMORY_WRITE);
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief  Read data from BQ76952 data memory
  * @param  address: Data memory address to read from
  * @param  data: Pointer to buffer to store read data
  * @param  length: Number of bytes to read (1-32)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76952_ReadDataMemory(uint16_t address, uint8_t *data, uint8_t length)
{
  uint8_t subcommand[2];
  uint8_t verify_bytes[2];
  uint8_t expected_checksum = 0;
  uint8_t actual_checksum = 0;
  uint8_t actual_length = 0;
  uint8_t i;
  uint8_t retry_count = 0;
  
  if (length > 32 || data == NULL) {
    return HAL_ERROR;  // Maximum block size is 32 bytes
  }
  
  // Add substantial delay before starting new data memory operation
  HAL_Delay(200);  // Give BMS time to finish any previous operations
  
  // Step 1: Write subcommand address to 0x3E and 0x3F
  subcommand[0] = (uint8_t)(address & 0xFF);        // Low byte to 0x3E
  subcommand[1] = (uint8_t)((address >> 8) & 0xFF); // High byte to 0x3F
  
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &subcommand[0], 1, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  HAL_Delay(10);  // Delay between subcommand writes
  
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &subcommand[1], 1, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  
  // Step 2: Wait for subcommand completion (per datasheet page 13)
  // Read 0x3E and 0x3F until they return original values (not 0xFF)
  HAL_Delay(50);  // Initial delay for subcommand processing
  
  for (retry_count = 0; retry_count < 15; retry_count++) {
    if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &verify_bytes[0], 1, HAL_MAX_DELAY) != HAL_OK) {
      System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
      return HAL_ERROR;
    }
    if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &verify_bytes[1], 1, HAL_MAX_DELAY) != HAL_OK) {
      System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
      return HAL_ERROR;
    }
    
    // Check if subcommand completed (returns original values, not 0xFF)
    if (verify_bytes[0] == subcommand[0] && verify_bytes[1] == subcommand[1]) {
      break;  // Subcommand completed successfully
    }
    
    HAL_Delay(30);  // Wait before retry
  }
  
  if (retry_count >= 15) {
    System_SendErrorMessage(ERROR_DATA_MEMORY_READ);
    return HAL_ERROR;  // Subcommand did not complete
  }
  
  // Step 3: Read the data length from 0x61
  if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_CONTROL, 1, &actual_length, 1, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  
  // Step 4: Read data from transfer buffer (0x40-0x5F)
  for (i = 0; i < length; i++) {
    if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_BASE + i, 1, &data[i], 1, HAL_MAX_DELAY) != HAL_OK) {
      System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
      return HAL_ERROR;
    }
    HAL_Delay(3);  // Small delay between each data byte read
  }
  
  // Step 5: Read checksum from 0x60 and verify
  if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_CHECKSUM, 1, &actual_checksum, 1, HAL_MAX_DELAY) != HAL_OK) {
    System_SendErrorMessage(ERROR_I2C_COMMUNICATION);
    return HAL_ERROR;
  }
  
  // Calculate expected checksum (sum of subcommand bytes + data bytes, then bitwise inverted)
  expected_checksum = subcommand[0] + subcommand[1];
  for (i = 0; i < length; i++) {
    expected_checksum += data[i];
  }
  expected_checksum = ~expected_checksum;  // Bitwise invert (per datasheet)
  
  // Verify checksum
  if (actual_checksum != expected_checksum) {
    System_SendErrorMessage(ERROR_DATA_MEMORY_READ);
    return HAL_ERROR;  // Checksum verification failed
  }
  
  return HAL_OK;
}

/**
  * @brief  Simplified BQ76952 data memory read for debugging
  * @param  address: Data memory address to read from
  * @param  data: Pointer to buffer to store read data
  * @param  length: Number of bytes to read (should be 1 for debugging)
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76952_ReadDataMemory_Simple(uint16_t address, uint8_t *data, uint8_t length)
{
  uint8_t subcommand[2];
  uint8_t i;
  
  if (length > 1 || data == NULL) {
    return HAL_ERROR;  // For debugging, only read 1 byte at a time
  }
  
  // Very long delay before starting
  HAL_Delay(1000);  
  
  // Step 1: Write subcommand address to 0x3E and 0x3F
  subcommand[0] = (uint8_t)(address & 0xFF);        // Low byte to 0x3E
  subcommand[1] = (uint8_t)((address >> 8) & 0xFF); // High byte to 0x3F
  
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &subcommand[0], 1, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }
  HAL_Delay(50);  
  
  if (HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &subcommand[1], 1, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }
  
  // Very long delay to let BMS process
  HAL_Delay(500);
  
  // Step 2: Simply read the first byte from transfer buffer (0x40)
  if (HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_BASE, 1, data, 1, HAL_MAX_DELAY) != HAL_OK) {
    return HAL_ERROR;
  }
  
  return HAL_OK;
}

/**
  * @brief  Reset BQ76952 communication state with multiple attempts
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76952_ResetCommState(void)
{
  uint8_t dummy_data = 0x00;
  uint8_t retry_count;
  
  // Try multiple reset attempts to ensure state is cleared
  for (retry_count = 0; retry_count < 3; retry_count++) {
    // Clear subcommand registers
    HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &dummy_data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &dummy_data, 1, HAL_MAX_DELAY);
    HAL_Delay(10);
    
    // Clear the data buffer control register
    HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_CONTROL, 1, &dummy_data, 1, HAL_MAX_DELAY);
    HAL_Delay(20);
    
    // Clear checksum register as well
    HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_CHECKSUM, 1, &dummy_data, 1, HAL_MAX_DELAY);
    HAL_Delay(30);  // Longer delay between reset attempts
  }
  
  return HAL_OK;
}

/**
  * @brief  Test basic BQ76952 I2C communication with enhanced recovery and retry logic
  * @param  min_temp_c: Pointer to store minimum temperature in Celsius
  * @param  max_temp_c: Pointer to store maximum temperature in Celsius
  * @retval HAL_StatusTypeDef
  */
HAL_StatusTypeDef BQ76952_ReadTemperatureThresholds(int8_t *min_temp_c, int8_t *max_temp_c)
{
  uint8_t test_data[2];
  uint8_t buffer_data = 0;
  uint8_t subcommand[2];
  int result;
  uint8_t retry_attempt;
  uint8_t comm_test_passed = 0;
  
  if (min_temp_c == NULL || max_temp_c == NULL) {
    return HAL_ERROR;
  }
  
  // Set default values
  *min_temp_c = -10;
  *max_temp_c = 60;
  
  // Try communication test with retries
  for (retry_attempt = 0; retry_attempt < 3; retry_attempt++) {
    // Always reset communication state first
    BQ76952_ResetCommState();
    HAL_Delay(100);  // Extra delay after reset
    
    // Test basic I2C communication with stack voltage read
    result = HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_STACK_VOLTAGE, 1, test_data, 2, HAL_MAX_DELAY);
    if (result == HAL_OK) {
      comm_test_passed = 1;
      break;  // Communication works, proceed
    }
    
    HAL_Delay(200);  // Wait before retry
  }
  
  if (!comm_test_passed) {
    System_SendErrorMessage(0xFF);  // Basic I2C failure after retries
    return HAL_OK;
  }
  
  // === Read UTC Threshold (Min Temp) with retry logic ===
  for (retry_attempt = 0; retry_attempt < 2; retry_attempt++) {
    subcommand[0] = (uint8_t)(BQ76952_UTC_THRESHOLD & 0xFF);        
    subcommand[1] = (uint8_t)((BQ76952_UTC_THRESHOLD >> 8) & 0xFF); 
    
    // Write subcommand with extended delays
    result = HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &subcommand[0], 1, HAL_MAX_DELAY);
    if (result != HAL_OK) {
      if (retry_attempt == 0) {
        BQ76952_ResetCommState();
        HAL_Delay(200);
        continue;  // Retry
      } else {
        System_SendErrorMessage(0xFE);  // Final failure
        BQ76952_ResetCommState();
        return HAL_OK;
      }
    }
    
    HAL_Delay(30);  // Longer delay between subcommand bytes
    
    result = HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &subcommand[1], 1, HAL_MAX_DELAY);
    if (result != HAL_OK) {
      if (retry_attempt == 0) {
        BQ76952_ResetCommState();
        HAL_Delay(200);
        continue;  // Retry
      } else {
        System_SendErrorMessage(0xFD);  // Final failure
        BQ76952_ResetCommState();
        return HAL_OK;
      }
    }
    
    // Extended wait for BMS to process subcommand and populate buffer
    HAL_Delay(300);  // Longer processing delay
    
    // Read from buffer
    result = HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_BASE, 1, &buffer_data, 1, HAL_MAX_DELAY);
    if (result == HAL_OK) {
      System_SendErrorMessage(buffer_data);  // Show min temp data
      *min_temp_c = (int8_t)buffer_data;
      break;  // Success, exit retry loop
    } else if (retry_attempt == 0) {
      BQ76952_ResetCommState();
      HAL_Delay(200);
      // Continue to retry
    } else {
      System_SendErrorMessage(0xFC);  // Final buffer read failure
      BQ76952_ResetCommState();
      return HAL_OK;
    }
  }
  
  // Clear state and wait before next operation
  BQ76952_ResetCommState();
  HAL_Delay(800);  // Even longer delay between operations
  
  // === Read OTC Threshold (Max Temp) with same retry logic ===
  for (retry_attempt = 0; retry_attempt < 2; retry_attempt++) {
    subcommand[0] = (uint8_t)(BQ76952_OTC_THRESHOLD & 0xFF);        
    subcommand[1] = (uint8_t)((BQ76952_OTC_THRESHOLD >> 8) & 0xFF); 
    
    result = HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_LOW, 1, &subcommand[0], 1, HAL_MAX_DELAY);
    if (result != HAL_OK) {
      if (retry_attempt == 0) {
        BQ76952_ResetCommState();
        HAL_Delay(200);
        continue;
      } else {
        System_SendErrorMessage(0xFA);
        BQ76952_ResetCommState();
        return HAL_OK;
      }
    }
    
    HAL_Delay(30);
    
    result = HAL_I2C_Mem_Write(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_SUBCOMMAND_HIGH, 1, &subcommand[1], 1, HAL_MAX_DELAY);
    if (result != HAL_OK) {
      if (retry_attempt == 0) {
        BQ76952_ResetCommState();
        HAL_Delay(200);
        continue;
      } else {
        System_SendErrorMessage(0xF9);
        BQ76952_ResetCommState();
        return HAL_OK;
      }
    }
    
    HAL_Delay(300);
    
    result = HAL_I2C_Mem_Read(&hi2c1, (BQ76952_I2C_ADDRESS << 1), BQ76952_BLOCK_DATA_BASE, 1, &buffer_data, 1, HAL_MAX_DELAY);
    if (result == HAL_OK) {
      System_SendErrorMessage(buffer_data + 0x80);  // Show max temp data (with offset)
      *max_temp_c = (int8_t)buffer_data;
      break;  // Success
    } else if (retry_attempt == 0) {
      BQ76952_ResetCommState();
      HAL_Delay(200);
      // Continue to retry
    } else {
      System_SendErrorMessage(0xF8);  // Final failure
    }
  }
  
  // Final cleanup
  BQ76952_ResetCommState();
  
  return HAL_OK;
}
