/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : bq76952_driver.h
  * @brief          : BQ76952 BMS driver header file
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

#ifndef __BQ76952_DRIVER_H
#define __BQ76952_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include <stdint.h>

/* BQ76952 Definitions -------------------------------------------------------*/
// I2C Communication
#define BQ76952_I2C_ADDRESS     0x08  // 7-bit I2C address
#define BQ76952_I2C_TIMEOUT     1000   // I2C timeout in milliseconds

// Device Configuration
#define BQ76952_MAX_CELLS       16     // Maximum number of cells supported
#define BQ76952_MAX_THERMISTORS 9      // Maximum number of external thermistors

// Direct Command Registers (7-bit addresses)
#define BQ76952_CONTROL_STATUS  0x00   // Control status register (2 bytes)
#define BQ76952_SAFETY_ALERT_A  0x02   // Safety Alert A register (2 bytes)
#define BQ76952_SAFETY_STATUS_A 0x03   // Safety Status A register (2 bytes)
#define BQ76952_SAFETY_ALERT_B  0x04   // Safety Alert B register (2 bytes)
#define BQ76952_SAFETY_STATUS_B 0x05   // Safety Status B register (2 bytes)
#define BQ76952_SAFETY_ALERT_C  0x06   // Safety Alert C register (2 bytes)
#define BQ76952_SAFETY_STATUS_C 0x07   // Safety Status C register (2 bytes)
#define BQ76952_PF_ALERT_A      0x08   // Permanent Fail Alert A register (2 bytes)
#define BQ76952_PF_STATUS_A     0x09   // Permanent Fail Status A register (2 bytes)
#define BQ76952_PF_ALERT_B      0x0A   // Permanent Fail Alert B register (2 bytes)
#define BQ76952_PF_STATUS_B     0x0B   // Permanent Fail Status B register (2 bytes)
#define BQ76952_PF_ALERT_C      0x0C   // Permanent Fail Alert C register (2 bytes)
#define BQ76952_PF_STATUS_C     0x0D   // Permanent Fail Status C register (2 bytes)
#define BQ76952_PF_ALERT_D      0x0E   // Permanent Fail Alert D register (2 bytes)
#define BQ76952_PF_STATUS_D     0x0F   // Permanent Fail Status D register (2 bytes)
#define BQ76952_BATTERY_STATUS  0x12   // Battery Status register (2 bytes)
#define BQ76952_ALARM_STATUS    0x13   // Alarm Status register (2 bytes)

// Voltage Measurement Registers
#define BQ76952_CELL1_VOLTAGE   0x14   // Cell 1 voltage register (2 bytes)
#define BQ76952_CELL2_VOLTAGE   0x16   // Cell 2 voltage register (2 bytes)
#define BQ76952_CELL3_VOLTAGE   0x18   // Cell 3 voltage register (2 bytes)
#define BQ76952_CELL4_VOLTAGE   0x1A   // Cell 4 voltage register (2 bytes)
#define BQ76952_CELL5_VOLTAGE   0x1C   // Cell 5 voltage register (2 bytes)
#define BQ76952_CELL6_VOLTAGE   0x1E   // Cell 6 voltage register (2 bytes)
#define BQ76952_CELL7_VOLTAGE   0x20   // Cell 7 voltage register (2 bytes)
#define BQ76952_CELL8_VOLTAGE   0x22   // Cell 8 voltage register (2 bytes)
#define BQ76952_CELL9_VOLTAGE   0x24   // Cell 9 voltage register (2 bytes)
#define BQ76952_CELL10_VOLTAGE  0x26   // Cell 10 voltage register (2 bytes)
#define BQ76952_CELL11_VOLTAGE  0x28   // Cell 11 voltage register (2 bytes)
#define BQ76952_CELL12_VOLTAGE  0x2A   // Cell 12 voltage register (2 bytes)
#define BQ76952_CELL13_VOLTAGE  0x2C   // Cell 13 voltage register (2 bytes)
#define BQ76952_CELL14_VOLTAGE  0x2E   // Cell 14 voltage register (2 bytes)
#define BQ76952_CELL15_VOLTAGE  0x30   // Cell 15 voltage register (2 bytes)
#define BQ76952_CELL16_VOLTAGE  0x32   // Cell 16 voltage register (2 bytes)
#define BQ76952_STACK_VOLTAGE   0x34   // Stack voltage register (2 bytes)
#define BQ76952_PACK_PIN_VOLTAGE 0x36  // PACK pin voltage register (2 bytes)
#define BQ76952_LD_PIN_VOLTAGE  0x38   // LD pin voltage register (2 bytes)

// Current Measurement Registers
#define BQ76952_CC2_CURRENT     0x3A   // CC2 current register (2 bytes)
#define BQ76952_CC1_CURRENT     0x3C   // CC1 current register (2 bytes)

// Subcommand Registers
#define BQ76952_SUBCOMMAND_LOW  0x3E   // Subcommand low byte register
#define BQ76952_SUBCOMMAND_HIGH 0x3F   // Subcommand high byte register

// Block Data Transfer Buffer (32 bytes: 0x40-0x5F)
#define BQ76952_BLOCK_DATA_BASE 0x40   // Block data transfer buffer start
#define BQ76952_BLOCK_DATA_END  0x5F   // Block data transfer buffer end
#define BQ76952_BLOCK_DATA_SIZE 32     // Block data transfer buffer size

// Block Data Control Registers
#define BQ76952_BLOCK_DATA_CHECKSUM 0x60 // Block data checksum register
#define BQ76952_BLOCK_DATA_CONTROL  0x61 // Block data control register

// Subcommand Addresses (16-bit)
#define BQ76952_SUBCMD_CONTROL_STATUS    0x0000  // Control Status subcommand
#define BQ76952_SUBCMD_DEVICE_TYPE       0x0001  // Device Type subcommand
#define BQ76952_SUBCMD_FW_VERSION        0x0002  // Firmware Version subcommand
#define BQ76952_SUBCMD_HW_VERSION        0x0003  // Hardware Version subcommand
#define BQ76952_SUBCMD_MANU_DATA         0x0070  // Manufacturing Data subcommand
#define BQ76952_SUBCMD_DASTATUS1         0x0071  // DAStatus1 subcommand
#define BQ76952_SUBCMD_DASTATUS2         0x0072  // DAStatus2 subcommand
#define BQ76952_SUBCMD_DASTATUS3         0x0073  // DAStatus3 subcommand
#define BQ76952_SUBCMD_DASTATUS4         0x0074  // DAStatus4 subcommand
#define BQ76952_SUBCMD_DASTATUS5         0x0075  // DAStatus5 subcommand
#define BQ76952_SUBCMD_DASTATUS6         0x0076  // DAStatus6 subcommand
#define BQ76952_SUBCMD_DASTATUS7         0x0077  // DAStatus7 subcommand

// Data Memory Addresses (16-bit) - Configuration Parameters
#define BQ76952_DM_CELL_BALANCING        0x9200  // Cell balancing configuration
#define BQ76952_DM_PROTECTION_CONFIG     0x9250  // Protection configuration
#define BQ76952_DM_ALARM_CONFIG          0x9280  // Alarm configuration

// Temperature Threshold Data Memory Addresses
#define BQ76952_OTC_THRESHOLD           0x929A  // Over Temperature Charge threshold (1 byte, signed)
#define BQ76952_OTD_THRESHOLD           0x929B  // Over Temperature Discharge threshold (1 byte, signed)
#define BQ76952_OTF_THRESHOLD           0x929C  // Over Temperature FET threshold (1 byte, signed)
#define BQ76952_UTC_THRESHOLD           0x92A6  // Under Temperature Charge threshold (1 byte, signed)
#define BQ76952_UTD_THRESHOLD           0x92A7  // Under Temperature Discharge threshold (1 byte, signed)

// Voltage Threshold Data Memory Addresses
#define BQ76952_CUV_THRESHOLD           0x9275  // Cell Undervoltage threshold (2 bytes)
#define BQ76952_COV_THRESHOLD           0x9277  // Cell Overvoltage threshold (2 bytes)
#define BQ76952_SUV_THRESHOLD           0x9279  // Stack Undervoltage threshold (2 bytes)
#define BQ76952_SOV_THRESHOLD           0x927B  // Stack Overvoltage threshold (2 bytes)

// Current Threshold Data Memory Addresses
#define BQ76952_OCC_THRESHOLD           0x9280  // Overcurrent in Charge threshold (2 bytes)
#define BQ76952_OCD1_THRESHOLD          0x9282  // Overcurrent in Discharge 1 threshold (2 bytes)
#define BQ76952_OCD2_THRESHOLD          0x9284  // Overcurrent in Discharge 2 threshold (2 bytes)
#define BQ76952_OCD3_THRESHOLD          0x9286  // Overcurrent in Discharge 3 threshold (2 bytes)
#define BQ76952_SCC_THRESHOLD           0x9288  // Short Circuit in Charge threshold (2 bytes)
#define BQ76952_SCD_THRESHOLD           0x928A  // Short Circuit in Discharge threshold (2 bytes)

// Manufacturing and Calibration Data Memory
#define BQ76952_DM_MANU_INFO            0x9300  // Manufacturing information
#define BQ76952_DM_CALIBRATION          0x9180  // Calibration data base address

// Bit Definitions for Status Registers
// Control Status Register Bits
#define BQ76952_STATUS_SS               (1 << 0)  // Safety Status
#define BQ76952_STATUS_PF               (1 << 1)  // Permanent Fail Status
#define BQ76952_STATUS_QIM              (1 << 2)  // Qmax Invalid Condition
#define BQ76952_STATUS_CB               (1 << 3)  // Cell Balancing Status
#define BQ76952_STATUS_DSG              (1 << 8)  // Discharge FET Status
#define BQ76952_STATUS_CHG              (1 << 9)  // Charge FET Status
#define BQ76952_STATUS_PCHG             (1 << 10) // Precharge FET Status

// Battery Status Register Bits
#define BQ76952_BAT_STATUS_RSVD         (1 << 0)  // Reserved
#define BQ76952_BAT_STATUS_VDQ          (1 << 1)  // Valid Discharge Qualified
#define BQ76952_BAT_STATUS_INITCOMP     (1 << 2)  // Initialization Complete
#define BQ76952_BAT_STATUS_TDA          (1 << 3)  // Thermistor Data Available
#define BQ76952_BAT_STATUS_CFGUPDATE    (1 << 4)  // Configuration Update Mode
#define BQ76952_BAT_STATUS_OTPW         (1 << 7)  // OTP Write Mode
#define BQ76952_BAT_STATUS_SLEEP        (1 << 8)  // Sleep Mode
#define BQ76952_BAT_STATUS_DEEPSLEEP    (1 << 9)  // DeepSleep Mode
#define BQ76952_BAT_STATUS_SHUTDOWN     (1 << 10) // Shutdown Mode

// Timing Constants
#define BQ76952_RESET_DELAY_MS          50    // Delay after reset command
#define BQ76952_SUBCOMMAND_DELAY_MS     50    // Delay after subcommand
#define BQ76952_DATA_MEMORY_DELAY_MS    200   // Delay for data memory operations
#define BQ76952_COMMUNICATION_RETRY     3     // Number of communication retries

// Utility Macros
#define BQ76952_CELSIUS_TO_KELVIN(c)    ((c) + 273)    // Convert Celsius to Kelvin
#define BQ76952_KELVIN_TO_CELSIUS(k)    ((k) - 273)    // Convert Kelvin to Celsius
#define BQ76952_VOLTAGE_LSB_MV          1              // Voltage LSB in millivolts
#define BQ76952_CURRENT_LSB_MA          1              // Current LSB in milliamps
#define BQ76952_TEMP_LSB_CELSIUS        1              // Temperature LSB in Celsius

// Error Codes specific to BQ76952
#define BQ76952_ERROR_NONE              0x00   // No error
#define BQ76952_ERROR_INVALID_CELL      0x01   // Invalid cell number
#define BQ76952_ERROR_I2C_TIMEOUT       0x02   // I2C communication timeout
#define BQ76952_ERROR_I2C_NACK          0x03   // I2C NACK received
#define BQ76952_ERROR_INVALID_ADDRESS   0x04   // Invalid memory address
#define BQ76952_ERROR_INVALID_LENGTH    0x05   // Invalid data length
#define BQ76952_ERROR_CHECKSUM_FAIL     0x06   // Checksum verification failed
#define BQ76952_ERROR_SUBCOMMAND_FAIL   0x07   // Subcommand execution failed
#define BQ76952_ERROR_DATA_MEMORY_FAIL  0x08   // Data memory operation failed
#define BQ76952_ERROR_DEVICE_NOT_READY  0x09   // Device not ready for operation

// Temperature Limits (in Celsius)
#define BQ76952_TEMP_MIN_CELSIUS        -40    // Minimum temperature in Celsius
#define BQ76952_TEMP_MAX_CELSIUS        120    // Maximum temperature in Celsius

// Voltage Limits (in millivolts)
#define BQ76952_CELL_VOLTAGE_MIN_MV     1000   // Minimum cell voltage (1V)
#define BQ76952_CELL_VOLTAGE_MAX_MV     5000   // Maximum cell voltage (5V)
#define BQ76952_STACK_VOLTAGE_MIN_MV    3000   // Minimum stack voltage (3V)
#define BQ76952_STACK_VOLTAGE_MAX_MV    80000  // Maximum stack voltage (80V)

// Current Limits (in milliamps)
#define BQ76952_CURRENT_MIN_MA          -200000 // Minimum current (-200A)
#define BQ76952_CURRENT_MAX_MA          200000  // Maximum current (200A)

/* Function Prototypes -------------------------------------------------------*/
// Basic Communication Functions
uint16_t BQ76952_ReadStackVoltage(void);
uint16_t BQ76952_ReadCellVoltage(uint8_t cell_number);
int16_t BQ76952_ReadCurrent(void);
int8_t BQ76952_ReadInternalTemperature(void);
int8_t BQ76952_ReadThermistorTemperature(uint8_t thermistor_number);

// Data Memory Functions
HAL_StatusTypeDef BQ76952_WriteDataMemory(uint16_t address, uint8_t *data, uint8_t length);
HAL_StatusTypeDef BQ76952_ReadDataMemory(uint16_t address, uint8_t *data, uint8_t length);
HAL_StatusTypeDef BQ76952_ReadDataMemory_Simple(uint16_t address, uint8_t *data, uint8_t length);

// Configuration Functions
HAL_StatusTypeDef BQ76952_SetTemperatureThresholds(int8_t min_temp_c, int8_t max_temp_c);
HAL_StatusTypeDef BQ76952_ReadTemperatureThresholds(int8_t *min_temp_c, int8_t *max_temp_c);
HAL_StatusTypeDef BQ76952_SetVoltageThresholds(uint16_t cuv_mv, uint16_t cov_mv);
HAL_StatusTypeDef BQ76952_ReadVoltageThresholds(uint16_t *cuv_mv, uint16_t *cov_mv);
HAL_StatusTypeDef BQ76952_SetCurrentThresholds(int16_t occ_ma, int16_t ocd_ma);
HAL_StatusTypeDef BQ76952_ReadCurrentThresholds(int16_t *occ_ma, int16_t *ocd_ma);

// Status and Control Functions
uint16_t BQ76952_ReadControlStatus(void);
uint16_t BQ76952_ReadBatteryStatus(void);
uint16_t BQ76952_ReadSafetyStatus(void);
HAL_StatusTypeDef BQ76952_EnableCellBalancing(uint16_t cell_mask);
HAL_StatusTypeDef BQ76952_DisableCellBalancing(void);
HAL_StatusTypeDef BQ76952_GetCellBalancingStatus(uint16_t *cell_mask);

// Subcommand Functions
HAL_StatusTypeDef BQ76952_SendSubcommand(uint16_t subcommand);
HAL_StatusTypeDef BQ76952_ReadSubcommandData(uint8_t *data, uint8_t length);
HAL_StatusTypeDef BQ76952_GetDeviceType(uint16_t *device_type);
HAL_StatusTypeDef BQ76952_GetFirmwareVersion(uint16_t *fw_version);
HAL_StatusTypeDef BQ76952_GetHardwareVersion(uint16_t *hw_version);

// System Functions
HAL_StatusTypeDef BQ76952_ResetCommState(void);
HAL_StatusTypeDef BQ76952_EnterSleepMode(void);
HAL_StatusTypeDef BQ76952_ExitSleepMode(void);
HAL_StatusTypeDef BQ76952_SoftwareReset(void);

// Utility Functions
HAL_StatusTypeDef BQ76952_VerifyI2CConnection(void);
uint8_t BQ76952_CalculateChecksum(uint8_t *data, uint8_t length);
HAL_StatusTypeDef BQ76952_WaitForSubcommandComplete(uint32_t timeout_ms);

/* External Variables --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;

#ifdef __cplusplus
}
#endif

#endif /* __BQ76952_DRIVER_H */
