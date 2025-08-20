/* 
 * BQ76952 I2C Communication Constants Reference
 * =============================================
 * 
 * This document provides a comprehensive reference for all BQ76952 I2C 
 * communication constants defined in bq76952_driver.h, based on the 
 * official Texas Instruments BQ76952 Technical Reference Manual.
 */

// =============================================================================
// I2C COMMUNICATION BASICS
// =============================================================================
#define BQ76952_I2C_ADDRESS     0x08   // 7-bit I2C slave address
#define BQ76952_I2C_TIMEOUT     1000   // I2C operation timeout (ms)

// =============================================================================
// DIRECT COMMAND REGISTERS (7-bit addresses)
// =============================================================================
// These registers can be accessed directly via I2C without subcommands

// Status Registers
#define BQ76952_CONTROL_STATUS  0x00   // Main control and status (2 bytes)
#define BQ76952_BATTERY_STATUS  0x12   // Battery operational status (2 bytes)
#define BQ76952_ALARM_STATUS    0x13   // Alarm conditions status (2 bytes)

// Safety System Registers
#define BQ76952_SAFETY_ALERT_A  0x02   // Safety Alert A (2 bytes)
#define BQ76952_SAFETY_STATUS_A 0x03   // Safety Status A (2 bytes)
#define BQ76952_SAFETY_ALERT_B  0x04   // Safety Alert B (2 bytes)
#define BQ76952_SAFETY_STATUS_B 0x05   // Safety Status B (2 bytes)
#define BQ76952_SAFETY_ALERT_C  0x06   // Safety Alert C (2 bytes)
#define BQ76952_SAFETY_STATUS_C 0x07   // Safety Status C (2 bytes)

// Permanent Fail Registers
#define BQ76952_PF_ALERT_A      0x08   // Permanent Fail Alert A (2 bytes)
#define BQ76952_PF_STATUS_A     0x09   // Permanent Fail Status A (2 bytes)
#define BQ76952_PF_ALERT_B      0x0A   // Permanent Fail Alert B (2 bytes)
#define BQ76952_PF_STATUS_B     0x0B   // Permanent Fail Status B (2 bytes)
#define BQ76952_PF_ALERT_C      0x0C   // Permanent Fail Alert C (2 bytes)
#define BQ76952_PF_STATUS_C     0x0D   // Permanent Fail Status C (2 bytes)
#define BQ76952_PF_ALERT_D      0x0E   // Permanent Fail Alert D (2 bytes)
#define BQ76952_PF_STATUS_D     0x0F   // Permanent Fail Status D (2 bytes)

// =============================================================================
// VOLTAGE MEASUREMENT REGISTERS (2 bytes each, little endian)
// =============================================================================
// Cell voltage registers return voltage in 1mV resolution
#define BQ76952_CELL1_VOLTAGE   0x14   // Cell 1 voltage (2 bytes)
#define BQ76952_CELL2_VOLTAGE   0x16   // Cell 2 voltage (2 bytes)
#define BQ76952_CELL3_VOLTAGE   0x18   // Cell 3 voltage (2 bytes)
#define BQ76952_CELL4_VOLTAGE   0x1A   // Cell 4 voltage (2 bytes)
#define BQ76952_CELL5_VOLTAGE   0x1C   // Cell 5 voltage (2 bytes)
#define BQ76952_CELL6_VOLTAGE   0x1E   // Cell 6 voltage (2 bytes)
#define BQ76952_CELL7_VOLTAGE   0x20   // Cell 7 voltage (2 bytes)
#define BQ76952_CELL8_VOLTAGE   0x22   // Cell 8 voltage (2 bytes)
#define BQ76952_CELL9_VOLTAGE   0x24   // Cell 9 voltage (2 bytes)
#define BQ76952_CELL10_VOLTAGE  0x26   // Cell 10 voltage (2 bytes)
#define BQ76952_CELL11_VOLTAGE  0x28   // Cell 11 voltage (2 bytes)
#define BQ76952_CELL12_VOLTAGE  0x2A   // Cell 12 voltage (2 bytes)
#define BQ76952_CELL13_VOLTAGE  0x2C   // Cell 13 voltage (2 bytes)
#define BQ76952_CELL14_VOLTAGE  0x2E   // Cell 14 voltage (2 bytes)
#define BQ76952_CELL15_VOLTAGE  0x30   // Cell 15 voltage (2 bytes)
#define BQ76952_CELL16_VOLTAGE  0x32   // Cell 16 voltage (2 bytes)

// Stack and Pin Voltages
#define BQ76952_STACK_VOLTAGE   0x34   // Total stack voltage (2 bytes, 0.01V resolution)
#define BQ76952_PACK_PIN_VOLTAGE 0x36  // PACK pin voltage (2 bytes)
#define BQ76952_LD_PIN_VOLTAGE  0x38   // LD pin voltage (2 bytes)

// =============================================================================
// CURRENT MEASUREMENT REGISTERS (2 bytes each, signed)
// =============================================================================
#define BQ76952_CC2_CURRENT     0x3A   // Coulomb Counter 2 current (2 bytes)
#define BQ76952_CC1_CURRENT     0x3C   // Coulomb Counter 1 current (2 bytes)

// =============================================================================
// SUBCOMMAND INTERFACE
// =============================================================================
// Used for accessing extended functionality and data memory
#define BQ76952_SUBCOMMAND_LOW  0x3E   // Subcommand low byte register
#define BQ76952_SUBCOMMAND_HIGH 0x3F   // Subcommand high byte register

// Block Data Transfer Buffer (for data memory operations)
#define BQ76952_BLOCK_DATA_BASE 0x40   // Start of 32-byte transfer buffer
#define BQ76952_BLOCK_DATA_END  0x5F   // End of 32-byte transfer buffer
#define BQ76952_BLOCK_DATA_SIZE 32     // Size of transfer buffer

// Block Data Control
#define BQ76952_BLOCK_DATA_CHECKSUM 0x60 // Checksum for transfer buffer
#define BQ76952_BLOCK_DATA_CONTROL  0x61 // Control register for transfer

// =============================================================================
// SUBCOMMAND ADDRESSES (16-bit values written to 0x3E/0x3F)
// =============================================================================
#define BQ76952_SUBCMD_CONTROL_STATUS    0x0000  // Read control status
#define BQ76952_SUBCMD_DEVICE_TYPE       0x0001  // Device type identification
#define BQ76952_SUBCMD_FW_VERSION        0x0002  // Firmware version
#define BQ76952_SUBCMD_HW_VERSION        0x0003  // Hardware version
#define BQ76952_SUBCMD_MANU_DATA         0x0070  // Manufacturing data access

// Data Acquisition Status Commands
#define BQ76952_SUBCMD_DASTATUS1         0x0071  // DA Status 1 (cells 1-4)
#define BQ76952_SUBCMD_DASTATUS2         0x0072  // DA Status 2 (cells 5-8)
#define BQ76952_SUBCMD_DASTATUS3         0x0073  // DA Status 3 (cells 9-12)
#define BQ76952_SUBCMD_DASTATUS4         0x0074  // DA Status 4 (cells 13-16)
#define BQ76952_SUBCMD_DASTATUS5         0x0075  // DA Status 5 (temperatures)
#define BQ76952_SUBCMD_DASTATUS6         0x0076  // DA Status 6 (additional data)
#define BQ76952_SUBCMD_DASTATUS7         0x0077  // DA Status 7 (misc measurements)

// =============================================================================
// DATA MEMORY ADDRESSES (16-bit addresses for configuration parameters)
// =============================================================================

// Configuration Sections
#define BQ76952_DM_CELL_BALANCING        0x9200  // Cell balancing configuration
#define BQ76952_DM_PROTECTION_CONFIG     0x9250  // Protection settings
#define BQ76952_DM_ALARM_CONFIG          0x9280  // Alarm thresholds

// Temperature Thresholds (signed 8-bit values in Celsius)
#define BQ76952_OTC_THRESHOLD           0x929A  // Over Temperature Charge
#define BQ76952_OTD_THRESHOLD           0x929B  // Over Temperature Discharge
#define BQ76952_OTF_THRESHOLD           0x929C  // Over Temperature FET
#define BQ76952_UTC_THRESHOLD           0x92A6  // Under Temperature Charge
#define BQ76952_UTD_THRESHOLD           0x92A7  // Under Temperature Discharge

// Voltage Thresholds (16-bit values in mV)
#define BQ76952_CUV_THRESHOLD           0x9275  // Cell Undervoltage
#define BQ76952_COV_THRESHOLD           0x9277  // Cell Overvoltage
#define BQ76952_SUV_THRESHOLD           0x9279  // Stack Undervoltage
#define BQ76952_SOV_THRESHOLD           0x927B  // Stack Overvoltage

// Current Thresholds (16-bit signed values)
#define BQ76952_OCC_THRESHOLD           0x9280  // Overcurrent in Charge
#define BQ76952_OCD1_THRESHOLD          0x9282  // Overcurrent in Discharge 1
#define BQ76952_OCD2_THRESHOLD          0x9284  // Overcurrent in Discharge 2
#define BQ76952_OCD3_THRESHOLD          0x9286  // Overcurrent in Discharge 3
#define BQ76952_SCC_THRESHOLD           0x9288  // Short Circuit in Charge
#define BQ76952_SCD_THRESHOLD           0x928A  // Short Circuit in Discharge

// Calibration and Manufacturing Data
#define BQ76952_DM_MANU_INFO            0x9300  // Manufacturing information
#define BQ76952_DM_CALIBRATION          0x9180  // Calibration data base

// =============================================================================
// STATUS REGISTER BIT DEFINITIONS
// =============================================================================

// Control Status Register (0x00) Bit Masks
#define BQ76952_STATUS_SS               (1 << 0)  // Safety Status
#define BQ76952_STATUS_PF               (1 << 1)  // Permanent Fail Status
#define BQ76952_STATUS_QIM              (1 << 2)  // Qmax Invalid Condition
#define BQ76952_STATUS_CB               (1 << 3)  // Cell Balancing Active
#define BQ76952_STATUS_DSG              (1 << 8)  // Discharge FET Status
#define BQ76952_STATUS_CHG              (1 << 9)  // Charge FET Status
#define BQ76952_STATUS_PCHG             (1 << 10) // Precharge FET Status

// Battery Status Register (0x12) Bit Masks
#define BQ76952_BAT_STATUS_VDQ          (1 << 1)  // Valid Discharge Qualified
#define BQ76952_BAT_STATUS_INITCOMP     (1 << 2)  // Initialization Complete
#define BQ76952_BAT_STATUS_TDA          (1 << 3)  // Thermistor Data Available
#define BQ76952_BAT_STATUS_CFGUPDATE    (1 << 4)  // Configuration Update Mode
#define BQ76952_BAT_STATUS_OTPW         (1 << 7)  // OTP Write Mode
#define BQ76952_BAT_STATUS_SLEEP        (1 << 8)  // Sleep Mode Active
#define BQ76952_BAT_STATUS_DEEPSLEEP    (1 << 9)  // DeepSleep Mode Active
#define BQ76952_BAT_STATUS_SHUTDOWN     (1 << 10) // Shutdown Mode Active

// =============================================================================
// TIMING AND OPERATIONAL LIMITS
// =============================================================================

// Communication Timing
#define BQ76952_RESET_DELAY_MS          50    // Delay after reset command
#define BQ76952_SUBCOMMAND_DELAY_MS     50    // Delay after subcommand execution
#define BQ76952_DATA_MEMORY_DELAY_MS    200   // Delay for data memory operations
#define BQ76952_COMMUNICATION_RETRY     3     // Number of retry attempts

// Device Limits
#define BQ76952_MAX_CELLS               16    // Maximum supported cells
#define BQ76952_MAX_THERMISTORS         9     // Maximum external thermistors

// Temperature Limits (Celsius)
#define BQ76952_TEMP_MIN_CELSIUS        -40   // Minimum temperature
#define BQ76952_TEMP_MAX_CELSIUS        120   // Maximum temperature

// Voltage Limits (millivolts)
#define BQ76952_CELL_VOLTAGE_MIN_MV     1000  // Minimum cell voltage (1V)
#define BQ76952_CELL_VOLTAGE_MAX_MV     5000  // Maximum cell voltage (5V)
#define BQ76952_STACK_VOLTAGE_MIN_MV    3000  // Minimum stack voltage (3V)
#define BQ76952_STACK_VOLTAGE_MAX_MV    80000 // Maximum stack voltage (80V)

// Current Limits (milliamps)
#define BQ76952_CURRENT_MIN_MA          -200000 // Minimum current (-200A)
#define BQ76952_CURRENT_MAX_MA          200000  // Maximum current (+200A)

// =============================================================================
// DATA CONVERSION UTILITIES
// =============================================================================
#define BQ76952_CELSIUS_TO_KELVIN(c)    ((c) + 273)    // Convert °C to K
#define BQ76952_KELVIN_TO_CELSIUS(k)    ((k) - 273)    // Convert K to °C
#define BQ76952_VOLTAGE_LSB_MV          1              // Voltage LSB (1mV)
#define BQ76952_CURRENT_LSB_MA          1              // Current LSB (1mA)
#define BQ76952_TEMP_LSB_CELSIUS        1              // Temperature LSB (1°C)

// =============================================================================
// ERROR CODES
// =============================================================================
#define BQ76952_ERROR_NONE              0x00   // No error
#define BQ76952_ERROR_INVALID_CELL      0x01   // Invalid cell number (1-16)
#define BQ76952_ERROR_I2C_TIMEOUT       0x02   // I2C communication timeout
#define BQ76952_ERROR_I2C_NACK          0x03   // I2C NACK received
#define BQ76952_ERROR_INVALID_ADDRESS   0x04   // Invalid memory address
#define BQ76952_ERROR_INVALID_LENGTH    0x05   // Invalid data length
#define BQ76952_ERROR_CHECKSUM_FAIL     0x06   // Checksum verification failed
#define BQ76952_ERROR_SUBCOMMAND_FAIL   0x07   // Subcommand execution failed
#define BQ76952_ERROR_DATA_MEMORY_FAIL  0x08   // Data memory operation failed
#define BQ76952_ERROR_DEVICE_NOT_READY  0x09   // Device not ready

// =============================================================================
// USAGE EXAMPLES
// =============================================================================

/*
 * Example 1: Reading Cell Voltage
 * --------------------------------
 * uint16_t voltage_mv = BQ76952_ReadCellVoltage(1);  // Read cell 1 voltage
 * 
 * Example 2: Reading Stack Voltage
 * --------------------------------
 * uint16_t stack_mv = BQ76952_ReadStackVoltage();
 * 
 * Example 3: Setting Temperature Thresholds
 * -----------------------------------------
 * HAL_StatusTypeDef status = BQ76952_SetTemperatureThresholds(-10, 60);
 * // Sets charge temperature limits: -10°C to +60°C
 * 
 * Example 4: Reading Control Status
 * ---------------------------------
 * uint16_t status = BQ76952_ReadControlStatus();
 * if (status & BQ76952_STATUS_CB) {
 *     // Cell balancing is active
 * }
 * 
 * Example 5: Data Memory Operation
 * --------------------------------
 * uint8_t temp_data[2];
 * HAL_StatusTypeDef result = BQ76952_ReadDataMemory(BQ76952_OTC_THRESHOLD, temp_data, 1);
 * int8_t otc_threshold = (int8_t)temp_data[0];  // Over-temperature charge threshold
 */
