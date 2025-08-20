# STM32L432KC ISO CAN Test Project - Modular Structure

This project has been refactored into a modular architecture for better organization, maintainability, and reusability. The code is now split into logical modules that handle specific functionality.

## Project Structure

### Core Files

#### `main.c` - Main Application Loop
- System initialization and peripheral configuration
- Main application state machine
- Coordinates between all modules
- Significantly reduced from ~1400 lines to ~300 lines

#### `main.h` - Main Header File  
- Includes all module headers
- GPIO pin definitions
- Function prototypes for STM32CubeMX generated functions

### Module Files

#### 1. **System Configuration Module**
- **Files**: `system_config.h`, `system_config.c`
- **Purpose**: Central configuration and error handling
- **Contains**:
  - Error code definitions
  - CAN ID definitions for all message types
  - Global variables for inter-module communication
  - System-wide error message function

#### 2. **BQ76952 Driver Module**
- **Files**: `bq76952_driver.h`, `bq76952_driver.c`
- **Purpose**: Complete BMS communication interface
- **Contains**:
  - All BQ76952 register and memory address definitions
  - Stack voltage and cell voltage reading functions
  - Data memory read/write operations
  - Temperature threshold management
  - Communication state reset and retry logic
  - I2C error handling and recovery

#### 3. **CAN Handler Module**
- **Files**: `can_handler.h`, `can_handler.c`
- **Contains**:
  - CAN filter configuration
  - All CAN transmission functions (ADC, stack voltage, cell voltages)
  - Multiplexed cell voltage transmission (4 mux values)
  - Temperature request/response handling
  - CAN receive callback and message processing
  - Heartbeat message transmission
  - Error message transmission

#### 4. **ADC Handler Module**
- **Files**: `adc_handler.h`, `adc_handler.c`
- **Purpose**: ADC operations and thermistor readings
- **Contains**:
  - ADC reading functions
  - Future expansion for multiplexer control
  - ADC calibration functions (extensible)

## Benefits of Modular Structure

### 1. **Maintainability**
- Each module has a single responsibility
- Easy to locate and fix bugs in specific functionality
- Clear separation of concerns

### 2. **Readability**
- Much shorter main.c file (~300 lines vs ~1400 lines)
- Related functions grouped together
- Clear header files define interfaces

### 3. **Reusability**
- BQ76952 driver can be reused in other projects
- CAN handler is modular and configurable
- ADC handler can be extended for other sensors

### 4. **Testability**
- Each module can be tested independently
- Mock interfaces can be created for unit testing
- Easier to isolate issues

### 5. **Scalability**
- Easy to add new modules (e.g., EEPROM driver, sensor modules)
- Can add more sophisticated state machines
- Simple to extend functionality

## Function Distribution

### Moved from main.c to modules:

**BQ76952 Driver (bq76952_driver.c)**:
- `BQ76952_ReadStackVoltage()`
- `BQ76952_ReadCellVoltage()`
- `BQ76952_WriteDataMemory()`
- `BQ76952_ReadDataMemory()`
- `BQ76952_ReadDataMemory_Simple()`
- `BQ76952_ResetCommState()`
- `BQ76952_SetTemperatureThresholds()`
- `BQ76952_ReadTemperatureThresholds()`

**CAN Handler (can_handler.c)**:
- `CAN_Config()`
- `CAN_TxData()`
- `CAN_TxStackVoltage()`
- `CAN_TxAllCellVoltagesMux()`
- `CAN_SendErrorMessage()`
- `CAN_SendHeartbeat()`
- `CAN_ProcessTemperatureRequests()`
- `HAL_CAN_RxFifo0MsgPendingCallback()`

**ADC Handler (adc_handler.c)**:
- `ADC_ReadValue()`

**System Config (system_config.c)**:
- Global variable definitions
- `System_SendErrorMessage()` wrapper

## Compilation

All files should be included in your STM32CubeIDE project. The modular structure doesn't change the compilation process - just ensure all `.c` files are in your source paths and all `.h` files are in your include paths.

## Future Enhancements

With this modular structure, you can easily add:

1. **EEPROM Module** - For configuration storage
2. **Sensor Module** - For additional temperature sensors
3. **State Machine Module** - For complex application logic
4. **Communication Protocol Module** - For higher-level protocols
5. **Diagnostic Module** - For system health monitoring

## Dependencies

Each module has clear dependencies:
- **BQ76952 Driver**: Depends on `system_config` for error reporting
- **CAN Handler**: Depends on `system_config` and `bq76952_driver`
- **ADC Handler**: Standalone module
- **System Config**: Depends on `can_handler` for error transmission
- **Main**: Coordinates all modules

This creates a clean dependency hierarchy with minimal circular dependencies.
