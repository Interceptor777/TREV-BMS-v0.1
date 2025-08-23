# TREV-BMS-v0.1

## Overview

A Battery Management System (BMS) firmware for STM32L432KC microcontroller that interfaces with a TI BQ76952 battery monitor IC and provides real-time telemetry over CAN bus. Designed for 16-cell lithium-ion battery packs with integrated thermistor monitoring.

## Hardware Architecture

- **MCU**: STM32L432KC (Cortex-M4, 80MHz)
- **BMS IC**: Texas Instruments BQ76952 (16-cell battery monitor)
- **Communication**: CAN 2.0B (500 kbps), I2C (BQ76952 interface)
- **ADC**: 8-channel multiplexed thermistor monitoring
- **GPIO**: 3-pin analog multiplexer control (MUX_SIG1, MUX_SIG2, MUX_SIG3)

## Core Features

### Battery Monitoring
- **Cell Voltages**: Individual monitoring of up to 16 lithium cells via BQ76952
- **Stack Voltage**: Total pack voltage measurement
- **Temperature Monitoring**: 8 thermistor channels via analog multiplexer
- **Safety**: Configurable temperature thresholds with CAN-based remote configuration

### CAN Communication
- **Telemetry Broadcasting**: Real-time battery data transmission
- **Command Interface**: Remote parameter configuration
- **Multiplexed Messaging**: Efficient data packing for high-throughput telemetry
- **Standard/Extended ID Support**: Compatible with 11-bit and 29-bit CAN identifiers

### Real-Time Operation
- **10ms Main Loop**: Fast cycling through multiplexed channels
- **Distributed Processing**: Heavy operations spread across multiple cycles
- **Interrupt-Driven CAN**: Non-blocking message reception

## CAN Protocol

### Message Categories
- **0x100-0x11F**: Command/Response (Read/Write operations)
- **0x600-0x61F**: Telemetry (Continuous data broadcasting)  
- **0x700-0x71F**: Debug/Legacy messages

### Key Messages
| ID | Description | Data Format |
|----|-------------|-------------|
| `0x600` | Multiplexed ADC voltages | 4 messages × 2 channels each |
| `0x601` | Stack voltage | 16-bit value in mV |
| `0x602` | Cell voltages | 4 messages × 4 cells (14-bit packed) |
| `0x603` | Heartbeat | Counter + status flags |
| `0x604-0x605` | Thermistor temps | 4 temperatures per message (0.1°C resolution) |

## File Structure

### Core Application
- `main.c` - Main control loop and peripheral initialization
- `system_config.h` - CAN IDs, error codes, global definitions

### Driver Modules
- `bq76952_driver.c/.h` - BQ76952 I2C communication and battery monitoring
- `can_handler.c/.h` - CAN message processing and telemetry transmission
- `adc_handler.c/.h` - ADC operations and thermistor multiplexing

### Configuration
- `iso_can.dbc` - CAN database for message definitions
- `STM32Make.make` - Build configuration
- `openocd.cfg` - Debug/flash configuration

## Build System

```bash
# Clean build
make -f STM32Make.make clean

# Compile (Debug)
make -f STM32Make.make DEBUG=1

# Flash to target
make -f STM32Make.make flash
```

**Requirements**: ARM GCC toolchain, OpenOCD, STM32 HAL drivers

## Technical Specifications

- **CAN Bus**: 500 kbps, 16MHz time quantum
- **I2C**: 400 kHz (BQ76952 communication)
- **ADC**: 12-bit resolution, 2.5μs sampling
- **Update Rates**: 
  - Thermistors: 0.1 Hz per channel
  - Cell voltages: ~0.2 Hz (all cells)
  - Heartbeat: 0.1 Hz

## Memory Usage
- **Flash**: ~64KB (optimized build)
- **RAM**: ~8KB (including HAL stack)
- **EEPROM**: Not used (BQ76952 handles parameter storage)

## Safety Features
- Hardware watchdog via STM32 internal WDT
- I2C timeout protection with retry logic
- CAN bus auto-recovery from bus-off state
- Temperature threshold monitoring with configurable limits
- Error reporting via dedicated CAN messages

---
*Built with STM32CubeIDE HAL v1.4 | Texas Instruments BQ76952 SDK*
