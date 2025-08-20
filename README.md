# TREV-BMS-v0.1

## 🔋 Advanced STM32L432KC Battery Management System

**TREV-BMS-v0.1** is a comprehensive Battery Management System (BMS) firmware built on the STM32L432KC microcontroller platform, designed to interface with the Texas Instruments BQ76952 multi-cell battery monitor and protector IC. This system provides real-time battery monitoring, safety protection, and CAN bus communication for electric vehicle and energy storage applications.

![STM32L432KC](https://img.shields.io/badge/STM32-L432KC-blue)
![BQ76952](https://img.shields.io/badge/BMS-BQ76952-green)
![CAN Bus](https://img.shields.io/badge/Protocol-CAN%20Bus-orange)
![License](https://img.shields.io/badge/License-STM32-yellow)

---

## 🚀 Key Features

### 🔍 **Battery Monitoring & Protection**
- **Multi-cell voltage monitoring** (up to 16 cells) via BQ76952
- **Real-time stack voltage measurement** with high precision
- **Temperature monitoring** using external thermistors
- **Configurable temperature thresholds** for thermal protection
- **Cell balancing support** for optimal battery performance

### 🌐 **CAN Bus Communication**
- **ISO CAN 2.0B** protocol implementation
- **500 kbps** communication speed
- **Comprehensive telemetry transmission** (voltages, temperatures, status)
- **Bidirectional command/response** messaging
- **Heartbeat monitoring** for system health verification
- **Error reporting** and diagnostic capabilities

### 📊 **Advanced Features**
- **Modular software architecture** for maintainability
- **ADC-based thermistor readings** with multiplexer support
- **I2C communication** with BQ76952 including error recovery
- **Real-time data multiplexing** for efficient CAN utilization
- **Configurable safety parameters** via CAN commands

---

## 🛠️ Hardware Requirements

### **Primary Components**
| Component | Part Number | Purpose |
|-----------|-------------|---------|
| **Microcontroller** | STM32L432KC | Main processing unit |
| **BMS IC** | BQ76952 | Multi-cell battery monitor |
| **CAN Transceiver** | External (e.g., TJA1050) | CAN bus physical layer |
| **Thermistors** | NTC 10kΩ | Temperature sensing |

### **STM32L432KC Specifications**
- **ARM Cortex-M4** @ 80MHz with FPU
- **256KB Flash**, 64KB SRAM
- **I2C, CAN, ADC** peripherals
- **Low power consumption** design
- **32-pin LQFP** package

### **BQ76952 Capabilities**
- **3-16 series cell support**
- **Integrated cell balancing**
- **Voltage accuracy: ±1.5mV**
- **Temperature range: -40°C to +85°C**
- **I2C communication interface**

---

## 🔌 Pin Configuration & Wiring

### **STM32L432KC Pin Assignments**
| Pin | Function | Peripheral | Description |
|-----|----------|------------|-------------|
| **PA0** | ADC_IN5 | ADC1 | Thermistor multiplexer input |
| **PA11** | CAN_RX | CAN1 | CAN bus receive |
| **PA12** | CAN_TX | CAN1 | CAN bus transmit |
| **PB6** | I2C_SCL | I2C1 | BQ76952 clock line |
| **PB7** | I2C_SDA | I2C1 | BQ76952 data line |

### **System Connection Diagram**
```
┌─────────────────┐    I2C     ┌─────────────────┐
│   STM32L432KC   │◄──────────►│     BQ76952     │
│                 │            │   (BMS IC)      │
│    PA11/CAN_RX  │◄─┐         │                 │
│    PA12/CAN_TX  │──┼─────────┤ Battery Pack    │
└─────────────────┘  │         │   (3-16 cells)  │
                     │         └─────────────────┘
                     │
              ┌─────────────┐
              │ CAN Bus     │
              │ Network     │
              └─────────────┘
```

---

## 🏗️ Software Architecture

### **Modular Design Philosophy**
The firmware follows a **clean modular architecture** with separation of concerns:

```
├── Core/
│   ├── Src/
│   │   ├── main.c              # Application entry point & coordination
│   │   ├── system_config.c     # Global configuration & error handling
│   │   ├── bq76952_driver.c    # BMS IC communication driver
│   │   ├── can_handler.c       # CAN protocol implementation
│   │   └── adc_handler.c       # ADC operations & thermistor reading
│   └── Inc/
│       ├── main.h              # Main application headers
│       ├── system_config.h     # System-wide definitions
│       ├── bq76952_driver.h    # BMS driver interface
│       ├── can_handler.h       # CAN communication interface
│       └── adc_handler.h       # ADC operations interface
```

### **Module Responsibilities**

#### 🧠 **System Configuration Module**
- **Global error code definitions**
- **CAN message ID allocation**
- **Inter-module communication variables**
- **Centralized error reporting**

#### 🔋 **BQ76952 Driver Module**
- **I2C communication with BQ76952**
- **Register and memory operations**
- **Cell/stack voltage reading**
- **Temperature threshold management**
- **Communication error recovery**

#### 📡 **CAN Handler Module**
- **CAN filter configuration**
- **Telemetry data transmission**
- **Command/response processing**
- **Message multiplexing**
- **Heartbeat generation**

#### 📈 **ADC Handler Module**
- **Thermistor voltage reading**
- **ADC calibration**
- **Future multiplexer support**

---

## 🔧 Development Setup

### **Prerequisites**
- **STM32CubeIDE** (recommended) or compatible ARM GCC toolchain
- **STM32CubeMX** for hardware configuration
- **STM32 ST-LINK Utility** or compatible programmer/debugger
- **CAN bus analyzer** (optional, for protocol testing)

### **Building the Project**

#### **Using STM32CubeIDE:**
1. **Import project**: `File → Import → Existing Projects into Workspace`
2. **Select folder**: Choose the repository root directory
3. **Build**: `Project → Build Project` or `Ctrl+B`
4. **Flash**: `Run → Debug` or use ST-LINK

#### **Using Command Line:**
```bash
# Navigate to project directory
cd TREV-BMS-v0.1/Debug

# Build using generated Makefile
make clean
make all

# Flash using ST-LINK (if available)
st-flash write ISO_CAN_TEST.bin 0x8000000
```

### **Project Configuration**
The project is pre-configured with STM32CubeMX (`.ioc` file included):
- **Clock configuration**: HSI16 → PLL → 80MHz system clock
- **CAN configuration**: 500 kbps, standard frame format
- **I2C configuration**: 400 kHz for BQ76952 communication
- **ADC configuration**: 12-bit resolution, single-ended

---

## 🌐 CAN Protocol Documentation

### **Message ID Allocation**
The CAN protocol uses a structured ID allocation scheme:

#### **Command/Response Range (0x100-0x11F)**
| ID | Direction | Purpose | Data Format |
|----|-----------|---------|-------------|
| `0x100` | Host → BMS | Read Cell Voltage Request | `[cell_number, 0, 0, 0, 0, 0, 0, 0]` |
| `0x101` | BMS → Host | Read Cell Voltage Response | `[cell_number, voltage_hi, voltage_lo, 0, 0, 0, 0, 0]` |
| `0x102` | Host → BMS | Read Stack Voltage Request | `[0, 0, 0, 0, 0, 0, 0, 0]` |
| `0x103` | BMS → Host | Read Stack Voltage Response | `[voltage_hi, voltage_lo, 0, 0, 0, 0, 0, 0]` |
| `0x104` | Host → BMS | Read Temperature Thresholds | `[0, 0, 0, 0, 0, 0, 0, 0]` |
| `0x105` | BMS → Host | Temperature Thresholds Response | `[min_hi, min_lo, max_hi, max_lo, 0, 0, 0, 0]` |
| `0x106` | Host → BMS | Write Temperature Thresholds | `[min_hi, min_lo, max_hi, max_lo, 0, 0, 0, 0]` |
| `0x107` | BMS → Host | Write Response | `[status, error_code, 0, 0, 0, 0, 0, 0]` |

#### **Telemetry Range (0x600-0x61F)**
| ID | Purpose | Data Format | Frequency |
|----|---------|-------------|-----------|
| `0x600` | ADC Thermistor Data | `[adc_hi, adc_lo, 0, 0, 0, 0, 0, 0]` | Continuous |
| `0x601` | Stack Voltage Telemetry | `[voltage_hi, voltage_lo, 0, 0, 0, 0, 0, 0]` | Continuous |
| `0x602` | Cell Voltages (Mux 0) | `[mux, cell0_hi, cell0_lo, cell1_hi, cell1_lo, cell2_hi, cell2_lo, cell3_hi]` | Continuous |
| `0x603` | Cell Voltages (Mux 1) | `[mux, cell4_hi, cell4_lo, cell5_hi, cell5_lo, cell6_hi, cell6_lo, cell7_hi]` | Continuous |

#### **System Range (0x700-0x71F)**
| ID | Purpose | Data Format | Frequency |
|----|---------|-------------|-----------|
| `0x700` | Heartbeat | `[counter_3, counter_2, counter_1, counter_0, rx_flag, 0, 0, 0]` | Every 100 cycles |
| `0x7FF` | Error Messages | `[error_code, module_id, additional_data...]` | On error |

### **Data Formats**
- **Voltages**: 16-bit values in millivolts (mV)
- **Temperatures**: 16-bit values in degrees Celsius × 10
- **ADC Values**: 12-bit raw ADC readings
- **Multi-byte values**: Big-endian format (MSB first)

---

## 🎯 Usage Instructions

### **System Startup**
1. **Power on** the STM32L432KC and BQ76952 system
2. **System initialization** occurs automatically:
   - Peripheral configuration (CAN, I2C, ADC)
   - BQ76952 communication establishment
   - CAN filters and interrupts setup
3. **Automatic telemetry** begins immediately

### **Real-time Monitoring**
The system continuously transmits:
- **ADC thermistor readings** via `0x600`
- **Stack voltage** via `0x601`
- **Individual cell voltages** via `0x602`-`0x605` (multiplexed)
- **Heartbeat messages** via `0x700` (every 100 cycles)

### **Interactive Commands**
Send CAN messages to interact with the system:

#### **Read Stack Voltage:**
```
TX: ID=0x102, Data=[00,00,00,00,00,00,00,00]
RX: ID=0x103, Data=[voltage_hi, voltage_lo, 00,00,00,00,00,00]
```

#### **Set Temperature Thresholds:**
```
TX: ID=0x106, Data=[min_hi, min_lo, max_hi, max_lo, 00,00,00,00]
RX: ID=0x107, Data=[status, error_code, 00,00,00,00,00,00]
```

### **Error Monitoring**
Monitor `0x7FF` for system errors:
- `0x01`: Temperature range invalid
- `0x02`: I2C communication failure
- `0x03`: Data memory write error
- `0x04`: Data memory read error

---

## 🧪 Testing & Validation

### **Hardware Testing**
1. **Power supply verification**: Ensure stable 3.3V supply
2. **I2C communication**: Verify BQ76952 responses
3. **CAN bus functionality**: Test with CAN analyzer
4. **ADC readings**: Validate thermistor measurements

### **Software Testing**
```bash
# Build test configuration
make clean && make all

# Check binary size
arm-none-eabi-size ISO_CAN_TEST.elf

# Flash and monitor via debugger
# Verify CAN messages using external analyzer
```

### **Performance Metrics**
- **Main loop frequency**: ~100 Hz
- **CAN message rate**: ~400 messages/second
- **I2C communication**: <10ms per BQ76952 transaction
- **Memory usage**: ~15KB Flash, ~2KB RAM

---

## 📋 Module Reference

### **BQ76952 Driver Functions**
```c
uint16_t BQ76952_ReadStackVoltage(void);
uint16_t BQ76952_ReadCellVoltage(uint8_t cell_number);
HAL_StatusTypeDef BQ76952_SetTemperatureThresholds(int16_t min_temp, int16_t max_temp);
HAL_StatusTypeDef BQ76952_ReadTemperatureThresholds(int16_t *min_temp, int16_t *max_temp);
```

### **CAN Handler Functions**
```c
void CAN_Config(void);
void CAN_TxData(uint16_t adc_value);
void CAN_TxStackVoltage(uint16_t voltage_mv);
void CAN_TxAllCellVoltagesMux(void);
void CAN_SendHeartbeat(uint32_t counter);
void CAN_ProcessTemperatureRequests(void);
```

### **ADC Handler Functions**
```c
uint16_t ADC_ReadValue(void);
HAL_StatusTypeDef ADC_Calibrate(void);
```

---

## 🔄 Contributing

### **Development Guidelines**
1. **Follow modular architecture**: Keep functionality separated by module
2. **Maintain CAN protocol compatibility**: Don't break existing message formats
3. **Add comprehensive comments**: Document complex algorithms
4. **Test thoroughly**: Verify both hardware and software functionality

### **Submission Process**
1. **Fork** the repository
2. **Create feature branch**: `git checkout -b feature/your-feature`
3. **Implement changes** following coding standards
4. **Test** with actual hardware when possible
5. **Submit pull request** with detailed description

### **Coding Standards**
- **HAL library usage**: Prefer STM32 HAL functions
- **Error handling**: Always check return values
- **Memory management**: Avoid dynamic allocation
- **Documentation**: Use Doxygen-style comments

---

## 📜 License & Acknowledgments

### **License**
This project is licensed under the **STMicroelectronics Software License Agreement**.
See `LICENSE` files in the Drivers directories for details.

### **Third-Party Components**
- **STM32L4xx HAL Drivers** - STMicroelectronics
- **CMSIS** - ARM Limited
- **BQ76952 Reference Designs** - Texas Instruments

### **Documentation Sources**
- **STM32L432KC Datasheet** - STMicroelectronics
- **BQ76952 Technical Reference Manual** - Texas Instruments
- **ISO 11898 CAN Specification** - International Organization for Standardization

### **Contributors**
- Initial development and modular refactoring
- CAN protocol implementation
- BQ76952 driver optimization

---

## 📞 Support & Resources

### **Documentation**
- 📖 **[MODULAR_STRUCTURE_README.md](MODULAR_STRUCTURE_README.md)** - Detailed module breakdown
- 📄 **BQ76952 Datasheet** (`bq76952.pdf`) - Hardware reference
- 📄 **STM32L432KC Datasheet** (`stm32l432kc-1.pdf`) - MCU reference
- 📄 **CAN DBC File** (`iso_can.dbc`) - Protocol definition

### **Hardware Resources**
- **STM32L432KC Nucleo Board** - Development platform
- **BQ76952EVM** - Texas Instruments evaluation module
- **CAN Bus Analyzers** - Protocol debugging tools

### **Online Resources**
- [STM32CubeIDE Download](https://www.st.com/en/development-tools/stm32cubeide.html)
- [BQ76952 Product Page](https://www.ti.com/product/BQ76952)
- [STM32 Community Forum](https://community.st.com/)

---

**🔋 Built for reliable battery management in electric vehicle and energy storage applications**

*For technical support or questions about this project, please refer to the documentation or create an issue in the repository.*