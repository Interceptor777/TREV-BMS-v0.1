# Thermistor MUX Control System

## Overview
This BMS now includes an 8-channel thermistor monitoring syst### Temperature Calculation
The system uses the B-parameter equation (derived from Steinhart-Hart) with your specific B25/85 value:
```
1/T = 1/T₀ + (1/B) × ln(R/R₀)
```
Where:
- T = Temperature in Kelvin
- T₀ = Reference temperature (298.15K = 25°C)
- B = B25/85 value of thermistor (4300K from your datasheet)
- R = Measured thermistor resistance
- R₀ = Thermistor resistance at 25°C (10kΩ)

**Key Fix Applied:**
The voltage divider calculation now correctly accounts for:
- 5V supply to the pullup resistor
- 3.3V maximum ADC input voltage
- Proper resistance calculation: R = (V_adc × R_pullup) / (5V - V_adc) 8:1 analog multiplexer controlled by three digital pins (MUX_SIG1, MUX_SIG2, MUX_SIG3).

## Hardware Configuration

### MUX Control Pins
- **MUX_SIG1** (PB1): Bit 0 of channel selection
- **MUX_SIG2** (PA8): Bit 1 of channel selection  
- **MUX_SIG3** (PB5): Bit 2 of channel selection

### ADC Input
- **PA0** (ADC1_IN5): Analog input from MUX output

### Channel Mapping
| Channel | MUX_SIG3 | MUX_SIG2 | MUX_SIG1 | Binary |
|---------|----------|----------|----------|--------|
| 0       | 0        | 0        | 0        | 000    |
| 1       | 0        | 0        | 1        | 001    |
| 2       | 0        | 1        | 0        | 010    |
| 3       | 0        | 1        | 1        | 011    |
| 4       | 1        | 0        | 0        | 100    |
| 5       | 1        | 0        | 1        | 101    |
| 6       | 1        | 1        | 0        | 110    |
| 7       | 1        | 1        | 1        | 111    |

## Thermistor Configuration

### Current Settings (in adc_handler.h)
```c
#define THERMISTOR_B_VALUE     4300    // B25/85 value of thermistor (4300K per datasheet)
#define THERMISTOR_R25         10000   // Resistance at 25°C (10kΩ)
#define REFERENCE_TEMP_K       298.15  // 25°C in Kelvin  
#define PULLUP_RESISTOR        10000   // 10kΩ pullup resistor
```

### Common Thermistor B-Values
- **Your thermistors**: B25/85 = 4300K (current setting)
- **Alternative B25/50**: 4250K (if using B25/50 instead)
- **NTC 3.3kΩ thermistors**: B-value typically 3380K  
- **NTC 47kΩ thermistors**: B-value typically 4050K

**To change the B-value:** Modify `THERMISTOR_B_VALUE` in `Core/Inc/adc_handler.h`

## CAN Message Protocol

### Individual ADC Readings (Legacy)
**CAN ID: 0x600 (TELEMETRY_ADC)**
- Byte 0: MUX channel (0-7)
- Bytes 1-2: Raw ADC value (12-bit, little endian)
- Bytes 3-7: Reserved

### All Thermistor Temperatures
**CAN ID: 0x604 (THERMISTOR_TEMPS_1_4)**
- Bytes 0-1: Temperature 1 (signed 16-bit, 0.1°C resolution)
- Bytes 2-3: Temperature 2 (signed 16-bit, 0.1°C resolution)
- Bytes 4-5: Temperature 3 (signed 16-bit, 0.1°C resolution)
- Bytes 6-7: Temperature 4 (signed 16-bit, 0.1°C resolution)

**CAN ID: 0x605 (THERMISTOR_TEMPS_5_8)**
- Bytes 0-1: Temperature 5 (signed 16-bit, 0.1°C resolution)
- Bytes 2-3: Temperature 6 (signed 16-bit, 0.1°C resolution)
- Bytes 4-5: Temperature 7 (signed 16-bit, 0.1°C resolution)
- Bytes 6-7: Temperature 8 (signed 16-bit, 0.1°C resolution)

### All ADC Voltage Readings
**CAN ID: 0x606 (ADC_VOLTAGES_1_4)**
- Bytes 0-1: Voltage 1 (unsigned 16-bit, 1mV resolution)
- Bytes 2-3: Voltage 2 (unsigned 16-bit, 1mV resolution)
- Bytes 4-5: Voltage 3 (unsigned 16-bit, 1mV resolution)
- Bytes 6-7: Voltage 4 (unsigned 16-bit, 1mV resolution)

**CAN ID: 0x607 (ADC_VOLTAGES_5_8)**
- Bytes 0-1: Voltage 5 (unsigned 16-bit, 1mV resolution)
- Bytes 2-3: Voltage 6 (unsigned 16-bit, 1mV resolution)
- Bytes 4-5: Voltage 7 (unsigned 16-bit, 1mV resolution)
- Bytes 6-7: Voltage 8 (unsigned 16-bit, 1mV resolution)

### Temperature Data Format
- **Resolution**: 0.1°C (divide by 10 to get actual temperature)
- **Range**: -40.0°C to +125.0°C
- **Example**: Value 235 = 23.5°C, Value -150 = -15.0°C

### Voltage Data Format
- **Resolution**: 1mV (divide by 1000 to get actual voltage)
- **Range**: 0.0V to 3.3V (STM32 ADC reference limit)
- **Example**: Value 2315 = 2.315V, Value 1234 = 1.234V

## Operation

### Timing
- **Individual channel cycling**: Every 1000ms (1 second per channel)
- **All thermistors read**: Every 8 seconds (every 8 cycles × 1000ms)
- **Temperature CAN messages**: Sent every 8 seconds
- **Heartbeat**: Every 10 seconds (every 10 cycles × 1000ms)

### MUX Channel Cycling Behavior
The main loop now properly cycles through MUX channels:
1. **Set MUX channel** using `MUX_SetChannel(current_channel)` 
2. **Hold for 1 second** to allow settling
3. **Read ADC** from the selected channel
4. **Send individual channel data** via CAN ID 0x600
5. **Move to next channel** and repeat

Every 8 seconds (after cycling through all channels), the system also:
- Reads all 8 channels rapidly via `ADC_ReadAllThermistors()`
- Sends complete temperature data via CAN IDs 0x604 and 0x605

### Temperature Calculation
The system uses the simplified Steinhart-Hart equation (B-parameter form):
```
1/T = 1/T0 + (1/B) × ln(R/R0)
```
Where:
- T = Temperature in Kelvin
- T0 = Reference temperature (298.15K = 25°C)
- B = B-value of thermistor (3950K)
- R = Measured thermistor resistance
- R0 = Thermistor resistance at 25°C (10kΩ)

### Circuit Configuration
Your actual circuit configuration:
```
5.0V ----[ 10kΩ pullup ]----+----[ Thermistor ]---- GND
                             |
                    ADC Input (PA0)
                             |
                        (3.3V max input)
```

**Important**: 
- Thermistor supply voltage: 5.0V
- STM32 ADC reference: 3.3V (internal)
- Maximum measurable voltage: 3.3V
- Voltage divider ensures ADC input ≤ 3.3V

## API Functions

### Core Functions
```c
void MUX_SetChannel(uint8_t channel);           // Set MUX to specific channel (0-7)
uint16_t MUX_ReadChannel(uint8_t channel);      // Read ADC from specific channel
float ADC_CalculateThermistorTemp(uint16_t adc_value);  // Convert ADC to temperature
void ADC_ReadAllThermistors(float *temperatures);      // Read all 8 channels
```

### CAN Functions
```c
void CAN_TxThermistorData(uint8_t mux_channel, uint16_t adc_value);  // Send single channel
void CAN_TxAllThermistorsMux(float *temperatures);                   // Send all temps
```

## Customization

### Different Thermistor Types
1. Update Steinhart-Hart coefficients for your specific thermistor in `Core/Inc/adc_handler.h`:
   ```c
   #define STEINHART_A            1.129148e-3  // Coefficient A
   #define STEINHART_B            2.34125e-4   // Coefficient B  
   #define STEINHART_C            8.76741e-8   // Coefficient C
   ```
2. Update `THERMISTOR_R25` if using different nominal resistance
3. Update `PULLUP_RESISTOR` if using different pullup value

**Note**: If you don't have specific Steinhart-Hart coefficients, you can derive them from the B-value:
- A ≈ (1/T₁) - (1/B) × ln(R₁)
- B ≈ (1/B)  
- C ≈ 0 (for simplified calculation)

Where T₁ = 298.15K (25°C) and R₁ = 10kΩ

### Different Voltage Reference
Update `ADC_VREF` if using different thermistor supply voltage:
- Current setting: 5.0V (for 5V thermistor supply)
- Alternative: 3.3V (for 3.3V supply)
- Must match your actual thermistor circuit supply voltage

### Different Temperature Ranges
Modify the clamping logic in `ADC_CalculateThermistorTemp()` function

## Troubleshooting

### Invalid Temperature Readings
- Check thermistor B-value setting
- Verify pullup resistor value
- Check ADC reference voltage
- Ensure proper thermistor connection

### MUX Not Switching
- Verify GPIO pin assignments match hardware
- Check MUX power supply
- Verify digital control signal levels

### CAN Message Issues
- Check CAN bus termination
- Verify message timing in CAN analyzer
- Ensure proper DBC file interpretation
