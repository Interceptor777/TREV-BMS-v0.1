/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : adc_handler.c
  * @brief          : ADC operations handler implementation
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
#include "adc_handler.h"

/* Function Implementations --------------------------------------------------*/

/**
  * @brief  Read ADC value from PA0 (ADC channel 5)
  * @retval ADC value (0-4095 for 12-bit resolution)
  */
uint16_t ADC_ReadValue(void)
{
  uint16_t adc_value = 0;
  
  // Start ADC conversion
  if (HAL_ADC_Start(&hadc1) == HAL_OK) {
    // Wait for conversion to complete
    if (HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK) {
      // Get ADC value
      adc_value = HAL_ADC_GetValue(&hadc1);
    }
    // Stop ADC
    HAL_ADC_Stop(&hadc1);
  }
  
  return adc_value;
}

/**
  * @brief  Set MUX channel using 3-bit digital control signals
  * @param  channel: MUX channel to select (0-7)
  * @retval None
  */
void MUX_SetChannel(uint8_t channel)
{
  // Validate channel range
  if (channel > 7) {
    return;
  }
  
  // Set MUX control signals based on 3-bit channel value
  // MUX_SIG1 (PB1) = bit 0
  // MUX_SIG2 (PA8) = bit 1  
  // MUX_SIG3 (PB5) = bit 2
  
  if (channel & 0x01) {
    HAL_GPIO_WritePin(MUX_SIG1_PORT, MUX_SIG1_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(MUX_SIG1_PORT, MUX_SIG1_PIN, GPIO_PIN_RESET);
  }
  
  if (channel & 0x02) {
    HAL_GPIO_WritePin(MUX_SIG2_PORT, MUX_SIG2_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(MUX_SIG2_PORT, MUX_SIG2_PIN, GPIO_PIN_RESET);
  }
  
  if (channel & 0x04) {
    HAL_GPIO_WritePin(MUX_SIG3_PORT, MUX_SIG3_PIN, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(MUX_SIG3_PORT, MUX_SIG3_PIN, GPIO_PIN_RESET);
  }
  
  // Allow time for MUX to settle
  HAL_Delay(2);
}

/**
  * @brief  Read ADC value from specific MUX channel
  * @param  channel: MUX channel to read (0-7)
  * @retval ADC value (0-4095 for 12-bit resolution)
  */
uint16_t MUX_ReadChannel(uint8_t channel)
{
  // Set MUX to desired channel
  MUX_SetChannel(channel);
  
  // Read ADC value
  return ADC_ReadValue();
}

/**
  * @brief  Calculate temperature from thermistor ADC reading using B-parameter equation
  * @param  adc_value: Raw ADC value (0-4095)
  * @retval Temperature in degrees Celsius
  */
float ADC_CalculateThermistorTemp(uint16_t adc_value)
{
  // Convert ADC value to voltage
  float voltage = ((float)adc_value / ADC_RESOLUTION) * ADC_VREF;
  
  // Calculate thermistor resistance using voltage divider
  // Circuit: 5V -> 10kΩ pullup -> ADC_input -> Thermistor -> GND
  // But ADC measures voltage at the junction, limited to 3.3V max
  // 
  // For voltage divider: V_adc = 5V * (R_thermistor / (R_pullup + R_thermistor))
  // Solving for R_thermistor: R_thermistor = (V_adc * R_pullup) / (5V - V_adc)
  // 
  // However, if the actual voltage would exceed 3.3V, we need to account for clamping
  float r_thermistor;
  
  if (voltage >= 3.29) {  // Close to 3.3V limit, assume clamped
    // When clamped, we know the actual voltage divider would give higher voltage
    // This happens when thermistor resistance is very high (cold temperatures)
    // Use a minimum resistance calculation
    r_thermistor = PULLUP_RESISTOR * 100;  // Assume very high resistance for cold temps
  } else {
    // Normal calculation - assume 5V supply for voltage divider
    r_thermistor = (voltage * PULLUP_RESISTOR) / (5.0 - voltage);
  }
  
  // Handle edge cases
  if (r_thermistor <= 0) {
    return 125.0; // Return maximum temperature for very low resistance
  }
  
  // Calculate temperature using B-parameter equation (derived from Steinhart-Hart)
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  // Where T0 = 298.15K (25°C), R0 = resistance at 25°C, B = B-value
  float ln_ratio = logf(r_thermistor / THERMISTOR_R25);
  float temp_kelvin = 1.0f / ((1.0f / REFERENCE_TEMP_K) + (ln_ratio / THERMISTOR_B_VALUE));
  
  // Convert to Celsius
  float temp_celsius = temp_kelvin - 273.15f;
  
  // Clamp to reasonable range
  if (temp_celsius < -40.0f) {
    temp_celsius = -40.0f;
  } else if (temp_celsius > 125.0f) {
    temp_celsius = 125.0f;
  }
  
  return temp_celsius;
}

/**
  * @brief  Read temperatures from all 8 MUX channels
  * @param  temperatures: Array to store 8 temperature readings
  * @retval None
  */
void ADC_ReadAllThermistors(float *temperatures)
{
  for (uint8_t channel = 0; channel < MUX_CHANNELS; channel++) {
    // Read ADC value for this channel
    uint16_t adc_value = MUX_ReadChannel(channel);
    
    // Calculate temperature
    temperatures[channel] = ADC_CalculateThermistorTemp(adc_value);
    
    // Small delay between readings
    HAL_Delay(1);
  }
}

/**
  * @brief  Convert ADC value to voltage
  * @param  adc_value: Raw ADC value (0-4095)
  * @retval Voltage in volts
  */
float ADC_ConvertToVoltage(uint16_t adc_value)
{
  return ((float)adc_value / ADC_RESOLUTION) * ADC_VREF;
}

/**
  * @brief  Read voltages from all 8 MUX channels
  * @param  voltages: Array to store 8 voltage readings
  * @retval None
  */
void ADC_ReadAllVoltages(float *voltages)
{
  for (uint8_t channel = 0; channel < MUX_CHANNELS; channel++) {
    // Read ADC value for this channel
    uint16_t adc_value = MUX_ReadChannel(channel);
    
    // Convert to voltage
    voltages[channel] = ADC_ConvertToVoltage(adc_value);
    
    // Small delay between readings
    HAL_Delay(1);
  }
}
