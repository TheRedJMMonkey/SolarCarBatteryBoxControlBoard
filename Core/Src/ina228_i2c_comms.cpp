/**
 * @file ina228_i2c_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of INA228 Asynchronous I2C communications class based on
 * Texas Instruments' INA228 datasheet at https://www.ti.com/lit/ds/symlink/ina228.pdf
 * @version 0.1
 * @date 2026-03-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ina228_i2c_comms.hpp"
#include "main.h"

// Global pointer for interrupt handler routing
INA228 *g_ina228Instance = nullptr;

INA228::INA228(I2C_HandleTypeDef *hi2c, uint8_t i2cAddr) : hi2c_(hi2c), i2cAddr_(i2cAddr) { g_ina228Instance = this; }

// ============================================================================
// Public Asynchronous API
// ============================================================================

HAL_StatusTypeDef INA228::startReadAllMeasurements() {
  if (readState_ != ReadState::Idle) {
    return HAL_BUSY; // Already reading
  }

  dataReady_ = false;
  lastError_ = HAL_OK; // Clear previous error
  readState_ = ReadState::ReadingShuntVoltage;
  HAL_StatusTypeDef status = processNextRead_();

  if (status != HAL_OK) {
    lastError_ = status;
    readState_ = ReadState::Idle;
#if INA228_DEBUG_ENABLED
    printf("INA228: Failed to start measurement read (status=%d)\n", status);
#endif
  }
  return status;
}

// ============================================================================
// Blocking Configuration API
// ============================================================================

HAL_StatusTypeDef INA228::reset() {
  // Set bit 15 of Config register to perform reset (0x8000)
  uint8_t data[2] = {0x80, 0x00}; // MSB, LSB of 0x8000

  HAL_StatusTypeDef status =
      HAL_I2C_Mem_Write(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::Config), I2C_MEMADD_SIZE_8BIT, data, sizeof(data), CONFIG_TIMEOUT_MS);
  if (status == HAL_OK) {
    HAL_Delay(10); // Wait for reset to complete
#if INA228_DEBUG_ENABLED
    printf("INA228: Reset successful\n");
#endif
  } else {
    lastError_ = status;
#if INA228_DEBUG_ENABLED
    printf("INA228: Reset failed (status=%d)\n", status);
#endif
  }

  return status;
}

HAL_StatusTypeDef INA228::setADCConfig(OperatingMode mode, ConversionTime busConvTime, ConversionTime shuntConvTime, ConversionTime tempConvTime,
                                       Averaging avg) {
  uint16_t adcConfig = (static_cast<uint16_t>(mode) << 12) | (static_cast<uint16_t>(busConvTime) << 9) | (static_cast<uint16_t>(shuntConvTime) << 6) |
                       (static_cast<uint16_t>(tempConvTime) << 3) | (static_cast<uint16_t>(avg) << 0);

  uint8_t data[2];
  data[0] = (adcConfig >> 8) & 0xFF; // MSB
  data[1] = adcConfig & 0xFF;        // LSB

  HAL_StatusTypeDef status =
      HAL_I2C_Mem_Write(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::ADCConfig), I2C_MEMADD_SIZE_8BIT, data, sizeof(data), CONFIG_TIMEOUT_MS);
  if (status != HAL_OK) {
    lastError_ = status;
#if INA228_DEBUG_ENABLED
    printf("INA228: setADCConfig failed (status=%d)\n", status);
#endif
  }
  return status;
}

HAL_StatusTypeDef INA228::setConfig(ADCRange adcrange) {
  uint16_t config = (static_cast<uint16_t>(adcrange) << 4);
  // Set shunt voltage LSB based on ADCRANGE
  shuntVoltageLSB_ = (adcrange == ADCRange::Range0) ? 312.5e-9f : 78.125e-9f;

  uint8_t data[2];
  data[0] = (config >> 8) & 0xFF; // MSB
  data[1] = config & 0xFF;        // LSB

  HAL_StatusTypeDef status =
      HAL_I2C_Mem_Write(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::Config), I2C_MEMADD_SIZE_8BIT, data, sizeof(data), CONFIG_TIMEOUT_MS);
  if (status != HAL_OK) {
    lastError_ = status;
#if INA228_DEBUG_ENABLED
    printf("INA228: setConfig failed (status=%d)\n", status);
#endif
  }
  return status;
}

HAL_StatusTypeDef INA228::setShuntCalibration(float rShunt, float maxExpectedCurrent) {
  currentLSB_ = maxExpectedCurrent / 524288.0f; // 2^19
  powerLSB_ = 3.2f * currentLSB_;

  float cal = 13107.2e6f * currentLSB_ * rShunt; // From datasheet
  cal *= (shuntVoltageLSB_ == 312.5e-9f) ? 1 : 4;
  uint16_t shuntCal = static_cast<uint16_t>(cal + 0.5f); // Round to nearest integer

  uint8_t data[2];
  data[0] = (shuntCal >> 8) & 0xFF; // MSB
  data[1] = shuntCal & 0xFF;        // LSB

  HAL_StatusTypeDef status =
      HAL_I2C_Mem_Write(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::ShuntCal), I2C_MEMADD_SIZE_8BIT, data, sizeof(data), CONFIG_TIMEOUT_MS);
  if (status != HAL_OK) {
    lastError_ = status;
#if INA228_DEBUG_ENABLED
    printf("INA228: setShuntCalibration failed (status=%d)\n", status);
#endif
  }
#if INA228_DEBUG_ENABLED
  else {
    printf("INA228: ShuntCal=0x%04X, CurrentLSB=%.9f A/bit, PowerLSB=%.6f W/bit\n", shuntCal, currentLSB_, powerLSB_);
  }
#endif

  return status;
}

// ============================================================================
// Private State Machine Implementation
// ============================================================================

HAL_StatusTypeDef INA228::processNextRead_() {
  HAL_StatusTypeDef status = HAL_OK;

  switch (readState_) {
  case ReadState::ReadingShuntVoltage: {
    status = HAL_I2C_Mem_Read_IT(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::ShuntVoltage), I2C_MEMADD_SIZE_8BIT, rxBuffer_,
                                 RegTraits<Register::ShuntVoltage>::size);
    break;
  }
  case ReadState::ReadingBusVoltage: {
    status = HAL_I2C_Mem_Read_IT(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::BusVoltage), I2C_MEMADD_SIZE_8BIT, rxBuffer_,
                                 RegTraits<Register::BusVoltage>::size);
    break;
  }
  case ReadState::ReadingCurrent: {
    status = HAL_I2C_Mem_Read_IT(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::Current), I2C_MEMADD_SIZE_8BIT, rxBuffer_,
                                 RegTraits<Register::Current>::size);
    break;
  }
  case ReadState::ReadingPower: {
    status = HAL_I2C_Mem_Read_IT(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::Power), I2C_MEMADD_SIZE_8BIT, rxBuffer_,
                                 RegTraits<Register::Power>::size);
    break;
  }
  case ReadState::ReadingTemperature: {
    status = HAL_I2C_Mem_Read_IT(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::Temperature), I2C_MEMADD_SIZE_8BIT, rxBuffer_,
                                 RegTraits<Register::Temperature>::size);
    break;
  }
  default:
    break;
  }

  if (status != HAL_OK) {
    readState_ = ReadState::Idle;
  }

  return status;
}

// ============================================================================
// I2C Callback Handler
// ============================================================================

void INA228::onI2CMemRxComplete(I2C_HandleTypeDef *hi2c) {
  if (hi2c != hi2c_) {
    return;
  }

  // Check for I2C errors
  if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
    lastError_ = HAL_ERROR;
    readState_ = ReadState::Idle;
    dataReady_ = false;
#if INA228_DEBUG_ENABLED
    printf("INA228: I2C error during read (state=%d)\n", static_cast<uint8_t>(readState_));
#endif
    return;
  }

  switch (readState_) {
  case ReadState::ReadingShuntVoltage: {
    int32_t shuntVoltageRaw = bytesToValue<Register::ShuntVoltage>(rxBuffer_);
    shuntVoltage_ = rawToShuntVoltage(shuntVoltageRaw);
    readState_ = ReadState::ReadingBusVoltage;
    processNextRead_();
    break;
  }
  case ReadState::ReadingBusVoltage: {
    int32_t busVoltageRaw = bytesToValue<Register::BusVoltage>(rxBuffer_);
    busVoltage_ = rawToBusVoltage(busVoltageRaw);
    readState_ = ReadState::ReadingCurrent;
    processNextRead_();
    break;
  }
  case ReadState::ReadingCurrent: {
    int32_t currentRaw = bytesToValue<Register::Current>(rxBuffer_);
    current_ = rawToCurrent(currentRaw);
    readState_ = ReadState::ReadingPower;
    processNextRead_();
    break;
  }
  case ReadState::ReadingPower: {
    uint32_t powerRaw = bytesToValue<Register::Power>(rxBuffer_);
    power_ = rawToPower(powerRaw);
    readState_ = ReadState::ReadingTemperature;
    processNextRead_();
    break;
  }
  case ReadState::ReadingTemperature: {
    int16_t temperatureRaw = bytesToValue<Register::Temperature>(rxBuffer_);
    temperature_ = rawToTemperature(temperatureRaw);
    readState_ = ReadState::Complete;
    dataReady_ = true;
#if INA228_DEBUG_ENABLED
    printf("INA228: Measurement complete - Vshunt=%.6f, Vbus=%.2f, I=%.3f, P=%.3f, Tdie=%.1f\n", shuntVoltage_, busVoltage_, current_, power_,
           temperature_);
#endif
    break;
  }
  default:
    readState_ = ReadState::Idle;
    break;
  }
}

// ============================================================================
// Conversion Functions
// ============================================================================

float INA228::rawToBusVoltage(int32_t raw) {
  // Extract 20-bit signed value from bits 23-4 (lower 4 bits are reserved)
  // Bus voltage LSB = 195.3125 µV
  return (raw >> 4) * 195.3125e-6f;
}

float INA228::rawToShuntVoltage(int32_t raw) const {
  // Extract 20-bit signed value from bits 23-4 (lower 4 bits are reserved)
  return (raw >> 4) * shuntVoltageLSB_;
}

float INA228::rawToCurrent(int32_t raw) const {
  // Extract 20-bit signed value from bits 23-4 (lower 4 bits are reserved)
  return (raw >> 4) * currentLSB_;
}

float INA228::rawToPower(uint32_t raw) const { return raw * powerLSB_; }

float INA228::rawToTemperature(int16_t raw) {
  // Temperature LSB = 0.125 °C
  return raw * 0.125f;
}

HAL_StatusTypeDef INA228::verifyDevicePresent() {
  uint8_t mfgID[2];
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(hi2c_, i2cAddr_, static_cast<uint8_t>(Register::ManufacturerID), I2C_MEMADD_SIZE_8BIT, mfgID,
                                              sizeof(mfgID), CONFIG_TIMEOUT_MS);

  if (status == HAL_OK) {
    uint16_t id = (static_cast<uint16_t>(mfgID[0]) << 8) | mfgID[1];
    if (id == 0x5449) {
#if INA228_DEBUG_ENABLED
      printf("INA228: Device verified - MfgID=0x%04X\n", id);
#endif
      return HAL_OK;
    } else {
#if INA228_DEBUG_ENABLED
      printf("INA228: Invalid MfgID=0x%04X (expected 0x5449)\n", id);
#endif
      lastError_ = HAL_ERROR;
      return HAL_ERROR;
    }
  } else {
    lastError_ = status;
#if INA228_DEBUG_ENABLED
    printf("INA228: Failed to read MfgID (status=%d)\n", status);
#endif
    return status;
  }
}

// ============================================================================
// Global Interrupt Handler Integration
// ============================================================================

/**
 * @brief Called from main.cpp in HAL_I2C_MemRxCpltCallback
 * This routes the interrupt to the INA228 instance.
 *
 * In main.cpp, add this call in HAL_I2C_MemRxCpltCallback:
 *     extern INA228 *g_ina228Instance;
 *     void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
 *       if (g_ina228Instance) {
 *         g_ina228Instance->onI2CMemRxComplete(hi2c);
 *       }
 *     }
 */
void ina228_i2c_callback_router(I2C_HandleTypeDef *hi2c) {
  if (g_ina228Instance) {
    g_ina228Instance->onI2CMemRxComplete(hi2c);
  }
}
