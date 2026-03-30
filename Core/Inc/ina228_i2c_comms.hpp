/**
 * @file ina228_i2c_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for INA228 Asynchronous I2C communications class
 * @version 0.1
 * @date 2026-03-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef INA228_I2C_COMMS_HPP
#define INA228_I2C_COMMS_HPP

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_i2c.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define INA228_DEBUG_ENABLED true

/**
 * @brief Class for INA228 Power/Energy Monitor I2C Communications (Asynchronous)
 *
 * Uses interrupt-based I2C reads with a state machine to minimize blocking time
 * and optimize for constrained microcontroller resources.
 */
class INA228 {
public:
  // Default I2C address with 7-bit addressing (A0 and A1 pins connected to GND)
  static constexpr uint8_t DEFAULT_ADDR = 0x40 << 1;

  // Configuration timeout (milliseconds) - prevents indefinite hangs
  static constexpr uint32_t CONFIG_TIMEOUT_MS = 100;

  /**
   * @brief Register Addresses
   */
  enum class Register : uint8_t {
    Config = 0x00,
    ADCConfig = 0x01,
    ShuntVoltage = 0x02,
    BusVoltage = 0x03,
    Temperature = 0x04,
    DiagAlert = 0x05,
    Current = 0x06,
    Power = 0x07,
    Energy = 0x08,
    Charge = 0x09,
    ShuntCal = 0x10,
    ManufacturerID = 0x3E,
    DeviceID = 0x3F
  };

  static constexpr uint8_t regSize(INA228::Register r) {
    switch (r) {
    case INA228::Register::ShuntVoltage:
    case INA228::Register::BusVoltage:
    case INA228::Register::Current:
    case INA228::Register::Power:
      return 3; // 24-bit

    case INA228::Register::Energy:
    case INA228::Register::Charge:
      return 5; // 40-bit

    default:
      return 2; // 16-bit
    }
  }

  /**
   * @brief Operating modes (ADC_CONFIG bits 15-12)
   */
  enum class OperatingMode : uint8_t {
    Shutdown = 0x0,
    BusTrig = 0x1,
    ShuntTrig = 0x2,
    ShuntBusTrig = 0x3,
    TempTrig = 0x4,
    TempBusTrig = 0x5,
    TempShuntTrig = 0x6,
    TempShuntBusTrig = 0x7,
    BusCont = 0x9,
    ShuntCont = 0xA,
    ShuntBusCont = 0xB,
    TempCont = 0xC,
    TempBusCont = 0xD,
    TempShuntCont = 0xE,
    TempShuntBusCont = 0xF
  };

  /**
   * @brief Conversion times (ADC_CONFIG bits 11-9, 8-6, 5-3)
   */
  enum class ConversionTime : uint8_t { us50 = 0x0, us84 = 0x1, us150 = 0x2, us280 = 0x3, us540 = 0x4, us1052 = 0x5, us2074 = 0x6, us4120 = 0x7 };

  /**
   * @brief Averaging modes (ADC_CONFIG bits 2-0)
   */
  enum class Averaging : uint8_t { Avg1 = 0x0, Avg4 = 0x1, Avg16 = 0x2, Avg64 = 0x3, Avg128 = 0x4, Avg256 = 0x5, Avg512 = 0x6, Avg1024 = 0x7 };

  /**
   * @brief ADC Range (CONFIG bit 4)
   */
  enum class ADCRange : uint8_t {
    Range0 = 0x0, // ±163.84 mV
    Range1 = 0x1  // ±40.96 mV
  };

  /**
   * @brief Construct a new INA228 object
   *
   * @param hi2c Pointer to I2C handle
   * @param i2cAddr 7-bit I2C address (default 0x40 << 1). Must already be shifted left 1 bit
   */
  INA228(I2C_HandleTypeDef *hi2c, uint8_t i2cAddr = DEFAULT_ADDR);

  /**
   * @brief Verify INA228 device is present and responding
   * Reads ManufacturerID register to check device is on I2C bus
   * @return HAL_StatusTypeDef - HAL_OK if device found with correct ID
   */
  HAL_StatusTypeDef verifyDevicePresent();

  /**
   * @brief Initiate non-blocking read of all measurement registers
   * @return HAL_StatusTypeDef - HAL_OK if read started, error otherwise
   */
  HAL_StatusTypeDef startReadAllMeasurements();

  /**
   * @brief Check if measurement data is ready
   * @return true if all measurements have been read successfully
   */
  bool isDataReady() const { return dataReady_; }

  /**
   * @brief Reset the INA228 to default configuration (blocking)
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef reset();

  /**
   * @brief Configure the INA228 ADC_CONFIG register using helper constants (blocking)
   * @param mode Operating mode
   * @param busConvTime Conversion time base value
   * @param shuntConvTime Conversion time base value
   * @param tempConvTime Conversion time base value
   * @param avg Averaging mode
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef setADCConfig(INA228::OperatingMode mode = INA228::OperatingMode::TempShuntBusCont,
                                 INA228::ConversionTime busConvTime = INA228::ConversionTime::us1052,
                                 INA228::ConversionTime shuntConvTime = INA228::ConversionTime::us1052,
                                 INA228::ConversionTime tempConvTime = INA228::ConversionTime::us1052,
                                 INA228::Averaging avg = INA228::Averaging::Avg1);

  /**
   * @brief Configure the INA228 CONFIG register using helper constants (blocking)
   * @param adcrange ADC range
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef setConfig(INA228::ADCRange adcrange = INA228::ADCRange::Range0);

  /**
   * @brief Calculate and write the shunt calibration value (blocking)
   *
   * Formula from datasheet (INA228):
   * current_lsb = maxExpectedCurrent / 524288
   * shunt_cal = round(13107200000 * current_lsb * rShunt)
   *
   * @param rShunt Shunt resistance in ohms
   * @param maxExpectedCurrent Maximum expected current in amps
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef setShuntCalibration(float rShunt, float maxExpectedCurrent);

  float getBusVoltage() const { return busVoltage_; }
  float getShuntVoltage() const { return shuntVoltage_; }
  float getCurrent() const { return current_; }
  float getPower() const { return power_; }
  float getTemperature() const { return temperature_; }
  uint8_t getI2CAddr() const { return i2cAddr_; }
  HAL_StatusTypeDef getLastError() const { return lastError_; }

  void setDataReady(bool dataReady) { dataReady_ = dataReady; }

  /**
   * @brief I2C callback handler - call from HAL_I2C_MemRxCpltCallback
   *
   * @param hi2c
   */
  void onI2CMemRxComplete(I2C_HandleTypeDef *hi2c);

private:
  // State machine for async reads
  enum class ReadState : uint8_t {
    Idle = 0,
    ReadingShuntVoltage = 1,
    ReadingBusVoltage = 2,
    ReadingCurrent = 3,
    ReadingPower = 4,
    ReadingTemperature = 5,
    Complete = 6
  };

  I2C_HandleTypeDef *hi2c_;
  uint8_t i2cAddr_;

  // Read state machine
  ReadState readState_ = ReadState::Idle;
  uint8_t rxBuffer_[5] = {}; // Holds up to 5 bytes for register reads
  bool dataReady_ = false;

  // Measurement data (stored as floats for ease of use)
  float busVoltage_ = 0.0f;   // Bus voltage in V
  float shuntVoltage_ = 0.0f; // Shunt voltage in V
  float current_ = 0.0f;      // Current in A
  float power_ = 0.0f;        // Power in W
  float temperature_ = 0.0f;  // Temperature in °C

  // Calibration factors (computed once during setup)
  float currentLSB_ = 0.0f;           // Current LSB in A/bit
  float powerLSB_ = 0.0f;             // Power LSB in W/bit
  float shuntVoltageLSB_ = 312.5e-9f; // Shunt voltage LSB in V/bit

  HAL_StatusTypeDef lastError_ = HAL_OK; // Track last I2C error

  /**
   * @brief Internal: Process next read in state machine
   *
   * @return HAL_StatusTypeDef
   */
  HAL_StatusTypeDef processNextRead_();

  /**
   * @brief Struct template to store register traits
   *
   * @tparam R Register address
   */
  template <INA228::Register R> struct RegTraits;

  /**
   * @brief Extend the sign bit of a smaller integer inside a uint64_t to an int64_t
   *
   * @tparam Bits The number of bits in the contained integer
   * @param x Value to extend
   * @return int64_t
   */
  template <unsigned Bits> static int64_t signExtend(uint64_t x);

  /**
   * @brief Convert raw bytes to signed/unsigned value based on register traits
   *
   * @tparam R Register address
   * @param data Pointer to store the read value
   * @return RegTraits<R>::RegType
   */
  template <INA228::Register R> typename RegTraits<R>::RegType bytesToValue(const uint8_t *data);

  /**
   * @brief Convert raw register value to bus voltage in volts
   * @param raw Raw register value
   * @return Bus voltage in V
   */
  static float rawToBusVoltage(int32_t raw);

  /**
   * @brief Convert raw register value to shunt voltage in volts
   * @param raw Raw register value
   * @return Shunt voltage in V
   */
  float rawToShuntVoltage(int32_t raw) const;

  /**
   * @brief Convert raw register value to current in amperes
   * @param raw Raw register value
   * @return Current in A
   */
  float rawToCurrent(int32_t raw) const;

  /**
   * @brief Convert raw register value to power in watts
   * @param raw Raw register value
   * @return Power in W
   */
  float rawToPower(uint32_t raw) const;

  /**
   * @brief Convert raw register value to temperature in celsius
   * @param raw Raw register value
   * @return Temperature in °C
   */
  static float rawToTemperature(int16_t raw);
};

template <INA228::Register R> struct INA228::RegTraits {
  static constexpr uint8_t size = 2;
  static constexpr bool isSigned = false; // default: unsigned
  using RegType = uint16_t;
};

// 24‑bit signed registers
template <> struct INA228::RegTraits<INA228::Register::ShuntVoltage> {
  static constexpr uint8_t size = 3;
  static constexpr bool isSigned = true;
  using RegType = int32_t;
};
template <> struct INA228::RegTraits<INA228::Register::BusVoltage> {
  static constexpr uint8_t size = 3;
  static constexpr bool isSigned = true;
  using RegType = int32_t;
};
template <> struct INA228::RegTraits<INA228::Register::Current> {
  static constexpr uint8_t size = 3;
  static constexpr bool isSigned = true;
  using RegType = int32_t;
};

// 24-bit unsigned registers
template <> struct INA228::RegTraits<INA228::Register::Power> {
  static constexpr uint8_t size = 3;
  static constexpr bool isSigned = false;
  using RegType = uint32_t;
};

// Temperature (16‑bit signed)
template <> struct INA228::RegTraits<INA228::Register::Temperature> {
  static constexpr uint8_t size = 2;
  static constexpr bool isSigned = true;
  using RegType = int16_t;
};

template <unsigned Bits> int64_t INA228::signExtend(uint64_t x) {
  constexpr uint64_t shift = 64 - Bits;

  // Right-shift on signed integral types is always arithemetic shift since C++20. Implementation defined behavior before C++20.
  return static_cast<int64_t>(x << shift) >> shift;
}

template <INA228::Register R> typename INA228::RegTraits<R>::RegType INA228::bytesToValue(const uint8_t *data) {
  constexpr uint8_t size = RegTraits<R>::size;
  uint64_t raw = 0;
  for (uint8_t i = 0; i < size; i++)
    raw = (raw << 8) | data[i];

  if constexpr (RegTraits<R>::isSigned) {
    int64_t signedValue = signExtend<RegTraits<R>::size * 8>(raw);
    return static_cast<typename RegTraits<R>::RegType>(signedValue);
  } else {
    return static_cast<typename RegTraits<R>::RegType>(raw);
  }
}

#endif /* INA228_I2C_COMMS_HPP */
