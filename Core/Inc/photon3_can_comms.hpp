/**
 * @file photon3_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for the Photon3 CAN communications class
 * @version 0.1
 * @date 2026-03-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef PHOTON3_CAN_COMMS_HPP
#define PHOTON3_CAN_COMMS_HPP

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define PHOTON3_DEBUG_ENABLED true

static const uint16_t DEFAULT_PHOTON3_BASE_ADDR = 0x600;

/**
 * @brief Enum for Photon3 Message IDs (offsets from base address)
 */
enum class Photon3MessageID : uint16_t { Channel1 = 0x00, Channel2 = 0x01 };

class Photon3 {
private:
  FDCAN_HandleTypeDef *hfdcan_;

  // Photon3 Base Address
  uint16_t baseAddr_;

  // Measurement Data
  uint16_t ch1ArrayVoltage_ = 0;
  uint16_t ch1ArrayCurrent_ = 0;
  uint16_t ch1BatteryVoltage_ = 0;
  uint16_t ch1UnitTemp_ = 0;
  uint16_t ch2ArrayVoltage_ = 0;
  uint16_t ch2ArrayCurrent_ = 0;
  uint16_t ch2BatteryVoltage_ = 0;
  uint16_t ch2UnitTemp_ = 0;

public:
  /**
   * @brief Construct a new Photon 3 object
   *
   * @param hfdcan Pointer to FDCAN Handle
   * @param baseAddr Base address for Photon 3 messages (default 0x600)
   */
  Photon3(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr = DEFAULT_PHOTON3_BASE_ADDR);

  /**
   * @brief Parse measurement data
   *
   * @param id CAN message ID offset as enum
   * @param rxData Received data
   */
  void measurementParser(Photon3MessageID id, uint8_t *rxData);

  uint16_t getCh1ArrayVoltage() const { return ch1ArrayVoltage_; }
  uint16_t getCh1ArrayCurrent() const { return ch1ArrayCurrent_; }
  uint16_t getCh1BatteryVoltage() const { return ch1BatteryVoltage_; }
  uint16_t getCh1UnitTemp() const { return ch1UnitTemp_; }
  uint16_t getCh2ArrayVoltage() const { return ch2ArrayVoltage_; }
  uint16_t getCh2ArrayCurrent() const { return ch2ArrayCurrent_; }
  uint16_t getCh2BatteryVoltage() const { return ch2BatteryVoltage_; }
  uint16_t getCh2UnitTemp() const { return ch2UnitTemp_; }

  uint16_t getBaseAddr() const { return baseAddr_; }

  /**
   * @brief Check if a CAN ID is within the Photon 3 message range
   *
   * @param id CAN message ID
   * @return true if ID is within Photon 3 range
   */
  bool isPhoton3Message(uint32_t id) const { return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(Photon3MessageID::Channel2); }
};

#endif /* PHOTON3_CAN_COMMS_HPP */
