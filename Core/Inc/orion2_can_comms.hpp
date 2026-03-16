/**
 * @file orion2_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for Orion2 CAN communications class
 * @version 0.1
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ORION2_CAN_COMMS_HPP
#define ORION2_CAN_COMMS_HPP

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define ORION_DEBUG_ENABLED true

static const uint16_t DEFAULT_ORION_BASE_ADDR = 0x240;

/**
 * @brief Enum for Orion Message IDs (offsets from base address)
 */
enum class OrionMessageID : uint16_t { a = 0x00 };

class Orion2 {
private:
  FDCAN_HandleTypeDef *hfdcan_;

  // Orion base address
  uint16_t baseAddr_;

  // Measurement data

public:
  /**
   * @brief Construct a new Orion object
   *
   * @param hfdcan Pointer to FDCAN handle
   * @param baseAddr Base address for Orion messages (Default 0x240)
   */
  Orion2(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr = DEFAULT_ORION_BASE_ADDR);

  /**
   * @brief Parse measurement data
   *
   * @param id CAN message ID offset as enum
   * @param rxData Received data
   */
  void measurementParser(OrionMessageID id, uint8_t *rxData);

  uint16_t getBaseAddr() const { return baseAddr_; }

  /**
   * @brief Check if a CAN ID is within the Orion message range
   *
   * @param id CAN message ID
   * @return true if ID is within Orion range
   */
  bool isOrionMessage(uint32_t id) const { return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(OrionMessageID::a); }
};

#endif /* ORION2_CAN_COMMS_HPP */
