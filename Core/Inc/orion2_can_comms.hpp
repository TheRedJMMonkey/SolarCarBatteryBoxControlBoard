/**
 * @file orion2_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for Orion2 BMS CAN communications class
 * @version 0.1
 * @date 2026-03-31
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef ORION2_CAN_COMMS_HPP
#define ORION2_CAN_COMMS_HPP

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define ORION2_DEBUG_ENABLED true

/**
 * @brief Class for Orion2 BMS CAN Communications
 *
 * Provides synchronous parsing of received CAN messages with state storage.
 * Measurements are updated when new CAN data is received via parseMeasurement().
 */
class Orion2 {
public:
  static constexpr uint16_t DEFAULT_BASE_ADDR = 0x240;

  /**
   * @brief Enum for Orion2 Message IDs (offsets from base address)
   */
  enum class MessageID : uint16_t { Status = 0x00 };

  /**
   * @brief Construct a new Orion2 object
   *
   * @param hfdcan Pointer to FDCAN handle
   * @param baseAddr Base address for Orion2 messages (default 0x240)
   */
  Orion2(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr = DEFAULT_BASE_ADDR);

  /**
   * @brief Parse received CAN measurement data and update internal state
   *
   * @param id CAN message ID offset as enum
   * @param rxData Pointer to received 8-byte CAN data
   * @return HAL_StatusTypeDef - HAL_OK on successful parse
   */
  HAL_StatusTypeDef parseMeasurement(Orion2::MessageID id, const uint8_t *rxData);

  /**
   * @brief Check if a CAN ID is within the Orion2 message range
   *
   * @param id CAN message ID
   * @return true if ID is within Orion2 range
   */
  bool isOrion2Message(uint32_t id) const { return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(Orion2::MessageID::Status); }

  uint16_t getBaseAddr() const { return baseAddr_; }
  HAL_StatusTypeDef getLastError() const { return lastError_; }

private:
  [[maybe_unused]] FDCAN_HandleTypeDef *hfdcan_; // Kept for potential future use
  uint16_t baseAddr_;                            // Used for message range checking

  // Error tracking
  HAL_StatusTypeDef lastError_ = HAL_OK;

  // TODO: Add measurement data members as messages are documented
};

#endif /* ORION2_CAN_COMMS_HPP */
