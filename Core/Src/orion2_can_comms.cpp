/**
 * @file orion2_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of Orion2 BMS CAN communications class
 * @version 0.1
 * @date 2026-03-31
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "orion2_can_comms.hpp"

Orion2::Orion2(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr) : hfdcan_(hfdcan), baseAddr_(baseAddr) {}

HAL_StatusTypeDef Orion2::parseMeasurement(Orion2::MessageID id, const uint8_t *rxData) {
  if (rxData == nullptr) {
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  lastError_ = HAL_OK;

  switch (id) {
  case MessageID::Status: {
    // TODO: Implement status message parsing once schema is documented
#if ORION2_DEBUG_ENABLED
    // printf("Orion2: Status message received\n");
#endif
    break;
  }
  default:
    lastError_ = HAL_ERROR;
    return HAL_ERROR;
  }

  return HAL_OK;
}