/**
 * @file orion2_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Functions to help with CAN Bus communications with the Orion BMS 2
 * @version 0.1
 * @date 2026-03-16
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "orion2_can_comms.hpp"

Orion2::Orion2(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr) : hfdcan_(hfdcan), baseAddr_(baseAddr) {}

void Orion2::measurementParser(OrionMessageID id, uint8_t *rxData) {
  switch (id) {}
}