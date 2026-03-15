/**
 * @file photon3_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of Photon3 CAN communications class based on
 * dilithium power systems' documentation at
 * https://www.dilithiumpower.com/products/photon-3#h.p_8MVrI1wk4RlD
 * @version 0.1
 * @date 2026-03-15
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "photon3_can_comms.hpp"
#include "main.h"
#include <cstring>

Photon3::Photon3(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr) : hfdcan_(hfdcan), baseAddr_(baseAddr) {}

void Photon3::measurementParser(Photon3MessageID id, uint8_t *rxData) {
  switch (id) {
  // Channel 1
  case Photon3MessageID::Channel1:
    memcpy(&ch1ArrayVoltage_, &rxData[0], sizeof(ch1ArrayVoltage_));     // 10mV / unit
    memcpy(&ch1ArrayCurrent_, &rxData[2], sizeof(ch1ArrayCurrent_));     // 1mA / unit
    memcpy(&ch1BatteryVoltage_, &rxData[4], sizeof(ch1BatteryVoltage_)); // 10mV / unit
    memcpy(&ch1UnitTemp_, &rxData[6], sizeof(ch1UnitTemp_));             // 10m°C / unit
#if PHOTON3_DEBUG_ENABLED
    printf("Photon3: Channel 1 - ArrayVolts: %.2f V, ArrayCurrent: %.2f A, BatVolts: %.2f V, Temp: %.2f °C\n", ch1ArrayVoltage_ * 0.01f,
           ch1ArrayCurrent_ * 0.001f, ch1BatteryVoltage_ * 0.01f, ch1UnitTemp_ * 0.01f);
#endif
    break;

  // Channel 2
  case Photon3MessageID::Channel2:
    memcpy(&ch2ArrayVoltage_, &rxData[0], sizeof(ch2ArrayVoltage_));     // 10mV / unit
    memcpy(&ch2ArrayCurrent_, &rxData[2], sizeof(ch2ArrayCurrent_));     // 1mA / unit
    memcpy(&ch2BatteryVoltage_, &rxData[4], sizeof(ch2BatteryVoltage_)); // 10mV / unit
    memcpy(&ch2UnitTemp_, &rxData[6], sizeof(ch2UnitTemp_));             // 10m°C / unit
#if PHOTON3_DEBUG_ENABLED
    printf("Photon3: Channel 2 - ArrayVolts: %.2f V, ArrayCurrent: %.2f A, BatVolts: %.2f V, Temp: %.2f °C\n", ch2ArrayVoltage_ * 0.01f,
           ch2ArrayCurrent_ * 0.001f, ch2BatteryVoltage_ * 0.01f, ch2UnitTemp_ * 0.01f);
#endif
    break;
  }
}
