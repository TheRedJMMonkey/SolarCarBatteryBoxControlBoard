/**
 * @file ws_can_comms.cpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Implementation of WaveSculptor CAN communications class based on
 * Prohelion's documentation at
 * https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html
 * @version 0.1
 * @date 2026-03-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ws_can_comms.hpp"
#include "main.h"
#include <cstring>

#if WS_DEBUG_ENABLED
/**
 * @brief Print a 16-bit value in binary format
 * @param value The value to print
 */
static void printBinary16(uint16_t value) {
  for (int i = 15; i >= 0; i--) {
    printf("%d", (value >> i) & 1);
  }
}
#endif

WaveSculptor::WaveSculptor(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr, uint16_t dcuBaseAddr)
    : hfdcan_(hfdcan), baseAddr_(baseAddr), dcuBaseAddr_(dcuBaseAddr) {}

/************************ Drive Control Commands *************************/
void WaveSculptor::sendMotorDrive(float motorCurrent, float motorRPM) {
#if WS_DEBUG_ENABLED
  printf("WS: Sending Motor Drive - Current: %.2f %%, RPM: %.2f\n", motorCurrent, motorRPM);
#endif
  FDCAN_TxHeaderTypeDef txHeader = dcuBaseTxHeader(WaveSculptorDCUMessageID::MotorDrive);

  uint8_t txData[8] = {0};
  memcpy(&txData[4], &motorCurrent, sizeof(motorCurrent));
  memcpy(&txData[0], &motorRPM, sizeof(motorRPM));

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::sendMotorPower(float busCurrent) {
#if WS_DEBUG_ENABLED
  printf("WS: Sending Motor Power - Bus Current: %.2f %%\n", busCurrent);
#endif
  FDCAN_TxHeaderTypeDef txHeader = dcuBaseTxHeader(WaveSculptorDCUMessageID::MotorPower);

  uint8_t txData[8] = {0};
  memcpy(&txData[4], &busCurrent, sizeof(busCurrent));

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::sendReset() {
#if WS_DEBUG_ENABLED
  printf("WS: Sending Reset Command\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = dcuBaseTxHeader(WaveSculptorDCUMessageID::Reset);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/***************** Motor Controller Measurement Commands *****************/
void WaveSculptor::requestIdentificationInformation() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Identification Information\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::Identification);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestStatusInformation() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Status Information\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::Status);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestBusMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Bus Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::BusMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestVelocityMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Velocity Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::VelocityMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestPhaseCurrentMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Phase Current Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::PhaseCurrentMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestMotorVoltageVectorMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Motor Voltage Vector Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::MotorVoltageVectorMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestMotorCurrentVectorMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Current Vector Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::MotorCurrentVectorMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestMotorBackEMFMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Back-EMF Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::MotorBackEMFMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::request15VRailMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting 15V Rail Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::VoltageRail15VMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::request3V3_1V9RailMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting 3.3V & 1.9V Rail Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::VoltageRail3V3_1V9Measurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestHeatSinkMotorTempMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Heat Sink & Motor Temperature Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::HeatSinkMotorTempMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestDSPTempMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting DSP Temp Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::DSPTempMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestOdometerBusAhMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Odometer & Bus Ah Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::OdometerBusAhMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::requestSlipSpeedMeasurement() {
#if WS_DEBUG_ENABLED
  printf("WS: Requesting Slip Speed Measurement\n");
#endif
  FDCAN_TxHeaderTypeDef txHeader = requestBaseTxHeader(WaveSculptorMessageID::SlipSpeedMeasurement);

  uint8_t txData[0] = {};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan_, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void WaveSculptor::measurementParser(WaveSculptorMessageID id, uint8_t *rxData) {
  switch (id) {
  // Identification Information
  case WaveSculptorMessageID::Identification:
    memcpy(&serialNumber_, &rxData[4], sizeof(serialNumber_));
    memcpy(&deviceID_, &rxData[0], sizeof(deviceID_));
#if WS_DEBUG_ENABLED
    printf("WS: Received Identification - Serial: %u, DeviceID: %x\n", serialNumber_, deviceID_);
#endif
    break;

  // Status Information
  case WaveSculptorMessageID::Status:
    memcpy(&receiveErrorCount_, &rxData[7], sizeof(receiveErrorCount_));
    memcpy(&transmitErrorCount_, &rxData[6], sizeof(transmitErrorCount_));
    memcpy(&activeMotor_, &rxData[4], sizeof(activeMotor_));
    memcpy(&errorFlags_, &rxData[2], sizeof(errorFlags_));
    memcpy(&limitFlags_, &rxData[0], sizeof(limitFlags_));
#if WS_DEBUG_ENABLED
    printf("WS: Received Status - RxErr: %u, TxErr: %u, ActiveMotor: %u, ErrFlags: 0b", receiveErrorCount_, transmitErrorCount_, activeMotor_);
    printBinary16(errorFlags_);
    printf(", LimitFlags: 0b");
    printBinary16(limitFlags_);
    printf("\n");
#endif
    break;

  // Bus Measurement
  case WaveSculptorMessageID::BusMeasurement:
    memcpy(&busCurrent_, &rxData[4], sizeof(busCurrent_)); // A
    memcpy(&busVoltage_, &rxData[0], sizeof(busVoltage_)); // V
#if WS_DEBUG_ENABLED
    printf("WS: Received Bus Measurement - Current: %.2f A, Voltage: %.2f V\n", busCurrent_, busVoltage_);
#endif
    break;

  // Velocity Measurement
  case WaveSculptorMessageID::VelocityMeasurement:
    memcpy(&vehicleVelocity_, &rxData[4], sizeof(vehicleVelocity_)); // m/s
    memcpy(&motorVelocity_, &rxData[0], sizeof(motorVelocity_));     // rpm
#if WS_DEBUG_ENABLED
    printf("WS: Received Velocity - Vehicle: %.2f m/s, Motor: %.2f rpm\n", vehicleVelocity_, motorVelocity_);
#endif
    break;

  // Phase Current Measurement
  case WaveSculptorMessageID::PhaseCurrentMeasurement:
    memcpy(&phaseCCurrent_, &rxData[4], sizeof(phaseCCurrent_)); // A_rms
    memcpy(&phaseBCurrent_, &rxData[0], sizeof(phaseBCurrent_)); // A_rms
#if WS_DEBUG_ENABLED
    printf("WS: Received Phase Current - PhaseC: %.2f A_rms, PhaseB: %.2f A_rms\n", phaseCCurrent_, phaseBCurrent_);
#endif
    break;

  // Motor Voltage Vector Measurement
  case WaveSculptorMessageID::MotorVoltageVectorMeasurement:
    memcpy(&dVoltage_, &rxData[4], sizeof(dVoltage_)); // V
    memcpy(&qVoltage_, &rxData[0], sizeof(qVoltage_)); // V
#if WS_DEBUG_ENABLED
    printf("WS: Received Motor Voltage Vector - D: %.2f V, Q: %.2f V\n", dVoltage_, qVoltage_);
#endif
    break;

  // Motor Current Vector Measurement
  case WaveSculptorMessageID::MotorCurrentVectorMeasurement:
    memcpy(&dCurrent_, &rxData[4], sizeof(dCurrent_)); // A
    memcpy(&qCurrent_, &rxData[0], sizeof(qCurrent_)); // A
#if WS_DEBUG_ENABLED
    printf("WS: Received Motor Current Vector - D: %.2f A, Q: %.2f A\n", dCurrent_, qCurrent_);
#endif
    break;

  // Motor BackEMF Measurement/Prediction
  case WaveSculptorMessageID::MotorBackEMFMeasurement:
    memcpy(&dBackEMF_, &rxData[4],
           sizeof(dBackEMF_));                         // V - Always 0V by definition
    memcpy(&qBackEMF_, &rxData[0], sizeof(qBackEMF_)); // V
#if WS_DEBUG_ENABLED
    printf("WS: Received Motor BackEMF - D: %.2f V, Q: %.2f V\n", dBackEMF_, qBackEMF_);
#endif
    break;

  // 15V Voltage Rail Measurement
  case WaveSculptorMessageID::VoltageRail15VMeasurement:
    memcpy(&measured15VSupply_, &rxData[4], sizeof(measured15VSupply_)); // V
#if WS_DEBUG_ENABLED
    printf("WS: Received 15V Rail - %.2f V\n", measured15VSupply_);
#endif
    break;

  // 3.3V & 1.9V Voltage Rail Measurement
  case WaveSculptorMessageID::VoltageRail3V3_1V9Measurement:
    memcpy(&measured3V3Supply_, &rxData[4], sizeof(measured3V3Supply_)); // V
    memcpy(&measured1V9Supply_, &rxData[0], sizeof(measured1V9Supply_)); // V
#if WS_DEBUG_ENABLED
    printf("WS: Received 3.3V/1.9V Rails - 3.3V: %.2f V, 1.9V: %.2f V\n", measured3V3Supply_, measured1V9Supply_);
#endif
    break;

  // Heat-sink & Motor Temperature Measurement
  case WaveSculptorMessageID::HeatSinkMotorTempMeasurement:
    memcpy(&heatsinkTemp_, &rxData[4], sizeof(heatsinkTemp_)); // °C
    memcpy(&motorTemp_, &rxData[0], sizeof(motorTemp_));       // °C
#if WS_DEBUG_ENABLED
    printf("WS: Received Temperatures - Heatsink: %.2f °C, Motor: %.2f °C\n", heatsinkTemp_, motorTemp_);
#endif
    break;

  // DSP Board Temperature Measurement
  case WaveSculptorMessageID::DSPTempMeasurement:
    memcpy(&dspBoardTemp_, &rxData[0], sizeof(dspBoardTemp_)); // °C
#if WS_DEBUG_ENABLED
    printf("WS: Received DSP Temp - %.2f °C\n", dspBoardTemp_);
#endif
    break;

  // Odometer & Bus AmpHours Measurement
  case WaveSculptorMessageID::OdometerBusAhMeasurement:
    memcpy(&dcBusAh_, &rxData[4], sizeof(dcBusAh_));   // Ah
    memcpy(&odometer_, &rxData[0], sizeof(odometer_)); // m
#if WS_DEBUG_ENABLED
    printf("WS: Received Odometer/Ah - Ah: %.2f, Odometer: %.2f m\n", dcBusAh_, odometer_);
#endif
    break;

  // Slip Speed Measurement
  case WaveSculptorMessageID::SlipSpeedMeasurement:
    memcpy(&slipSpeed_, &rxData[4], sizeof(slipSpeed_)); // Hz
#if WS_DEBUG_ENABLED
    printf("WS: Received Slip Speed - %.2f Hz\n", slipSpeed_);
#endif
    break;
  }
}
