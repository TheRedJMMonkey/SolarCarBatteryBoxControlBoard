/**
 * @file ws_can_comms.c
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Functions to help with CAN Bus communications with the WaveSculptor22
 * motor controller. Based on Prohelion's documentation at
 * https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html
 * @version 0.1
 * @date 2026-03-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "ws_can_comms.h"

#include "main.h"
#include <string.h>

/**
 * @brief Constructor for FDCAN txHeader structs for the WaveSculptor Driver
 * Control Unit
 *
 * @param id
 * @return FDCAN_TxHeaderTypeDef
 */
static inline FDCAN_TxHeaderTypeDef WS_DCUBaseTxHeader(uint16_t idOffset) {
  FDCAN_TxHeaderTypeDef txHeader = {.Identifier = WS_DCU_BASE_ADDR + idOffset,
                                    .IdType = FDCAN_STANDARD_ID,
                                    .TxFrameType = FDCAN_DATA_FRAME,
                                    .DataLength = FDCAN_DLC_BYTES_8,
                                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                                    .BitRateSwitch = FDCAN_BRS_OFF,
                                    .FDFormat = FDCAN_CLASSIC_CAN,
                                    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                                    .MessageMarker = 0};
  return txHeader;
}

/**
 * @brief Constructor for FDCAN txHeader structs for the WaveSculptor
 *
 * @param id
 * @return FDCAN_TxHeaderTypeDef
 */
static inline FDCAN_TxHeaderTypeDef WS_RequestBaseTxHeader(uint16_t idOffset) {
  FDCAN_TxHeaderTypeDef txHeader = {.Identifier = WS_BASE_ADDR + idOffset,
                                    .IdType = FDCAN_STANDARD_ID,
                                    .TxFrameType = FDCAN_REMOTE_FRAME,
                                    .DataLength = FDCAN_DLC_BYTES_8,
                                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                                    .BitRateSwitch = FDCAN_BRS_OFF,
                                    .FDFormat = FDCAN_CLASSIC_CAN,
                                    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                                    .MessageMarker = 0};
  return txHeader;
}

/************************ Drive Control Commands *************************/

/**
 * @brief WaveSculptor Motor Drive Command
 *
 * @param hfdcan
 * @param motorCurrent
 * @param motorRPM
 */
void WS_SendMotorDrive(FDCAN_HandleTypeDef *hfdcan, float motorCurrent,
                       float motorRPM) {
  FDCAN_TxHeaderTypeDef txHeader = WS_DCUBaseTxHeader(0x01);

  uint8_t txData[8] = {0};
  memcpy(&txData[0], &motorCurrent, sizeof(motorCurrent));
  memcpy(&txData[4], &motorRPM, sizeof(motorRPM));

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief WaveSculptor Motor Current Command
 *
 * @param hfdcan
 * @param busCurrent
 */
void WS_SendMotorPower(FDCAN_HandleTypeDef *hfdcan, float busCurrent) {
  FDCAN_TxHeaderTypeDef txHeader = WS_DCUBaseTxHeader(0x02);

  uint8_t txData[8] = {0};
  memcpy(&txData[0], &busCurrent, sizeof(busCurrent));

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief WaveSculptor Reset Command
 *
 * @param hfdcan
 */
void WS_SendReset(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_DCUBaseTxHeader(0x03);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/***************** Motor Controller Measurement Commands *****************/

/**
 * @brief Request identification information from the WaveSculptor motor
 * controller
 *
 * @param hfdcan
 */
void WS_RequestIdentificationInformation(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x00);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request status information from the WaveSculptor motor controller
 *
 * @param hfdcan
 */
void WS_RequestStatusInformation(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x01);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request bus current and voltage measurement from the WaveSculptor
 * motor controller
 *
 * @param hfdcan
 */
void WS_RequestBusMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x02);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request vehicle and motor velocity measurement from the WaveSculptor
 * motor controller
 *
 * @param hfdcan
 */
void WS_RequestVelocityMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x03);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request motor phase B and C current from the WaveSculptor motor
 * controller
 *
 * @param hfdcan
 */
void WS_RequestPhaseCurrentMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x04);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request motor voltage real and imaginary component measurement from
 * the WaveSculptor motor controller
 *
 * @param hfdcan
 */
void WS_RequestMotorVoltageVectorMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x05);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request motor current real and imaginary component measurement from
 * the WaveSculptor motor controller
 *
 * @param hfdcan
 */
void WS_RequestMotorCurrentVectorMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x06);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request motor back EMF measurement from the WaveSculptor motor
 * controller
 *
 * @param hfdcan
 */
void WS_RequestMotorBackEMFMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x07);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request 15V rail voltage measurement from the WaveSculptor motor
 * controller
 *
 * @param hfdcan
 */
void WS_Request15VRailMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x08);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request 3.3V and 1.9V rail voltage measurement from the WaveSculptor
 * motor controller
 *
 * @param hfdcan
 */
void WS_Request3V3_1V9RailMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x09);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request heat-sink and motor temperature measurement from the
 * WaveSculptor motor controller
 *
 * @param hfdcan
 */
void WS_RequestHeatSinkMotorTempMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x0B);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request DSP board temperature measurement from the WaveSculptor motor
 * controller
 *
 * @param hfdcan
 */
void WS_RequestDSPTempMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x0C);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request Odometer and DC Bus AmpHours measurement from the WaveSculptor
 * motor controller
 *
 * @param hfdcan
 */
void WS_RequestOdometerBusAhMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x0E);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief Request slip speed measurement from the WaveSculptor motor controller
 *
 * @param hfdcan
 */
void WS_RequestSlipSpeedMeasurement(FDCAN_HandleTypeDef *hfdcan) {
  FDCAN_TxHeaderTypeDef txHeader = WS_RequestBaseTxHeader(0x17);

  uint8_t txData[8] = {0};

  if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}
