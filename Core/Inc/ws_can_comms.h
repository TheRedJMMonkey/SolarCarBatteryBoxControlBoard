/**
 * @file ws_can_comms.h
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for ws_can_comms.c
 * @version 0.1
 * @date 2026-03-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef WS_CAN_COMMS_H

#define WS_CAN_COMMS_H

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"

static const uint16_t WS_DCU_BASE_ADDR = 0x500;
static const uint16_t WS_BASE_ADDR = 0x400;

/************************ Drive Control Commands *************************/

void WS_SendMotorDrive(FDCAN_HandleTypeDef *hfdcan, float motorCurrent,
                       float motorRPM);
void WS_SendMotorPower(FDCAN_HandleTypeDef *hfdcan, float busCurrent);
void WS_SendReset(FDCAN_HandleTypeDef *hfdcan);

/***************** Motor Controller Measurement Commands *****************/

void WS_RequestIdentificationInformation(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestStatusInformation(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestBusMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestVelocityMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestPhaseCurrentMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestMotorVoltageVectorMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestMotorCurrentVectorMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestMotorBackEMFMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_Request15VRailMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_Request3V3_1V9RailMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestHeatSinkMotorTempMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestDSPTempMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestOdometerBusAhMeasurement(FDCAN_HandleTypeDef *hfdcan);
void WS_RequestSlipSpeedMeasurement(FDCAN_HandleTypeDef *hfdcan);

#endif