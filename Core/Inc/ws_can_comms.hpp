/**
 * @file ws_can_comms.hpp
 * @author James Metcalf (jammetc@siue.edu)
 * @brief Header file for WaveSculptor CAN communications class
 * @version 0.1
 * @date 2026-03-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef WS_CAN_COMMS_HPP
#define WS_CAN_COMMS_HPP

#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_fdcan.h"

// Debug toggle - set to true to enable debug prints, false to disable
#define WS_DEBUG_ENABLED true

static const uint16_t DEFAULT_WS_BASE_ADDR = 0x400;
static const uint16_t DEFAULT_WS_DCU_BASE_ADDR = 0x500;

/**
 * @brief Enum for WaveSculptor message IDs (offsets from base address)
 */
enum class WaveSculptorMessageID : uint16_t {
  Identification = 0x00,
  Status = 0x01,
  BusMeasurement = 0x02,
  VelocityMeasurement = 0x03,
  PhaseCurrentMeasurement = 0x04,
  MotorVoltageVectorMeasurement = 0x05,
  MotorCurrentVectorMeasurement = 0x06,
  MotorBackEMFMeasurement = 0x07,
  VoltageRail15VMeasurement = 0x08,
  VoltageRail3V3_1V9Measurement = 0x09,
  HeatSinkMotorTempMeasurement = 0x0B,
  DSPTempMeasurement = 0x0C,
  OdometerBusAhMeasurement = 0x0E,
  SlipSpeedMeasurement = 0x17
};

/**
 * @brief Enum for WaveSculptor Driver Control Unit message IDs (offsets from
 * base address)
 */
enum class WaveSculptorDCUMessageID : uint16_t { MotorDrive = 0x01, MotorPower = 0x02, Reset = 0x02 };

/**
 * @brief Class for WaveSculptor motor controller CAN communications
 */
class WaveSculptor {
private:
  FDCAN_HandleTypeDef *hfdcan_;

  // Base addresses
  uint16_t baseAddr_;
  uint16_t dcuBaseAddr_;

  // Measurement data
  uint32_t serialNumber_ = 0;
  uint32_t deviceID_ = 0;
  uint8_t receiveErrorCount_ = 0;
  uint8_t transmitErrorCount_ = 0;
  uint16_t activeMotor_ = 0;
  uint16_t errorFlags_ = 0;
  uint16_t limitFlags_ = 0;
  float busCurrent_ = 0.0f;
  float busVoltage_ = 0.0f;
  float vehicleVelocity_ = 0.0f;
  float motorVelocity_ = 0.0f;
  float phaseCCurrent_ = 0.0f;
  float phaseBCurrent_ = 0.0f;
  float dVoltage_ = 0.0f;
  float qVoltage_ = 0.0f;
  float dCurrent_ = 0.0f;
  float qCurrent_ = 0.0f;
  float dBackEMF_ = 0.0f;
  float qBackEMF_ = 0.0f;
  float measured15VSupply_ = 0.0f;
  float measured3V3Supply_ = 0.0f;
  float measured1V9Supply_ = 0.0f;
  float heatsinkTemp_ = 0.0f;
  float motorTemp_ = 0.0f;
  float dspBoardTemp_ = 0.0f;
  float dcBusAh_ = 0.0f;
  float odometer_ = 0.0f;
  float slipSpeed_ = 0.0f;

  /**
   * @brief Constructor for FDCAN txHeader structs for the WaveSculptor Driver
   * Control Unit
   *
   * @param idOffset
   * @return FDCAN_TxHeaderTypeDef
   */
  inline FDCAN_TxHeaderTypeDef dcuBaseTxHeader(WaveSculptorDCUMessageID id) const {
    FDCAN_TxHeaderTypeDef txHeader = {.Identifier = static_cast<uint32_t>(dcuBaseAddr_ + static_cast<uint16_t>(id)),
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
   * @param idOffset
   * @return FDCAN_TxHeaderTypeDef
   */
  inline FDCAN_TxHeaderTypeDef requestBaseTxHeader(WaveSculptorMessageID id) const {
    FDCAN_TxHeaderTypeDef txHeader = {.Identifier = static_cast<uint32_t>(baseAddr_ + static_cast<uint16_t>(id)),
                                      .IdType = FDCAN_STANDARD_ID,
                                      .TxFrameType = FDCAN_REMOTE_FRAME,
                                      .DataLength = FDCAN_DLC_BYTES_0,
                                      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                                      .BitRateSwitch = FDCAN_BRS_OFF,
                                      .FDFormat = FDCAN_CLASSIC_CAN,
                                      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                                      .MessageMarker = 0};
    return txHeader;
  }

public:
  /**
   * @brief Construct a new Wave Sculptor object
   *
   * @param hfdcan Pointer to FDCAN handle
   * @param baseAddr Base address for WaveSculptor messages (default 0x400)
   * @param dcuBaseAddr Base address for DCU messages (default 0x500)
   */
  WaveSculptor(FDCAN_HandleTypeDef *hfdcan, uint16_t baseAddr = DEFAULT_WS_BASE_ADDR, uint16_t dcuBaseAddr = DEFAULT_WS_DCU_BASE_ADDR);

  /**
   * @brief WaveSculptor Motor Drive Command
   *
   * @param motorCurrent
   * @param motorRPM
   */
  void sendMotorDrive(float motorCurrent, float motorRPM);

  /**
   * @brief WaveSculptor Motor Current Command
   *
   * @param busCurrent
   */
  void sendMotorPower(float busCurrent);

  /**
   * @brief WaveSculptor Reset Command
   */
  void sendReset();

  /**
   * @brief Request identification information from the WaveSculptor motor
   * controller
   */
  void requestIdentificationInformation();

  /**
   * @brief Request status information from the WaveSculptor motor controller
   */
  void requestStatusInformation();

  /**
   * @brief Request bus current and voltage measurement from the WaveSculptor
   * motor controller
   */
  void requestBusMeasurement();

  /**
   * @brief Request vehicle and motor velocity measurement from the WaveSculptor
   * motor controller
   */
  void requestVelocityMeasurement();

  /**
   * @brief Request motor phase B and C current from the WaveSculptor motor
   * controller
   */
  void requestPhaseCurrentMeasurement();

  /**
   * @brief Request motor voltage real and imaginary component measurement from
   * the WaveSculptor motor controller
   */
  void requestMotorVoltageVectorMeasurement();

  /**
   * @brief Request motor current real and imaginary component measurement from
   * the WaveSculptor motor controller
   */
  void requestMotorCurrentVectorMeasurement();

  /**
   * @brief Request motor back EMF measurement from the WaveSculptor motor
   * controller
   */
  void requestMotorBackEMFMeasurement();

  /**
   * @brief Request 15V rail voltage measurement from the WaveSculptor motor
   * controller
   */
  void request15VRailMeasurement();

  /**
   * @brief Request 3.3V and 1.9V rail voltage measurement from the WaveSculptor
   * motor controller
   */
  void request3V3_1V9RailMeasurement();

  /**
   * @brief Request heat-sink and motor temperature measurement from the
   * WaveSculptor motor controller
   */
  void requestHeatSinkMotorTempMeasurement();

  /**
   * @brief Request DSP board temperature measurement from the WaveSculptor
   * motor controller
   */
  void requestDSPTempMeasurement();

  /**
   * @brief Request Odometer and DC Bus AmpHours measurement from the
   * WaveSculptor motor controller
   */
  void requestOdometerBusAhMeasurement();

  /**
   * @brief Request slip speed measurement from the WaveSculptor motor
   * controller
   */
  void requestSlipSpeedMeasurement();

  /**
   * @brief Parse received measurement data
   *
   * @param id CAN message ID offset as enum
   * @param rxData Received data
   */
  void measurementParser(WaveSculptorMessageID id, uint8_t *rxData);

  uint32_t getSerialNumber() const { return serialNumber_; }
  uint32_t getDeviceID() const { return deviceID_; }
  uint8_t getReceiveErrorCount() const { return receiveErrorCount_; }
  uint8_t getTransmitErrorCount() const { return transmitErrorCount_; }
  uint16_t getActiveMotor() const { return activeMotor_; }
  uint16_t getErrorFlags() const { return errorFlags_; }
  uint16_t getLimitFlags() const { return limitFlags_; }
  float getBusCurrent() const { return busCurrent_; }
  float getBusVoltage() const { return busVoltage_; }
  float getVehicleVelocity() const { return vehicleVelocity_; }
  float getMotorVelocity() const { return motorVelocity_; }
  float getPhaseCCurrent() const { return phaseCCurrent_; }
  float getPhaseBCurrent() const { return phaseBCurrent_; }
  float getDVoltage() const { return dVoltage_; }
  float getQVoltage() const { return qVoltage_; }
  float getDCurrent() const { return dCurrent_; }
  float getQCurrent() const { return qCurrent_; }
  float getDBackEMF() const { return dBackEMF_; }
  float getQBackEMF() const { return qBackEMF_; }
  float getMeasured15VSupply() const { return measured15VSupply_; }
  float getMeasured3V3Supply() const { return measured3V3Supply_; }
  float getMeasured1V9Supply() const { return measured1V9Supply_; }
  float getHeatsinkTemp() const { return heatsinkTemp_; }
  float getMotorTemp() const { return motorTemp_; }
  float getDSPBoardTemp() const { return dspBoardTemp_; }
  float getDCBusAh() const { return dcBusAh_; }
  float getOdometer() const { return odometer_; }
  float getSlipSpeed() const { return slipSpeed_; }

  uint16_t getBaseAddr() const { return baseAddr_; }
  uint16_t getDcuBaseAddr() const { return dcuBaseAddr_; }

  /**
   * @brief Check if a CAN ID is within the WaveSculptor message range
   *
   * @param id CAN message ID
   * @return true if ID is within WaveSculptor range
   */
  bool isWaveSculptorMessage(uint32_t id) const {
    return id >= baseAddr_ && id <= baseAddr_ + static_cast<uint16_t>(WaveSculptorMessageID::SlipSpeedMeasurement);
  }
};

#endif /* WS_CAN_COMMS_HPP */
