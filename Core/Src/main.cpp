/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.cpp
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "gpdma.h"
#include "gpio.h"
#include "i2c.h"
#include "icache.h"
#include "spi.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ina228_i2c_comms.hpp"
#include "orion2_can_comms.hpp"
#include "photon3_can_comms.hpp"
#include "ring_buffer.hpp"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_gpio.h"
#include "uart_guard.hpp"
#include "ws_can_comms.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBUG_MESSAGES_ENABLED true

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */
// FDCAN2 defines
FDCAN_TxHeaderTypeDef txHeader;
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t txData[64];
uint8_t rxData[64];

// Ring buffer for deferred RX processing (16 entries for bursts)
static RingBuffer<FDCAN_RxMessage, 16> rxBuffer;

// I2C defines
extern INA228 *g_ina228Instance; // Only works with a single INA228

// Analog input defines
constexpr uint8_t NUM_ANALOG_INPUTS = 4;

// Array for ADC values to be transferred into by DMA
volatile uint16_t analogInputData[NUM_ANALOG_INPUTS];

// Variable to report status of DMA transfer of ADC group regular conversions
//  0: DMA transfer is not completed
//  1: DMA transfer is completed
//  2: DMA transfer has not yet been started yet (initial state)
volatile uint8_t adcDMATransferStatus = 2; // Variable set in DMA interrupt callback

// Orion opto inputs are open-drain and active-low (low = permit on).
volatile bool g_dischargeEnablePermitted = false;    // OI1 -> LIO_4 (Motor)
volatile bool g_chargeEnablePermitted = false;       // OI2 -> LIO_1 (Solar charge)
volatile bool g_multipurposeEnablePermitted = false; // OI3 -> LIO_3/LIO_2 (Battery +/-)
volatile bool g_contactorUpdatePending = true;

// Control switch inputs from external controller (active-low, like Orion inputs).
volatile bool g_runSwitchAsserted = false;                // OI_4 -> Run State
volatile bool g_chargeSwitchAsserted = false;             // OI_5 -> Charge State
volatile bool g_dischargeContactorSwitchAsserted = false; // OI_6 -> Discharge contactor gate (AND with BMS)
volatile bool g_chargeContactorSwitchAsserted = false;    // OI_7 -> Charge contactor gate (AND with BMS)

// BMS control signal outputs (derived from control switches).
volatile bool g_readyPowerOutput = false;  // Ready Power to BMS (Run or Charge state)
volatile bool g_chargePowerOutput = false; // Charge Power to BMS (Charge state only)

constexpr uint32_t CONTACTOR_ON_SPACING_MS = 130U;
uint32_t g_nextContactorOnAllowedTick = 0U;

constexpr uint32_t ORION_DTC_SHORT_FLASH_ON_MS = 125U;
constexpr uint32_t ORION_DTC_LONG_FLASH_ON_MS = 375U;
constexpr uint32_t ORION_DTC_FLASH_OFF_MS = 175U;
constexpr uint32_t ORION_DTC_FLASH_PAUSE_MS = 1200U;
constexpr uint32_t ORION_DTC_CLEAR_DEBOUNCE_MESSAGES = 50U;
volatile uint16_t g_orionDtcFlags1 = 0U;
volatile uint16_t g_orionDtcFlags2 = 0U;
volatile bool g_orionFaultActive = false;
volatile uint8_t g_orionFaultFlashPatternPulseCount = 0U;
volatile uint8_t g_orionFaultFlashPulsesRemaining = 0U;
volatile bool g_orionFaultFlashPulseOn = false;
volatile bool g_orionFaultFlashPulseLong = false;
volatile uint32_t g_orionFaultFlashNextTick = 0U;
uint32_t g_orionFaultClearMessageCount = 0U;

WaveSculptor ws;

constexpr uint32_t DCU_DISABLE_DELAY = 100000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#if DEBUG_MESSAGES_ENABLED
/**
 * @brief Get the number of data bytes from FDCAN DLC code
 * @param dlc FDCAN DLC code
 * @return Number of data bytes
 */
[[maybe_unused]] static uint8_t FDCAN_DLCToBytes(uint32_t dlc) {
  constexpr uint8_t dlcTable[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
  if (dlc <= 15) {
    return dlcTable[dlc];
  }
  return 0;
}
#endif

/**
 * @brief E-Stop function to turn off all contactors immediately and keep them off until exit conditions are met
 *
 */
void eStopFunction() {
#if DEBUG_MESSAGES_ENABLED
  {
    UartGuard guard;
    printf("E-Stop Triggered. Set run switch to off position for 10 seconds and release E-Stop to resume.\n");
  }
#endif

  // ws.disableDCU();
  for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
    ;

  uint32_t offDuration, lastFlash = 0;
  uint32_t offStart = HAL_GetTick();
  uint32_t now = HAL_GetTick();
  while (HAL_GPIO_ReadPin(OI_10_GPIO_Port, OI_10_Pin) == GPIO_PIN_RESET || offDuration < 10000) {
    // Open all contactors
    HAL_GPIO_WritePin(LIO_1_GPIO_Port, LIO_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMO_2_GPIO_Port, LMO_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIO_2_GPIO_Port, LIO_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIO_3_GPIO_Port, LIO_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);

    // Keep fans on
    HAL_GPIO_WritePin(LMO_3_GPIO_Port, LMO_3_Pin, GPIO_PIN_SET);

    if (HAL_GPIO_ReadPin(OI_4_GPIO_Port, OI_4_Pin) == GPIO_PIN_SET && HAL_GPIO_ReadPin(OI_5_GPIO_Port, OI_5_Pin) == GPIO_PIN_SET) {
      offDuration = HAL_GetTick() - offStart;
    } else {
      offDuration = 0;
      offStart = HAL_GetTick();
    }

    if (now - lastFlash >= 250) {
      HAL_GPIO_TogglePin(LMO_1_GPIO_Port, LMO_1_Pin);
      lastFlash = now;
    }

    now = HAL_GetTick();
  }

#if DEBUG_MESSAGES_ENABLED
  {
    UartGuard guard;
    printf("E-Stop exited.\n");
  }
#endif
}

// ============================================================================
// Functions for Switching Contactors Controlled by Orion BMS 2
// ============================================================================

struct OrionFaultFlashPattern {
  uint8_t pulseCount;
  bool longPulse;
};

static OrionFaultFlashPattern GetOrionFaultFlashPattern(uint16_t flags1, uint16_t flags2) {
  for (uint8_t bit = 0U; bit < 16U; ++bit) {
    if ((flags1 & (1U << bit)) != 0U) {
      return {static_cast<uint8_t>(bit + 1U), false};
    }
  }

  for (uint8_t bit = 0U; bit < 16U; ++bit) {
    if ((flags2 & (1U << bit)) != 0U) {
      return {static_cast<uint8_t>(bit + 1U), true};
    }
  }

  return {0U, false};
}

/**
 * @brief Non-blocking sequencer for Orion-controlled contactors.
 *
 * Behavior:
 * - Forces immediate OFF for any de-asserted permit (safe state).
 * - Turns ON at most one contactor per call.
 * - Enforces minimum global spacing between ON transitions.
 * - Preserves ISR-updated pending work to avoid missing edge-driven state changes.
 *
 * Safety/race model:
 * - EXTI ISR owns immediate shutoff on rising edges.
 * - This function only performs deferred ON sequencing from main-loop context.
 */
static void ProcessContactorTurnOnSequencing() {
  bool dischargeContactorPermitted;
  bool chargeContactorPermitted;
  bool sysContactorsPermitted;

  __disable_irq();
  // This cycle consumes the pending work flag; ISR can re-assert it at any time.
  g_contactorUpdatePending = false;
  dischargeContactorPermitted = g_dischargeEnablePermitted && g_runSwitchAsserted;
  chargeContactorPermitted = g_chargeEnablePermitted && (g_runSwitchAsserted || g_chargeSwitchAsserted);
  sysContactorsPermitted = g_multipurposeEnablePermitted && (g_runSwitchAsserted || g_chargeSwitchAsserted);
  __enable_irq();

  // Rising edge de-assertions should already switch off in ISR; this keeps state coherent.
  if (!sysContactorsPermitted) {
    // Turning off the contactors is always safe, so no need to write the global state

    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(LIO_3_GPIO_Port, LIO_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIO_2_GPIO_Port, LIO_2_Pin, GPIO_PIN_RESET);
  }
  if (!chargeContactorPermitted) {
    // Turning off the contactors is always safe, so no need to write the global state
    HAL_GPIO_WritePin(LIO_1_GPIO_Port, LIO_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMO_2_GPIO_Port, LMO_2_Pin, GPIO_PIN_RESET);
  }
  if (!dischargeContactorPermitted) {
    // Turning off the contactors is always safe, so no need to write the global state

    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);
  }
  // Force discharge contactor (motor) OFF if in charge mode for safety.
  // The car must not be drivable while plugged in to the charger.
  if (g_chargeSwitchAsserted) {
    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);
  }

  uint32_t now = HAL_GetTick();
  if (static_cast<int32_t>(now - g_nextContactorOnAllowedTick) < 0) {
    g_contactorUpdatePending = true;
    return;
  }

  // Turn on at most one contactor per call; enforce minimum spacing globally.
  // Note: Discharge (LIO_4) and Charge (LIO_1) are gated by their respective external control switches
  // via AND logic: contactor can only turn on if both BMS permit AND external switch assert.
  if (sysContactorsPermitted && (HAL_GPIO_ReadPin(LIO_3_GPIO_Port, LIO_3_Pin) == GPIO_PIN_RESET)) {
    // Re-read permit inside the critical section so the GPIO write cannot use stale authorization.
    __disable_irq();
    HAL_GPIO_WritePin(LIO_3_GPIO_Port, LIO_3_Pin, static_cast<GPIO_PinState>(g_multipurposeEnablePermitted)); // Switch battery (-) contactor
    __enable_irq();
    g_nextContactorOnAllowedTick = now + CONTACTOR_ON_SPACING_MS;
  } else if (sysContactorsPermitted && (HAL_GPIO_ReadPin(LIO_2_GPIO_Port, LIO_2_Pin) == GPIO_PIN_RESET)) {
    // Re-read permit inside the critical section so the GPIO write cannot use stale authorization.
    __disable_irq();
    HAL_GPIO_WritePin(LIO_2_GPIO_Port, LIO_2_Pin, static_cast<GPIO_PinState>(g_multipurposeEnablePermitted)); // Switch battery (+) contactor
    __enable_irq();
    g_nextContactorOnAllowedTick = now + CONTACTOR_ON_SPACING_MS;
  } else if (chargeContactorPermitted && g_chargeContactorSwitchAsserted && (HAL_GPIO_ReadPin(LIO_1_GPIO_Port, LIO_1_Pin) == GPIO_PIN_RESET)) {
    // Charge contactor (LIO_1) requires both BMS charge permit AND external charge control switch.
    // Re-read both permit and switch inside the critical section to prevent stale values.
    __disable_irq();
    bool chargeGateOpen = g_chargeEnablePermitted && g_chargeContactorSwitchAsserted;
    HAL_GPIO_WritePin(LIO_1_GPIO_Port, LIO_1_Pin, static_cast<GPIO_PinState>(chargeGateOpen)); // Switch solar relay
    HAL_GPIO_WritePin(LMO_2_GPIO_Port, LMO_2_Pin, static_cast<GPIO_PinState>(chargeGateOpen)); // Switch motor relay
    __enable_irq();
    g_nextContactorOnAllowedTick = now + CONTACTOR_ON_SPACING_MS;
  } else if (dischargeContactorPermitted && g_dischargeContactorSwitchAsserted && !g_chargeSwitchAsserted &&
             (HAL_GPIO_ReadPin(LIO_4_GPIO_Port, LIO_4_Pin) == GPIO_PIN_RESET)) {
    // Discharge contactor (LIO_4) requires: BMS discharge permit AND external discharge control switch
    // AND NOT in charge mode. Charge mode prevents motor operation for safety.
    // Re-read both permit and switch inside the critical section to prevent stale values.
    __disable_irq();
    bool dischargeGateOpen = g_dischargeEnablePermitted && g_dischargeContactorSwitchAsserted && !g_chargeSwitchAsserted;
    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, static_cast<GPIO_PinState>(dischargeGateOpen)); // Switch motor contactor
    __enable_irq();
    g_nextContactorOnAllowedTick = now + CONTACTOR_ON_SPACING_MS;
  }

  // Turn on fans when both system contactors are closed or if a BMS fault is active
  if ((HAL_GPIO_ReadPin(LIO_2_GPIO_Port, LIO_2_Pin) && HAL_GPIO_ReadPin(LIO_3_GPIO_Port, LIO_3_Pin)) || g_orionFaultActive) {
    HAL_GPIO_WritePin(LMO_3_GPIO_Port, LMO_3_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(LMO_3_GPIO_Port, LMO_3_Pin, GPIO_PIN_RESET);
  }

  // Enable the DCU after the system and motor contactors are closed.
  if (HAL_GPIO_ReadPin(LIO_2_GPIO_Port, LIO_2_Pin) && HAL_GPIO_ReadPin(LIO_3_GPIO_Port, LIO_3_Pin) && HAL_GPIO_ReadPin(LIO_4_GPIO_Port, LIO_4_Pin)) {
    // ws.enableDCU();
  }

  __disable_irq();
  dischargeContactorPermitted = g_dischargeEnablePermitted && g_runSwitchAsserted;
  chargeContactorPermitted = g_chargeEnablePermitted && (g_runSwitchAsserted || g_chargeSwitchAsserted);
  sysContactorsPermitted = g_multipurposeEnablePermitted && (g_runSwitchAsserted || g_chargeSwitchAsserted);
  bool dischargeSwitch = g_dischargeContactorSwitchAsserted;
  bool chargeSwitch = g_chargeContactorSwitchAsserted;
  __enable_irq();
  // Preserve any ISR update that may have occurred while this function was running.
  // Note: Charge and discharge contactors now include switch gating in the condition.
  // Discharge is also blocked when in charge mode (OI_5 asserted) for safety.
  g_contactorUpdatePending =
      g_contactorUpdatePending || ((sysContactorsPermitted && (HAL_GPIO_ReadPin(LIO_3_GPIO_Port, LIO_3_Pin) == GPIO_PIN_RESET)) ||
                                   (sysContactorsPermitted && (HAL_GPIO_ReadPin(LIO_2_GPIO_Port, LIO_2_Pin) == GPIO_PIN_RESET)) ||
                                   (chargeContactorPermitted && chargeSwitch && (HAL_GPIO_ReadPin(LIO_1_GPIO_Port, LIO_1_Pin) == GPIO_PIN_RESET)) ||
                                   (dischargeContactorPermitted && dischargeSwitch && !g_chargeSwitchAsserted &&
                                    (HAL_GPIO_ReadPin(LIO_4_GPIO_Port, LIO_4_Pin) == GPIO_PIN_RESET)));
}

static void ProcessOrionFaultIndication(uint32_t now) {
  // Keep all contactors open while the BMS reports a fault.
  HAL_GPIO_WritePin(LIO_1_GPIO_Port, LIO_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LMO_2_GPIO_Port, LMO_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIO_2_GPIO_Port, LIO_2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIO_3_GPIO_Port, LIO_3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);

  // Keep fans on during fault
  HAL_GPIO_WritePin(LMO_3_GPIO_Port, LMO_2_Pin, GPIO_PIN_SET);

  if (!g_orionFaultActive || g_orionFaultFlashPatternPulseCount == 0U) {
    HAL_GPIO_WritePin(LMO_1_GPIO_Port, LMO_1_Pin, GPIO_PIN_RESET);
    return;
  }

  if (static_cast<int32_t>(now - g_orionFaultFlashNextTick) < 0) {
    return;
  }

  if (!g_orionFaultFlashPulseOn) {
    HAL_GPIO_WritePin(LMO_1_GPIO_Port, LMO_1_Pin, GPIO_PIN_SET);
    g_orionFaultFlashPulseOn = true;
    g_orionFaultFlashNextTick = now + (g_orionFaultFlashPulseLong ? ORION_DTC_LONG_FLASH_ON_MS : ORION_DTC_SHORT_FLASH_ON_MS);
    return;
  }

  HAL_GPIO_WritePin(LMO_1_GPIO_Port, LMO_1_Pin, GPIO_PIN_RESET);
  g_orionFaultFlashPulseOn = false;

  uint8_t pulsesRemaining = g_orionFaultFlashPulsesRemaining;
  if (pulsesRemaining > 0) {
    --pulsesRemaining;
    g_orionFaultFlashPulsesRemaining = pulsesRemaining;
  }

  if (g_orionFaultFlashPulsesRemaining == 0U) {
    g_orionFaultFlashPulsesRemaining = g_orionFaultFlashPatternPulseCount;
    g_orionFaultFlashNextTick = now + ORION_DTC_FLASH_PAUSE_MS;
  } else {
    g_orionFaultFlashNextTick = now + ORION_DTC_FLASH_OFF_MS;
  }
}

static void UpdateOrionFaultState(uint16_t flags1, uint16_t flags2, uint32_t now) {
  OrionFaultFlashPattern pattern = GetOrionFaultFlashPattern(flags1, flags2);
  bool requestDcuDisable = false;
  bool clearFault = false;
  bool patternChanged = false;

  __disable_irq();
  g_orionDtcFlags1 = flags1;
  g_orionDtcFlags2 = flags2;

  if (pattern.pulseCount > 0U) {
    patternChanged =
        !g_orionFaultActive || (g_orionFaultFlashPatternPulseCount != pattern.pulseCount) || (g_orionFaultFlashPulseLong != pattern.longPulse);

    requestDcuDisable = !g_orionFaultActive;
    g_orionFaultActive = true;
    g_orionFaultClearMessageCount = 0U;
  } else if (g_orionFaultActive) {
    if (g_orionFaultClearMessageCount < ORION_DTC_CLEAR_DEBOUNCE_MESSAGES) {
      ++g_orionFaultClearMessageCount;
    }

    if (g_orionFaultClearMessageCount >= ORION_DTC_CLEAR_DEBOUNCE_MESSAGES) {
      g_orionFaultActive = false;
      g_orionFaultClearMessageCount = 0U;
      clearFault = true;
    }
  } else {
    g_orionFaultClearMessageCount = 0U;
  }

  if (requestDcuDisable) {
    __enable_irq();
    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;
    __disable_irq();
  }

  if (patternChanged) {
    g_orionFaultFlashPatternPulseCount = pattern.pulseCount;
    g_orionFaultFlashPulsesRemaining = pattern.pulseCount;
    g_orionFaultFlashPulseLong = pattern.longPulse;
    g_orionFaultFlashPulseOn = false;
    g_orionFaultFlashNextTick = now;
  }

  if (clearFault) {
    HAL_GPIO_WritePin(LMO_1_GPIO_Port, LMO_1_Pin, GPIO_PIN_RESET);
    g_orionFaultFlashPatternPulseCount = 0U;
    g_orionFaultFlashPulsesRemaining = 0U;
    g_orionFaultFlashPulseOn = false;
    g_orionFaultFlashPulseLong = false;
    g_orionFaultFlashNextTick = 0U;
  }

  g_contactorUpdatePending = true;
  __enable_irq();
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_ICACHE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  // Start FDCAN2
  if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
  // Activate FIFO0 receipt notification
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
    /* Notification Error */
    Error_Handler();
  }
  // Activate interrupts for proper Bus-Off recovery
  if (HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0) != HAL_OK) {
    /* Notification Error */
    Error_Handler();
  }

  // Configure TX Header for FDCAN2
  txHeader.Identifier = 0x11;
  txHeader.IdType = FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_OFF;
  txHeader.FDFormat = FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;

  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) != HAL_OK) {
    /* Calibration Error */
    Error_Handler();
  }

  /* Start ADC group regular conversion */
  /* Note: First start with DMA transfer initialization, following ones with basic ADC start. */
  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t *)analogInputData, NUM_ANALOG_INPUTS) != HAL_OK) {
    /* Error: ADC conversion start could not be performed */
    Error_Handler();
  }

  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits = COM_STOPBITS_1;
  BspCOMInit.Parity = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  if (HAL_GPIO_ReadPin(OI_10_GPIO_Port, OI_10_Pin) == GPIO_PIN_RESET) {
    eStopFunction();
  }

  g_runSwitchAsserted = (HAL_GPIO_ReadPin(OI_4_GPIO_Port, OI_4_Pin) == GPIO_PIN_RESET);
  g_chargeSwitchAsserted = (HAL_GPIO_ReadPin(OI_5_GPIO_Port, OI_5_Pin) == GPIO_PIN_RESET);
  g_dischargeEnablePermitted = (HAL_GPIO_ReadPin(OI_1_GPIO_Port, OI_1_Pin) == GPIO_PIN_RESET);
  g_chargeEnablePermitted = (HAL_GPIO_ReadPin(OI_2_GPIO_Port, OI_2_Pin) == GPIO_PIN_RESET);
  g_multipurposeEnablePermitted = (HAL_GPIO_ReadPin(OI_3_GPIO_Port, OI_3_Pin) == GPIO_PIN_RESET);
  g_dischargeContactorSwitchAsserted = (HAL_GPIO_ReadPin(OI_6_GPIO_Port, OI_6_Pin) == GPIO_PIN_RESET);
  g_chargeContactorSwitchAsserted = (HAL_GPIO_ReadPin(OI_7_GPIO_Port, OI_7_Pin) == GPIO_PIN_RESET);
  g_nextContactorOnAllowedTick = HAL_GetTick();
  g_contactorUpdatePending = true;

  HAL_GPIO_WritePin(HMO_2_GPIO_Port, HMO_2_Pin, static_cast<GPIO_PinState>(!g_runSwitchAsserted));
  HAL_GPIO_WritePin(HMO_1_GPIO_Port, HMO_1_Pin, static_cast<GPIO_PinState>(!g_chargeSwitchAsserted));

  // Initialize CAN devices
  ws.init(&hfdcan2, 0x400, 0x500);
  Photon3 photon3(&hfdcan2, 0x600);
  Orion2 orion2(&hfdcan2, 0x6B0);

  // Initialize I2C devices
  INA228 pwrMonitor(&hi2c1);
  // Configuration is blocking, but all other comms are non-blocking
  pwrMonitor.reset();
  HAL_Delay(5);
  pwrMonitor.setConfig(); // Use default config (Range0)
  HAL_Delay(5);
  pwrMonitor.setADCConfig(); // Use default config (TempShuntBusCont, us1052, Avg1)
  HAL_Delay(5);
  pwrMonitor.setShuntCalibration(0.007f, 12.5f); // Configure according to hardware
  HAL_Delay(5);

  pwrMonitor.verifyDevicePresent();

  uint32_t now, lastADCRead, lastPwrMonitorRead, lastAliveBlink;
  lastADCRead = lastPwrMonitorRead = lastAliveBlink = 0;

  while (1) {
    now = HAL_GetTick();

    if (g_orionFaultActive) {
      ProcessOrionFaultIndication(now);
    } else {
      if (g_contactorUpdatePending) {
        // Run deferred sequencing in main-loop context; ISR handles immediate shutoff.
        ProcessContactorTurnOnSequencing();
      }
      HAL_GPIO_WritePin(LMO_1_GPIO_Port, LMO_1_Pin, GPIO_PIN_RESET);
    }

    if (lastAliveBlink - now >= 750) {
      // Blink to show alive
      BSP_LED_Toggle(LED_GREEN);
      lastAliveBlink = now;
    }

    if (now - lastADCRead >= 3000) {
      lastADCRead = now;
      /* Start ADC group regular conversion */
      if (HAL_ADC_Start(&hadc1) != HAL_OK) {
        /* Error: ADC conversion start could not be performed */
#if DEBUG_MESSAGES_ENABLED
        {
          UartGuard guard;
          printf("ADC conversion could not be started: Error = 0x%08X\n", HAL_ADC_GetError(&hadc1));
        }
#endif
      }
    }

    if (now - lastPwrMonitorRead >= 2000) {
      lastPwrMonitorRead = now;
      // Start pwrMonitor I2C read
      pwrMonitor.startReadAllMeasurements();
    }

    // Retrieve pwrMonitor data if it is ready
    if (pwrMonitor.isDataReady()) {
      float shuntV = pwrMonitor.getShuntVoltage(); // in Volts
      float busV = pwrMonitor.getBusVoltage();     // in Volts
      float current = pwrMonitor.getCurrent();     // in Amperes
      float power = pwrMonitor.getPower();         // in Watts
      float temp = pwrMonitor.getTemperature();    // in Celsius

      // Reset dataReady_ after retrieving
      pwrMonitor.setDataReady(false);
#if DEBUG_MESSAGES_ENABLED
      {
        UartGuard guard;
        printf("Power Monitor: Data retrieved - Vshunt=%.6f, Vbus=%.2f, I=%.3f, P=%.3f, Tdie=%.1f\n", shuntV, busV, current, power, temp);
      }
#endif
    }

    // Process buffered CAN RX messages
    FDCAN_RxMessage msg;
    while (rxBuffer.pop(msg)) {
      // Check if message is from WaveSculptor
      if (ws.isWaveSculptorMessage(msg.header.Identifier)) {
        uint16_t offset = msg.header.Identifier - ws.getBaseAddr();
        ws.parseMeasurement(static_cast<WaveSculptor::MessageID>(offset), msg.data.data());
      } else if (photon3.isPhoton3Message(msg.header.Identifier)) {
        uint16_t offset = msg.header.Identifier - photon3.getBaseAddr();
        photon3.parseMeasurement(static_cast<Photon3::MessageID>(offset), msg.data.data());
      } else if (orion2.isOrion2Message(msg.header.Identifier)) {
        uint16_t offset = msg.header.Identifier - orion2.getBaseAddr();
        orion2.parseMeasurement(static_cast<Orion2::MessageID>(offset), msg.data.data());
      } else {

        // Handle other messages or ignore
        // #if DEBUG_MESSAGES_ENABLED
        //         UartGuard guard;
        //         uint8_t dataLength = FDCAN_DLCToBytes(msg.header.DataLength);
        //         printf("Received unknown message: ID 0x%x, DLC %u, Data: ", (unsigned int)msg.header.Identifier, dataLength);
        //         for (uint8_t i = 0; i < dataLength; i++) {
        //           printf("%02X ", msg.data[i]);
        //         }
        //         printf("\n");
        // #endif
      }
    }

    if (adcDMATransferStatus == 1) {
      // Reset transfer status
      adcDMATransferStatus = 0;

      // Process the data

#if DEBUG_MESSAGES_ENABLED
      {
        UartGuard guard;
        for (uint8_t i = 0; i < NUM_ANALOG_INPUTS; i++) {
          printf("Analog Input %d: %4d\n", i, analogInputData[i]);
        }
      }
#endif
    }

    HAL_Delay(5);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the programming delay
   */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/* USER CODE BEGIN 4 */

/**
 * @brief Callback function for the interrupt triggered when receiving a message into Fifo0
 *
 * @param hfdcan
 * @param RxFifo0ITs
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    /* Retrieve Rx messages from RX FIFO0 */
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
      /* Reception Error */
#if DEBUG_MESSAGES_ENABLED
      {
        UartGuard guard;
        printf("FDCAN: RX FIFO0 GetRxMessage failed\n");
      }
#endif
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
      /* Notification Error */
#if DEBUG_MESSAGES_ENABLED
      {
        UartGuard guard;
        printf("FDCAN: RX FIFO0 ActivateNotification failed\n");
      }
#endif
    }

    if (rxHeader.Identifier == 0x001) { // Critical message (Orion BMS 2 DTC Faults)
      uint16_t orionDTCFlags1 = 0U;
      uint16_t orionDTCFlags2 = 0U;
      memcpy(&orionDTCFlags1, &rxData[0], sizeof(orionDTCFlags1));
      memcpy(&orionDTCFlags2, &rxData[2], sizeof(orionDTCFlags2));

      UpdateOrionFaultState(orionDTCFlags1, orionDTCFlags2, HAL_GetTick());
    } else {
      // Push to buffer for deferred processing
      FDCAN_RxMessage msg = {rxHeader, {}};
      std::copy(std::begin(rxData), std::end(rxData), msg.data.begin());
      rxBuffer.push(msg);
    }
  }
}

/**
 * @brief Callback function for the interrupt triggered when an FDCAN error status occurs
 *
 * @param hfdcan
 * @param ErrorStatusITs
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
  // If Bus-Off error occurred
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0) {
    hfdcan->Instance->CCCR &= ~FDCAN_CCCR_INIT; // Clear INIT bit to recover from Bus-Off
  }
}

/**
 * @brief Callback function for the interrupt triggered when an I2C Memory Read (IT or DMA) is complete
 *
 * @param hi2c
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (g_ina228Instance) {
    g_ina228Instance->onI2CMemRxComplete(hi2c);
  }
}

/**
 * @brief  DMA transfer complete callback
 * @note   This function is executed when the transfer complete interrupt is generated
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  // Update DMA transfer status variable
  adcDMATransferStatus = 1;
}

/**
 * @brief EXTI rising edge callback for Orion opto-input permit revocation and control switch de-assertion.
 *
 * Inputs are active-low (open-drain with pull-up):
 * - low  = permit ON / switch ON
 * - high = revoke permit / switch OFF
 *
 * Rising edge (low→high) indicates permit/switch revocation or de-assertion.
 * For Orion BMS inputs (OI_1-3): immediately shutoff mapped contactors (safety-critical).
 * For control switches (OI_4-7): update switch state and request sequencing reconciliation.
 *
 * @param GPIO_Pin EXTI source pin number provided by HAL.
 */
void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case OI_1_Pin: // Discharge EN
    g_dischargeEnablePermitted = false;

    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);
    g_contactorUpdatePending = true;
    break;

  case OI_2_Pin: // Charge EN
    g_chargeEnablePermitted = false;
    HAL_GPIO_WritePin(LIO_1_GPIO_Port, LIO_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMO_2_GPIO_Port, LMO_2_Pin, GPIO_PIN_RESET);
    g_contactorUpdatePending = true;
    break;

  case OI_3_Pin: // Multipurpose EN
    g_multipurposeEnablePermitted = false;

    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(LIO_3_GPIO_Port, LIO_3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIO_2_GPIO_Port, LIO_2_Pin, GPIO_PIN_RESET);
    g_contactorUpdatePending = true;
    break;

  case OI_4_Pin: // Run Switch (Run)
    // Run state switch de-asserted. Force outputs to off state (both switches inactive = off).
    g_runSwitchAsserted = false;
    HAL_GPIO_WritePin(HMO_1_GPIO_Port, HMO_1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(HMO_2_GPIO_Port, HMO_2_Pin, GPIO_PIN_SET);
    g_contactorUpdatePending = true; // Allow discharge contactor to be re-evaluated
    break;

  case OI_5_Pin: // Run Switch (Charge)
    // Charge state switch de-asserted. If Run switch also inactive, turn off all outputs.
    g_chargeSwitchAsserted = false;
    if (!g_runSwitchAsserted) {
      HAL_GPIO_WritePin(HMO_1_GPIO_Port, HMO_1_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(HMO_2_GPIO_Port, HMO_2_Pin, GPIO_PIN_SET);
    } else {
      // Run switch is active: go to Run mode (Ready Power on, Charge Power off).
      HAL_GPIO_WritePin(HMO_1_GPIO_Port, HMO_1_Pin, GPIO_PIN_SET);
    }
    g_contactorUpdatePending = true; // Allow discharge contactor to be re-evaluated
    break;

  case OI_6_Pin: // Discharge Contactor Enable
    // Discharge contactor control switch de-asserted. Force contactor OFF immediately.
    g_dischargeContactorSwitchAsserted = false;

    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);
    g_contactorUpdatePending = true;
    break;

  case OI_7_Pin: // Charge Contactor Enable
    // Charge contactor control switch de-asserted. Force contactor OFF immediately.
    g_chargeContactorSwitchAsserted = false;
    HAL_GPIO_WritePin(LIO_1_GPIO_Port, LIO_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LMO_2_GPIO_Port, LMO_2_Pin, GPIO_PIN_RESET);
    g_contactorUpdatePending = true;
    break;

  default:
    break;
  }
}

/**
 * @brief EXTI falling edge callback for Orion opto-input permit grant and control switch assertion.
 *
 * Inputs are active-low (open-drain with pull-up):
 * - low  = permit ON / switch ON
 * - high = revoke permit / switch OFF
 *
 * Falling edge (high→low) indicates permit/switch grant or assertion.
 * For Orion BMS inputs (OI_1-3): permits are re-asserted and deferred sequencing is requested.
 * For control switches (OI_4-7): switch states are asserted and sequencing is requested.
 *
 * @param GPIO_Pin EXTI source pin number provided by HAL.
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
  switch (GPIO_Pin) {
  case OI_1_Pin: // Discharge EN
    g_dischargeEnablePermitted = true;
    g_contactorUpdatePending = true;
    break;

  case OI_2_Pin: // Charge EN
    g_chargeEnablePermitted = true;
    g_contactorUpdatePending = true;
    break;

  case OI_3_Pin: // Multipurpose EN
    g_multipurposeEnablePermitted = true;
    g_contactorUpdatePending = true;
    break;

  case OI_4_Pin: // Run Switch (Run)
    // Run state switch asserted (Run mode): Ready Power on, Charge Power off.
    g_runSwitchAsserted = true;
    HAL_GPIO_WritePin(HMO_2_GPIO_Port, HMO_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HMO_1_GPIO_Port, HMO_1_Pin, GPIO_PIN_SET);
    g_contactorUpdatePending = true; // Allow discharge contactor to be re-evaluated
    break;

  case OI_5_Pin: // Run Switch (Charge)
    // Charge state switch asserted (Charge mode): both Ready Power and Charge Power on.
    // Also force discharge contactor off.
    g_chargeSwitchAsserted = true;

    // ws.disableDCU();
    for (uint32_t i = 0; i < DCU_DISABLE_DELAY; i++)
      ;

    HAL_GPIO_WritePin(HMO_1_GPIO_Port, HMO_1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(HMO_2_GPIO_Port, HMO_2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LIO_4_GPIO_Port, LIO_4_Pin, GPIO_PIN_RESET);
    g_contactorUpdatePending = true; // Force discharge contactor evaluation
    break;

  case OI_6_Pin: // Discharge Contactor Enable
    // Discharge contactor control switch asserted. Request sequencing for turn-on.
    g_dischargeContactorSwitchAsserted = true;
    g_contactorUpdatePending = true;
    break;

  case OI_7_Pin: // Charge Contactor Enable
    // Charge contactor control switch asserted. Request sequencing for turn-on.
    g_chargeContactorSwitchAsserted = true;
    g_contactorUpdatePending = true;
    break;

  case OI_10_Pin: // E-Stop triggered
    eStopFunction();
    break;

  default:
    break;
  }
}

/**
 * @brief  Button callback when USER button is pressed (for testing).
 * @note   Reports all control switch states and BMS signal outputs.
 * @param  Button The button number (BUTTON_USER in this case).
 * @retval None
 */
void BSP_PB_Callback(Button_TypeDef Button) {
  if (Button == BUTTON_USER) {
    // Report all control switch and permission states for debugging.
    UartGuard guard;
    printf("\n=== Control State Report ===");
    printf("\nOrion BMS Permits:");
    printf("\n  Discharge Enable (OI_1): %d", g_dischargeEnablePermitted);
    printf("\n  Charge Enable (OI_2):    %d", g_chargeEnablePermitted);
    printf("\n  Multipurpose (OI_3):     %d", g_multipurposeEnablePermitted);
    printf("\nExternal Control Switches:");
    printf("\n  Run State (OI_4):        %d", g_runSwitchAsserted);
    printf("\n  Charge State (OI_5):     %d", g_chargeSwitchAsserted);
    printf("\n  Discharge Switch (OI_6): %d", g_dischargeContactorSwitchAsserted);
    printf("\n  Charge Switch (OI_7):    %d", g_chargeContactorSwitchAsserted);
    printf("\nBMS Output Signals (via HMO):");
    printf("\n  HMO_1 (Charge Power):    %d", HAL_GPIO_ReadPin(HMO_1_GPIO_Port, HMO_1_Pin) == GPIO_PIN_RESET);
    printf("\n  HMO_2 (Ready Power):     %d", HAL_GPIO_ReadPin(HMO_2_GPIO_Port, HMO_2_Pin) == GPIO_PIN_RESET);
    printf("\nContactor States:");
    printf("\n  LIO_1 (Solar Relay):     %d", HAL_GPIO_ReadPin(LIO_1_GPIO_Port, LIO_1_Pin));
    printf("\n  LIO_2 (Battery +):       %d", HAL_GPIO_ReadPin(LIO_2_GPIO_Port, LIO_2_Pin));
    printf("\n  LIO_3 (Battery -):       %d", HAL_GPIO_ReadPin(LIO_3_GPIO_Port, LIO_3_Pin));
    printf("\n  LIO_4 (Motor):           %d\n", HAL_GPIO_ReadPin(LIO_4_GPIO_Port, LIO_4_Pin));
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  {
    UartGuard guard;
    printf("\n\nThere was an error\n");
  }
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
