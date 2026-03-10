/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c  (single-file example)
 * @brief          : STM32 (FDCAN2) ↔ MCP2562 ↔ WaveSculptor 22 CAN protocol
 * demo
 ******************************************************************************
 * @attention
 *
 * This example configures FDCAN2 for classic CAN 2.0 at 500 kbps, sets filters
 * to receive WaveSculptor22 (WS22) telemetry, and (edited) periodically
 * requests Status (0x421) and Bus Measurement (0x422) using Remote Frames (RTR).
 * Payloads for Bus Measurement are two float32 values (LSB-first) per WS22
 * Appendix C.
 * [1](https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html)
 *
 * MCP2562 notes:
 *  - STBY pin must be LOW for Normal mode (else you won't see the bus).
 *  - VDD = 5V, VIO = MCU I/O supply (e.g., 3.3V), decouple both.
 *  - Proper CAN bus: twisted pair, 120 Ω termination at both ends.
 * [2](https://ww1.microchip.com/downloads/en/DeviceDoc/20005167C.pdf)[3](https://nusolar.github.io/training-f23/pdfs/WaveSculptor22.pdf)
 *
 * WaveSculptor protocol essentials:
 *  - 11-bit IDs, base address uses bits 10..5; message ID uses bits 4..0.
 *  - Two IEEE‑754 float32 values (LSB-first) per 8-byte frame for measurement frames.
 *  - Remote frames (RTR) are enabled; sending RTR to 0x421/0x422 provokes reply.
 * [1](https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html)
 *
 * Copyright (c) 2026.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_nucleo.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdint.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ================= User-configurable IDs (match your system) ===============
 */
#define DRIVER_BASE 0x400U /* Your STM32 node (Driver Controls)          */
#define WS_BASE     0x420U /* WaveSculptor22 base address                 */
/* IDs per WS22 Appendix C: device field=bits10..5; message field=bits4..0. */

/* ================= Optional: MCP2562 STBY GPIO ============================= */
/* If your MCP2562 STBY pin is wired to a GPIO, define its port/pin here.
   Otherwise ensure STBY is strapped LOW in hardware. STBY=LOW -> Normal mode.
   [2](https://ww1.microchip.com/downloads/en/DeviceDoc/20005167C.pdf) */
// #define MCP2562_STBY_GPIO_Port GPIOX
// #define MCP2562_STBY_Pin       GPIO_PIN_Y

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline void pack_f32_le(uint8_t *dst, float v) {
  memcpy(dst, &v, sizeof(float));
}
static inline float unpack_f32_le(const uint8_t *src) {
  float v;
  memcpy(&v, src, sizeof(float));
  return v;
}

/* DLC code → byte length (covers FD too; we clamp to ≤8 for classic) */
static inline uint8_t dlc_to_len(uint32_t dlc_code) {
  static const uint8_t map[16] = {0, 1,  2,  3,  4,  5,  6,  7,
                                  8, 12, 16, 20, 24, 32, 48, 64};
  return map[dlc_code & 0xF]; // <-- no shifting
}
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

FDCAN_HandleTypeDef hfdcan2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

/* USER CODE BEGIN PV */
/* Removed drive command usage; we only request/receive now */
static uint32_t last_led_ms = 0;
/* Timers for RTR cadence */
static uint32_t t_status = 0, t_bus = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_ICACHE_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length);
HAL_StatusTypeDef CAN_ReceiveMessage(uint32_t *id, uint8_t *data,
                                     uint8_t *length);
static void WS_ProcessRxOnce(void);

/* NEW: RTR helper + WS request wrappers */
static void CAN_RequestRemote(uint32_t std_id, uint8_t dlc_bytes);
static inline void WS_RequestStatusOnce(void) { CAN_RequestRemote(WS_BASE + 0x21U, 8); } /* 0x421 */
static inline void WS_RequestBusOnce(void)    { CAN_RequestRemote(WS_BASE + 0x22U, 8); } /* 0x422 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Classic CAN TX helper (clamps to 0..8 bytes) */
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t length) {
  FDCAN_TxHeaderTypeDef TxHeader = {0};
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (length > 8)
    length = 8;
  switch (length) {
  case 0:
    TxHeader.DataLength = FDCAN_DLC_BYTES_0;
    break;
  case 1:
    TxHeader.DataLength = FDCAN_DLC_BYTES_1;
    break;
  case 2:
    TxHeader.DataLength = FDCAN_DLC_BYTES_2;
    break;
  case 3:
    TxHeader.DataLength = FDCAN_DLC_BYTES_3;
    break;
  case 4:
    TxHeader.DataLength = FDCAN_DLC_BYTES_4;
    break;
  case 5:
    TxHeader.DataLength = FDCAN_DLC_BYTES_5;
    break;
  case 6:
    TxHeader.DataLength = FDCAN_DLC_BYTES_6;
    break;
  case 7:
    TxHeader.DataLength = FDCAN_DLC_BYTES_7;
    break;
  default:
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    break;
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, data) != HAL_OK) {
    Error_Handler();
  }
}

/* NEW: Classic CAN Remote Frame (RTR) TX helper */
static void CAN_RequestRemote(uint32_t std_id, uint8_t dlc_bytes) {
  FDCAN_TxHeaderTypeDef TxHeader = {0};
  TxHeader.Identifier = std_id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_REMOTE_FRAME;   /* RTR */
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  if (dlc_bytes > 8) dlc_bytes = 8;
  switch (dlc_bytes) {
    case 0: TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
    case 1: TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
    case 2: TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
    case 3: TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
    case 4: TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
    case 5: TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
    case 6: TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
    case 7: TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
    default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
  }

  /* Data pointer is ignored for RTR but HAL expects it. */
  uint8_t dummy[8] = {0};
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, dummy) != HAL_OK) {
    Error_Handler();
  }
}

/* Polling RX helper (FIFO0) */
HAL_StatusTypeDef CAN_ReceiveMessage(uint32_t *id, uint8_t *data,
                                     uint8_t *length) {
  FDCAN_RxHeaderTypeDef RxHeader;
  HAL_StatusTypeDef status =
      HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, data);
  if (status == HAL_OK) {
    *id = RxHeader.Identifier;
    *length = dlc_to_len(RxHeader.DataLength);
    if (*length > 8)
      *length = 8; // classic safety
  }
  return status;
}

/* Parse WS22 telemetry and print numeric-only CSV lines.
   Known IDs:
     0x421 (WS_BASE+0x21): status          -> "status,rx,tx,active,err,lim"
     0x422 (WS_BASE+0x22): bus meas        -> "bus,current_A,voltage_V"
     0x423 (WS_BASE+0x23): velocity        -> "vel,vehicle_mps,motor_rpm"
     0x42B (WS_BASE+0x2B): temperatures    -> "temps,heatsink_C,motor_C"
     0x405 (WS_BASE+0x05): voltage vector  -> "vvec,Vd,Vq"
     0x7F1:               boot heartbeat   -> "boot,1"
   Any other 8-byte frame falls back to:   -> "raw,<id_dec>,b0..b7"
*/
static void WS_ProcessRxOnce(void) {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
    if (CAN_ReceiveMessage(&id, data, &len) != HAL_OK) {
      break;
    }

    if (len < 8) {
      continue; /* we only handle 8-byte frames here */
    }

    if (id == (WS_BASE + 0x22U)) {                /* Bus measurement: [A],[V] */
      float bus_current_A = unpack_f32_le(&data[0]);
      float bus_voltage_V = unpack_f32_le(&data[4]);
      printf("bus,%.3f,%.3f\n", bus_current_A, bus_voltage_V);

    } else if (id == (WS_BASE + 0x23U)) {         /* Velocity: [m/s],[rpm] */
      float vehicle_mps = unpack_f32_le(&data[0]);
      float motor_rpm   = unpack_f32_le(&data[4]);
      printf("vel,%.3f,%.3f\n", vehicle_mps, motor_rpm);

    } else if (id == (WS_BASE + 0x2BU)) {         /* Temperatures: [°C],[°C] */
      float heatsink_C = unpack_f32_le(&data[0]);
      float motor_C    = unpack_f32_le(&data[4]);
      printf("temps,%.2f,%.2f\n", heatsink_C, motor_C);

    } else if (id == (WS_BASE + 0x21U)) {         /* Status (integers) */
      uint8_t  rxErr       = data[0];
      uint8_t  txErr       = data[1];
      uint16_t activeMotor = (uint16_t)(data[2] | (data[3] << 8));
      uint16_t errFlags    = (uint16_t)(data[4] | (data[5] << 8));
      uint16_t limFlags    = (uint16_t)(data[6] | (data[7] << 8));
      printf("status,%u,%u,%u,%u,%u\n",
             rxErr, txErr, activeMotor, errFlags, limFlags);

    } else if (id == 0x405U || id == (WS_BASE + 0x05U)) {  /* Voltage vector */
      float Vd = unpack_f32_le(&data[0]);   /* real component (V)   */
      float Vq = unpack_f32_le(&data[4]);   /* imaginary component (V) */
      printf("vvec,%.3f,%.3f\n", Vd, Vq);

    } else if (id == 0x7F1U) {                       /* Bootloader heartbeat */
      printf("boot,1\n");

    } else {
      /* Numeric-only fallback for anything unexpected (ID in decimal) */
      printf("raw,%lu,%u,%u,%u,%u,%u,%u,%u,%u\n",
             (unsigned long)id,
             data[0], data[1], data[2], data[3],
             data[4], data[5], data[6], data[7]);
    }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL1Q;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FDCAN2_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_ICACHE_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
#ifdef MCP2562_STBY_GPIO_Port
  HAL_GPIO_WritePin(MCP2562_STBY_GPIO_Port, MCP2562_STBY_Pin, GPIO_PIN_RESET);
#endif

  /* === Configure a simple filter for WS22 device group and start FDCAN ==== */
  {
    FDCAN_FilterTypeDef filter = {0};
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = 0;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = (uint32_t)WS_BASE;   /* e.g., 0x420 */
    filter.FilterID2    = 0x7E0U;              /* mask device bits 10..5 */
    if (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) != HAL_OK) {
      Error_Handler();
    }

    /* Debug: accept ALL non-matching standard IDs into RX FIFO0, reject extended.
   Also accept remote frames (std/ext). */
if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
      FDCAN_ACCEPT_IN_RX_FIFO0,   /* non-matching standard IDs -> FIFO0 */
      FDCAN_REJECT,               /* non-matching extended IDs -> reject */
      FDCAN_FILTER_REMOTE,        /* accept std remote frames */
      FDCAN_FILTER_REMOTE) != HAL_OK) {  /* accept ext remote frames */
  Error_Handler();
}


    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
      Error_Handler();
    }
  }

  uint32_t last_led_ms = 0;
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

/* Infinite loop */
printf("Main loop start\n");
while (1)
{
  uint32_t now = HAL_GetTick();

  /* Request Status every 250 ms, Bus measurement every 100 ms */
  if ((now - t_status) >= 250U) {
    WS_RequestStatusOnce();
    t_status = now;
    /* You can comment the next line if you want zero hex/strings */
    // printf("[RTR] Request Status 0x%03lX\n", (unsigned long)(WS_BASE + 0x21U));
  }
  if ((now - t_bus) >= 100U) {
    WS_RequestBusOnce();
    t_bus = now;
    /* You can comment the next line if you want zero hex/strings */
    // printf("[RTR] Request Bus    0x%03lX\n", (unsigned long)(WS_BASE + 0x22U));
  }

  /* Poll and parse inbound frames (this is where id/data exist) */
  WS_ProcessRxOnce();

  /* Blink or status */
  if ((now - last_led_ms) >= 500U) {
    BSP_LED_Toggle(LED_GREEN);
    last_led_ms = now;
  }

  HAL_Delay(5);
}
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /* Configure the main internal regulator output voltage */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure. */

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV2;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2|RCC_CLOCKTYPE_PCLK3;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure the programming delay */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  /* Common config */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.SamplingMode = ADC_SAMPLING_MODE_NORMAL;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Regular Channel */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;

  /* 500 kbps timing example; ensure kernel clock suits this config */
  hfdcan2.Init.NominalPrescaler       = 10;   // 80/10 = 8 MHz tq (example)
  hfdcan2.Init.NominalSyncJumpWidth   = 2;
  hfdcan2.Init.NominalTimeSeg1        = 11;
  hfdcan2.Init.NominalTimeSeg2        = 2;

  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Analogue filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Analogue filter */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Digital filter */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{
  /* default */
}

/*
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT; /* left as-is */
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x7;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi2.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi2.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT; /* left as-is */
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x7;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  hspi4.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  hspi4.Init.ReadyPolarity = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
}

/*
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LO_10_GPIO_Port, LO_10_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LO_9_Pin|HO_2_Pin|LO_3_Pin|LO_2_Pin
                          |LO_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LO_6_Pin|LO_5_Pin|LO_8_Pin|LO_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LO_4B4_Pin|HO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OI_8_Pin OI_9_Pin OI_5_Pin OI_4_Pin
                           OI_7_Pin */
  GPIO_InitStruct.Pin = OI_8_Pin|OI_9_Pin|OI_5_Pin|OI_4_Pin
                          |OI_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OI_10_Pin */
  GPIO_InitStruct.Pin = OI_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OI_10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LO_10_Pin */
  GPIO_InitStruct.Pin = LO_10_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LO_10_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LO_9_Pin HO_2_Pin LO_3_Pin LO_2_Pin
                           LO_4_Pin */
  GPIO_InitStruct.Pin = LO_9_Pin|HO_2_Pin|LO_3_Pin|LO_2_Pin
                          |LO_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LO_6_Pin LO_5_Pin LO_8_Pin LO_7_Pin */
  GPIO_InitStruct.Pin = LO_6_Pin|LO_5_Pin|LO_8_Pin|LO_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OI_2_Pin OI_1_Pin */
  GPIO_InitStruct.Pin = OI_2_Pin|OI_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : OI_3_Pin */
  GPIO_InitStruct.Pin = OI_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OI_3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OI_6_Pin */
  GPIO_InitStruct.Pin = OI_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OI_6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LO_4B4_Pin HO_1_Pin */
  GPIO_InitStruct.Pin = LO_4B4_Pin|HO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/*
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();

  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/*
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */