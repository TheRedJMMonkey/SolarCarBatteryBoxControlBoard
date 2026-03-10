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
 * to receive WaveSculptor22 (WS22) telemetry, and transmits the Motor Drive
 * Command every 100 ms (keep-alive ≤ 250 ms). Payloads are two float32 values
 * (LSB-first) per WS22 Appendix C.
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
 *  - Two IEEE‑754 float32 values (LSB-first) per 8-byte frame.
 *  - Percent fields are 0.0–1.0 (not 0–100).
 *  - Must receive Motor Drive Command at least every 250 ms to stay in drive.
 * [1](https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html)
 *
 * Copyright (c) 2026.
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ================= User-configurable IDs (match your system) ===============
 */
#define DRIVER_BASE 0x400U /* Your STM32 node (Driver Controls)          */
#define WS_BASE 0x420U     /* WaveSculptor22 base address                 */
/* IDs per WS22 Appendix C: device field=bits10..5; message field=bits4..0.

 ================= Optional: MCP2562 STBY GPIO ============================= */
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
static uint32_t last_drive_tx_ms = 0;
static float throttle_0_to_1 =
    0.10f; /* example fixed throttle; map from ADC in real use */
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
static void WS_SendMotorDrive(float velocity_rpm, float motor_current_pct);
static void WS_ProcessRxOnce(void);
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

/* WaveSculptor Motor Drive Command (0x401): send every ≤ 250 ms (we do 100 ms)
   [1](https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html)
   payload: [0..3]=velocity_rpm (float), [4..7]=motor_current_pct (float, 0..1)
 */
static void WS_SendMotorDrive(float velocity_rpm, float motor_current_pct) {
  uint8_t payload[8];
  pack_f32_le(&payload[0], velocity_rpm);
  pack_f32_le(&payload[4], motor_current_pct);
  CAN_SendMessage(DRIVER_BASE + 0x01U, payload, 8);
}

/* Parse a subset of WS22 telemetry:
   0x422 Bus:   [A], [V]
   0x423 Vel:   [m/s], [rpm]
   0x42B Temps: [°C], [°C]
   0x421 Status: error/limit flags (U16/U16) — shown as comment.  */
static void WS_ProcessRxOnce(void) {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) > 0) {
    if (CAN_ReceiveMessage(&id, data, &len) != HAL_OK)
      break;

    // Print received CAN message
    printf("CAN RX: ID=0x%03X, Len=%d, Data=", (unsigned int)id, len);
    for (uint8_t i = 0; i < len; i++) {
      printf("%02X ", data[i]);
    }
    printf("\n");

    if (len < 8)
      continue;

    if (id == (WS_BASE + 0x22U)) { /* Bus measurement */
      float bus_current_A = unpack_f32_le(&data[0]);
      float bus_voltage_V = unpack_f32_le(&data[4]);
      printf("Bus: Current=%.2f A, Voltage=%.2f V\n", bus_current_A,
             bus_voltage_V);
    } else if (id == (WS_BASE + 0x23U)) { /* Velocity measurement */
      float vehicle_mps = unpack_f32_le(&data[0]);
      float motor_rpm = unpack_f32_le(&data[4]);
      printf("Velocity: Vehicle=%.2f m/s, Motor=%.2f rpm\n", vehicle_mps,
             motor_rpm);
    } else if (id == (WS_BASE + 0x2BU)) { /* Temps */
      float heatsink_C = unpack_f32_le(&data[0]);
      float motor_C = unpack_f32_le(&data[4]);
      printf("Temps: Heatsink=%.2f C, Motor=%.2f C\n", heatsink_C, motor_C);
    } else if (id == (WS_BASE + 0x21U)) { /* Status */
      /* Per spec, 0x421 packs RX/TX error counts (U8), Active Motor (U16),
         Error Flags (U16), Limit Flags (U16). Treat as integers if needed.
         [1](https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html)
         Example:
         uint32_t hi = *((uint32_t*)&data[0]);
         uint32_t lo = *((uint32_t*)&data[4]);
         uint16_t errFlags = (uint16_t)((lo >> 16) & 0xFFFF);
         uint16_t limFlags = (uint16_t)(lo & 0xFFFF);
      */
      printf("Status: Received\n");
    } else if (id == 0x7F1U) {
      /* WS22 bootloader heartbeat after reset @ 500 kbps. Useful sanity check.
       */
      printf("Bootloader Heartbeat: 0x7F1\n");
    } else {
      // Handle other frames if enabled (Id/Iq, Vd/Vq, odometer/Ah, etc.)
    }
  }
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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

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
  /* If MCP2562 STBY is on a GPIO, ensure it is driven LOW for Normal mode.
   * [2](https://ww1.microchip.com/downloads/en/DeviceDoc/20005167C.pdf) */
#ifdef MCP2562_STBY_GPIO_Port
  HAL_GPIO_WritePin(MCP2562_STBY_GPIO_Port, MCP2562_STBY_Pin, GPIO_PIN_RESET);
#endif
  uint32_t last_led_ms = 0;
  /* USER CODE END 2 */

  /* Initialize leds */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time
   * it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity
   */
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
  while (1) {
    printf("Main loop alive. Tick=%lu ms\n", (unsigned long)HAL_GetTick());
    HAL_Delay(500);
    uint32_t now = HAL_GetTick();

    /* Send Motor Drive Command every 100 ms (keep-alive ≤ 250 ms).
       [1](https://docs.prohelion.com/Motor_Controllers/WaveSculptor22/User_Manual/Appendix_C.html)
       Torque mode example: velocity= +/-20000 rpm, current = 0..1 (pedal).  */
    if ((now - last_drive_tx_ms) >= 100U) {
      last_drive_tx_ms = now;
      WS_SendMotorDrive(20000.0f, throttle_0_to_1); /* forward torque */
      // WS_SendMotorDrive(-20000.0f, throttle_0_to_1 );/* reverse torque (if
      // needed) */
    }

    /* Process inbound WS22 telemetry */
    WS_ProcessRxOnce();

    /* Blink LED @ 1 Hz just to show main loop alive */
    if ((now - last_led_ms) >= 500U) {
      last_led_ms = now;
      BSP_LED_Toggle(LED_GREEN);
    }

    /* TODO: Read ADC pedal → throttle_0_to_1, clamp to 0..1 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType =
      RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_CSI;
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                                RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure the programming delay
   */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_0);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
   */
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  hfdcan2.Init.NominalPrescaler = 16;
  hfdcan2.Init.NominalSyncJumpWidth = 1;
  hfdcan2.Init.NominalTimeSeg1 = 6;
  hfdcan2.Init.NominalTimeSeg2 = 1;
  hfdcan2.Init.DataPrescaler = 1;
  hfdcan2.Init.DataSyncJumpWidth = 1;
  hfdcan2.Init.DataTimeSeg1 = 1;
  hfdcan2.Init.DataTimeSeg2 = 1;
  hfdcan2.Init.StdFiltersNbr = 0;
  hfdcan2.Init.ExtFiltersNbr = 0;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Analogue filter
   */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  /** Configure Digital filter
   */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void) {

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */
}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_4BIT;
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
  if (HAL_SPI_Init(&hspi2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */
}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void) {

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_4BIT;
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
  if (HAL_SPI_Init(&hspi4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
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
  HAL_GPIO_WritePin(GPIOC, LO_9_Pin | HO_2_Pin | LO_3_Pin | LO_2_Pin | LO_4_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LO_6_Pin | LO_5_Pin | LO_8_Pin | LO_7_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LO_4B4_Pin | HO_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OI_8_Pin OI_9_Pin OI_5_Pin OI_4_Pin
                           OI_7_Pin */
  GPIO_InitStruct.Pin = OI_8_Pin | OI_9_Pin | OI_5_Pin | OI_4_Pin | OI_7_Pin;
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
  GPIO_InitStruct.Pin = LO_9_Pin | HO_2_Pin | LO_3_Pin | LO_2_Pin | LO_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LO_6_Pin LO_5_Pin LO_8_Pin LO_7_Pin */
  GPIO_InitStruct.Pin = LO_6_Pin | LO_5_Pin | LO_8_Pin | LO_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OI_2_Pin OI_1_Pin */
  GPIO_InitStruct.Pin = OI_2_Pin | OI_1_Pin;
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
  GPIO_InitStruct.Pin = LO_4B4_Pin | HO_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
