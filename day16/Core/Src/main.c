/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Day16 - 自作ドライバによる FOC（電圧制御モード）
  *                   AS5600 で電気角を取得し、逆Park→逆Clarke→PWM で駆動
  *                   ※電流PI制御は入れない（Day17で追加）
  *                   ※ゼロ相注入なし = SPWM（Day17でSVPWM化）
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* --- ADC / 電流センス（Day15 から継続。今回はモニタ用途のみ） --- */
#define ADC_REF_V      3.3f
#define ADC_RESOLUTION 4095.0f
#define CSA_GAIN_VA    1.0f     // DRV8311H GAIN Hi-Z = 1 V/A

/* --- PWM --- */
#define PWM_PERIOD     4250.0f  // ARR値 (20kHz Center Aligned + PWM Mode 2)
#define DUTY_MAX       0.95f    // ローサイドON期間を 2.5us 以上残すための上限
#define DUTY_MIN       0.05f

/* --- モータ / 電源 --- */
#define POLE_PAIRS     7.0f     // GBM2804: 7極対
#define VBUS_V        12.0f     // 電源電圧 [V]（相電圧の線形範囲は ±VBUS_V/2）

/* --- 角度センサ AS5600 --- */
#define AS5600_ADDR   (0x36 << 1) // 7bitアドレス 0x36 → 8bit形式
#define AS5600_RAW_ANGLE 0x0C     // RAW_ANGLE レジスタ（12bit）
#define AS5600_CPR     4096.0f    // 12bit = 4096分割

// -----------------------------------------------------------------------
// エンコーダの計数方向がモータ回転方向と逆の場合は 1 にする。
// アライメントとメインループの両方に同じ反転が掛かる（片方だけだとズレる）。
// -----------------------------------------------------------------------
#define INVERT_ENCODER_DIR  1

// -----------------------------------------------------------------------
// ★ q軸電圧指令 [V]。まず 1.0〜3.0V から始める。
//   SPWM の線形上限は相電圧振幅 VBUS_V/2 = 6.0V。
// -----------------------------------------------------------------------
#define TARGET_VOLTAGE  2.0f

#define LOOP_LOG_DECIM  200     // ログ出力の間引き（ループ何回に1回出すか）
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */
/* 電流モニタ（ADC ISR が更新。今回は制御に使わずログ表示のみ） */
volatile float g_Iu = 0.0f, g_Iv = 0.0f, g_Iw = 0.0f;
volatile uint32_t g_debug_cnt = 0;

float offset_u = 2048.0f;
float offset_v = 2048.0f;
float offset_w = 2048.0f;

/* FOC 用の角度 */
float Theta_mech = 0.0f;        // 機械角 [rad]（方向反転適用後）
float Theta_elec = 0.0f;        // 電気角 [rad]
float zero_offset_mech = 0.0f;  // アライメントで求める機械角ゼロ点 [rad]
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)ptr, len, 100);
    return len;
}

/* 角度を [0, 2pi) に正規化 */
static float normalize_angle(float angle)
{
    float a = fmodf(angle, 2.0f * (float)M_PI);
    return (a < 0.0f) ? a + 2.0f * (float)M_PI : a;
}

/*
 * AS5600 の RAW_ANGLE を読んで機械角 [rad] を返す。
 * I2C は約 100kHz なので 2 バイト読み出しに ~450us かかる。
 * このブロッキング時間がそのまま制御周期の下限になる（約 2.2kHz）。
 */
static float AS5600_GetAngle(void)
{
    uint8_t raw[2];
    if (HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDR, AS5600_RAW_ANGLE,
                         I2C_MEMADD_SIZE_8BIT, raw, 2, 10) != HAL_OK) {
        return Theta_mech;   // 読み出し失敗時は前回値を維持
    }
    uint16_t val = (uint16_t)(((uint16_t)raw[0] << 8) | raw[1]) & 0x0FFF;
    float mech = (float)val * (2.0f * (float)M_PI / AS5600_CPR);

#if INVERT_ENCODER_DIR
    mech = 2.0f * (float)M_PI - mech;
#endif
    return normalize_angle(mech);
}

/* 全相 Duty 0% にする。PWM Mode 2 では CCR = ARR が Duty 0%。 */
static void PWM_SetAllOff(void)
{
    TIM1->CCR1 = (uint32_t)PWM_PERIOD;
    TIM1->CCR2 = (uint32_t)PWM_PERIOD;
    TIM1->CCR3 = (uint32_t)PWM_PERIOD;
}

static float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
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

  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();

  /* USER CODE BEGIN 2 */
  printf("=== Day16: FOC Voltage Control Mode ===\r\n");

  // ------------------------------------------------------------------
  // ★重要: PWM 開始前に必ず Duty 0% を書く。
  //   MX_TIM1_Init() 直後の CCR は 0 で、PWM Mode 2 では CCR=0 が
  //   Duty 100%（フルパワー）になるため、先に CCR=ARR を入れる。
  // ------------------------------------------------------------------
  PWM_SetAllOff();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // --- 1. 電流センサのゼロ点オフセット計測（全相OFF状態で行う） ---
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  printf("Calibrating current offsets...\r\n");
  HAL_ADCEx_InjectedStart(&hadc1);
  long sum_u = 0, sum_v = 0, sum_w = 0;
  for (int i = 0; i < 1000; i++) {
      HAL_ADCEx_InjectedPollForConversion(&hadc1, 10);
      sum_u += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
      sum_v += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
      sum_w += HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);
  }
  HAL_ADCEx_InjectedStop(&hadc1);
  offset_u = (float)sum_u / 1000.0f;
  offset_v = (float)sum_v / 1000.0f;
  offset_w = (float)sum_w / 1000.0f;
  printf("Offsets: U=%.1f V=%.1f W=%.1f\r\n", offset_u, offset_v, offset_w);

  // 割り込みモードで ADC 開始 (TIM1 TRGO2 自動トリガ)
  HAL_ADCEx_InjectedStart_IT(&hadc1);

  // ------------------------------------------------------------------
  // --- 2. 電気角アライメント（エンコーダ0点と磁極0点の同期） ---
  //   U相にのみ電圧を印加して電気角0度相当の磁界を作り、
  //   ローターが引き寄せられて静止した位置をゼロ点として記録する。
  //   PWM Mode 2 なので CCR 小 = Duty 大。
  // ------------------------------------------------------------------
  printf("Aligning encoder...\r\n");
  TIM1->CCR1 = (uint32_t)(PWM_PERIOD * 0.5f);   // U相 Duty 50%
  TIM1->CCR2 = (uint32_t)PWM_PERIOD;            // V相 Duty  0%
  TIM1->CCR3 = (uint32_t)PWM_PERIOD;            // W相 Duty  0%
  HAL_Delay(1000);                              // ローターが落ち着くのを待つ

  zero_offset_mech = AS5600_GetAngle();
  printf("Encoder offset: %.3f rad\r\n", zero_offset_mech);

  PWM_SetAllOff();
  HAL_Delay(500);

  printf("Start FOC (Vq = %.1f V)\r\n", TARGET_VOLTAGE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  static uint32_t log_cnt = 0;

  while (1)
  {
      /* --- 1. 機械角を取得（AS5600 / I2C ブロッキング ~450us） --- */
      Theta_mech = AS5600_GetAngle();

      /* --- 2. 電気角へ変換: theta_e = Pn * (theta_m - theta_m0) --- */
      Theta_elec = normalize_angle((Theta_mech - zero_offset_mech) * POLE_PAIRS);

      /* --- 3. 電圧ベクトル指令 ---
       * 電流PI制御を入れないので Vq を定数で与える（電圧制御モード）。
       * SPMモータはリラクタンストルクがないため Vd = 0。
       */
      float Vd = 0.0f;
      float Vq = TARGET_VOLTAGE;

      /* --- 4. 逆Park変換 (dq -> alpha,beta) --- */
      float c = cosf(Theta_elec);
      float s = sinf(Theta_elec);
      float Valpha = Vd * c - Vq * s;
      float Vbeta  = Vd * s + Vq * c;

      /* --- 5. 逆Clarke変換 (alpha,beta -> U,V,W) ---
       * ここでゼロ相成分を注入していないので SPWM。
       * SVPWM 化する場合は v_zero = -0.5*(max+min) を3相に加算する（Day17）。
       */
      float Vu_ref = Valpha;
      float Vv_ref = (-0.5f * Valpha) + (0.8660254f * Vbeta);   // sqrt(3)/2
      float Vw_ref = (-0.5f * Valpha) - (0.8660254f * Vbeta);

      /* --- 6. 相電圧 -> Duty比 ---
       * Duty 0.5 を DCリンク中点とする対応。相電圧の線形範囲は ±VBUS_V/2。
       */
      float du = clampf(Vu_ref / VBUS_V + 0.5f, DUTY_MIN, DUTY_MAX);
      float dv = clampf(Vv_ref / VBUS_V + 0.5f, DUTY_MIN, DUTY_MAX);
      float dw = clampf(Vw_ref / VBUS_V + 0.5f, DUTY_MIN, DUTY_MAX);

      /* --- 7. PWM出力（Mode 2 なので CCR = (1 - duty) * ARR） --- */
      TIM1->CCR1 = (uint32_t)((1.0f - du) * PWM_PERIOD);
      TIM1->CCR2 = (uint32_t)((1.0f - dv) * PWM_PERIOD);
      TIM1->CCR3 = (uint32_t)((1.0f - dw) * PWM_PERIOD);

      /* --- 8. ログ出力（CSV 2列: 電気角, U相Duty）---
       * printf はブロッキングなので毎回出すと制御周期が伸びる。間引く。
       */
      if (++log_cnt >= LOOP_LOG_DECIM) {
          printf("%.4f,%.4f\r\n", Theta_elec, du);
          log_cnt = 0;
      }
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  // Rank1: CH1 (PA0 / SO_A)
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  // Rank2: CH2 (PA1 / SO_B)
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  // Rank3: CH5 (PB14 / SO_C)
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x30A0A7FB;
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
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief LPUART1 Initialization Function
  */
static void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4250;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 1;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance != ADC1) return;

    // タイミング検証: 谷底(CNT=0)でトリガされていれば小さい値になる
    g_debug_cnt = TIM1->CNT;

    // 生ADC値取得
    uint32_t raw_u = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
    uint32_t raw_v = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_2);
    uint32_t raw_w = HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_3);

    // ADCカウント→電圧→電流[A]
    // I = (V_SO - V_offset) / CSA_GAIN_VA
    // オフセットはキャリブレーション値(ADCカウント)で補正
    g_Iu = ((float)raw_u - offset_u) * (ADC_REF_V / ADC_RESOLUTION) / CSA_GAIN_VA;
    g_Iv = ((float)raw_v - offset_v) * (ADC_REF_V / ADC_RESOLUTION) / CSA_GAIN_VA;
    g_Iw = ((float)raw_w - offset_w) * (ADC_REF_V / ADC_RESOLUTION) / CSA_GAIN_VA;

    // LD2 LED トグル (オシロでトリガタイミング確認用)
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
