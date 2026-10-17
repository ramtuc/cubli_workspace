/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
// --- ハードウェア定数 ---
#define POLE_PAIRS 7.0f       // GBM2804: 7極対
#define V_REF 12.0f           // 電源電圧 12V
#define PWM_PERIOD 4250.0f    // ARR値
#define DRV8311_GAIN 1.0f     // 電流センスゲイン [V/A]

// --- I2Cアドレス (AS5600) ---
#define AS5600_ADDR (0x36 << 1)

// --- 制御ゲイン (Day 17設定) ---
#define KP_GAIN 1.0f          // Pゲイン: 実機で 0.6 から引き上げ
#define KI_GAIN 1.3f         // Iゲイン: 積分で誤差を潰す
#define IQ_TARGET 0.6f        // 定数トルク指令で試す場合の目標電流 [A]
                              // ※バネ動作時は未使用（Iq_ref を計算で求める）

// 電圧制限 (12V電源の半分くらいを上限に、過剰な電圧を防ぐ)
#define VOLTAGE_LIMIT 6.0f

#define CONTROL_DT  (1.0f / 20000.0f)   // 制御周期 (50us)
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
// --- FOC用 変数 ---
// 1. センサ値
float Theta_mech = 0.0f;      // 機械角 [rad]
float Theta_elec = 0.0f;      // 電気角 [rad]
float zero_offset_mech = 0.0f; // アライメントオフセット

// 2. 電流値 (ADC & 変換後)
// volatileをつけることでISRとMainでの共有を安全にする
volatile float g_Iu = 0.0f, g_Iv = 0.0f, g_Iw = 0.0f;
volatile float g_Id = 0.0f, g_Iq = 0.0f;

// 3. 角度共有用
volatile float g_theta_elec = 0.0f;
float g_total_angle = 0.0f;  // これが「何回転したか」を含む総角度
float prev_mech_angle = 0.0f; // 前回の角度を覚えておく

// 4. PI制御用
float Id_err_int = 0.0f;      // d軸積分項
float Iq_err_int = 0.0f;      // q軸積分項

// 5. 出力Duty (ログ用)
volatile float g_duty_u = 0.5f, g_duty_v = 0.5f, g_duty_w = 0.5f;

// 6. キャリブレーション (0A基準値)
float offset_u = 2048.0f;
float offset_v = 2048.0f;
float offset_w = 2048.0f;

// 制御有効フラグ
volatile uint8_t foc_enable = 0;

// --- サンプリング位置の検証用（制御には使わない。確認後は削除可） ---
// ISR が走った時点の TIM1 カウンタと計数方向を記録する。
// TRGO2=OC4REF / CCR4=2125 の場合、理論上は谷底の 12.5us 手前
// （ダウンカウント中の CNT≈2124）でトリガされるはず。
volatile uint32_t g_debug_cnt = 0;
volatile uint8_t  g_debug_dir = 0;   // 0 = アップカウント, 1 = ダウンカウント
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
// printf用
int _write(int file, char *ptr, int len) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)ptr, len, 100);
  return len;
}

// AS5600 角度取得
float AS5600_GetAngle(void) {
    uint8_t raw[2];
    HAL_I2C_Mem_Read(&hi2c2, AS5600_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT, raw, 2, 10);
    uint16_t val = ((uint16_t)raw[0] << 8) | raw[1];
    return (float)val * (2.0f * M_PI / 4096.0f);
}

// 角度正規化
float normalize_angle(float angle) {
    float a = fmod(angle, 2.0f * M_PI);
    return (a < 0.0f) ? a + 2.0f * M_PI : a;
}

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  SystemCoreClockUpdate();
    DWT_Init();

    printf("=== Day 17: Current Control Mode Start ===\r\n");

    // ADCトリガ用。CH4 は PWM Mode 1 なので、ダウンカウント中に
    // CNT が CCR4 を下回った瞬間に TRGO2 が立ち上がる。
    // CCR4=200 → 谷底の約 1.2us 手前でトリガし、3相の変換ウィンドウが
    // 谷底をまたぐ形になる（ローサイド ON 期間を外さないため）。
    //   旧: PWM_PERIOD*0.5f = 2125 → 谷底の 12.5us 手前（Duty>50% の相で
    //       ハイサイド ON のまま読んでいた）
    TIM1->CCR4 = 200;

    // PWM開始
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); // ADCトリガ用

    // ADCキャリブレーション & 開始
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    // 割り込みはまだ有効にしない(enableフラグでガード)
    foc_enable = 0;
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    // --- 1. オフセットキャリブレーション (電流センサ0点) ---
    printf("Calibrating Current Offsets...\r\n");
    // 全相LOW (Mode2なのでCCR=ARR)
    TIM1->CCR1 = 4250; TIM1->CCR2 = 4250; TIM1->CCR3 = 4250;
    HAL_Delay(500);

    long sum_u = 0, sum_v = 0, sum_w = 0;
    for(int i=0; i<1000; i++) {
      sum_u += ADC1->JDR1;
      sum_v += ADC1->JDR2;
      sum_w += ADC1->JDR3;
      HAL_Delay(1);
    }
    offset_u = (float)sum_u / 1000.0f;
    offset_v = (float)sum_v / 1000.0f;
    offset_w = (float)sum_w / 1000.0f;
    printf("Offsets: U=%.1f, V=%.1f, W=%.1f\r\n", offset_u, offset_v, offset_w);

    // --- 2. エンコーダアライメント (電気角0点合わせ) ---
    printf("Aligning Encoder...\r\n");
    // U相に電圧印加 (Mode2: CCR小 = Duty大)
    // Duty = (ARR - CCR) / ARR
    TIM1->CCR1 = 500;  // Duty 88.2% … ローターを確実に引き込むため Day16 より強め
    TIM1->CCR2 = 4250; // Duty  0%
    TIM1->CCR3 = 4250; // Duty  0%
    HAL_Delay(1000);

    float raw = AS5600_GetAngle();
    // ★重要: 前回成功した「逆回転ロジック」を維持
    //zero_offset_mech = 2.0f * M_PI - raw;
    //if(zero_offset_mech < 0) zero_offset_mech += 2.0f*M_PI;
    //if(zero_offset_mech > 2.0f*M_PI) zero_offset_mech -= 2.0f*M_PI;
    zero_offset_mech = raw;
    printf("Encoder Offset: %.2f rad\r\n", zero_offset_mech);

    // 電圧オフ
    TIM1->CCR1 = 4250; TIM1->CCR2 = 4250; TIM1->CCR3 = 4250;
    HAL_Delay(500);
    prev_mech_angle = AS5600_GetAngle(); // 逆回転対応不要ならそのまま
      g_total_angle = 0.0f; // ここを基準点(0)にする
    // --- 制御開始 ---
    printf("Start Closed Loop Control!\r\n");
    foc_enable = 1; // これでISR内の制御が走り出す
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    int count = 0;
    while (1)
      {
    	float raw = AS5600_GetAngle();
    	      Theta_mech = raw; // 素直な値

    	      // 電気角計算 (FOC用)
    	      Theta_elec = normalize_angle((Theta_mech - zero_offset_mech) * POLE_PAIRS);
    	      g_theta_elec = Theta_elec; // ISRに渡す

    	      // ★追加: 累積角度の計算 (マルチターン対応)
    	      float delta = Theta_mech - prev_mech_angle;

    	      // 「0 ⇔ 2π」の継ぎ目またぎ検知
    	      if (delta > M_PI)  delta -= 2.0f * M_PI; // 0 -> 360 をまたいだ(逆回転)
    	      if (delta < -M_PI) delta += 2.0f * M_PI; // 360 -> 0 をまたいだ(正回転)

    	      g_total_angle += delta; // 変化量を足し込む
    	      prev_mech_angle = Theta_mech; // 次回用に保存

          // 2. ログ出力 (低頻度で)
          count++;
          if(count >= 50) { // ループ速度によるが適度に間引く
              // ISRで計算された電流値やDutyを表示
              // CNT/DIR はサンプリング位置の検証用（確認後は削除可）
              printf("Th:%.2f Id:%.2f Iq:%.2f du:%.2f CNT:%lu DIR:%u\r\n",
                      g_theta_elec, g_Id, g_Iq, g_duty_u,
                      (unsigned long)g_debug_cnt, (unsigned)g_debug_dir);
              count = 0;
          }
          // Delayは入れない（角度更新を早くするため）
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
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

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
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

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_5;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
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

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
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
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // ADC1以外からの呼び出しや、FOC無効時は何もしない
    if (hadc->Instance != ADC1 || !foc_enable) return;

    // サンプリング位置の検証用（最優先で読む。制御には影響しない）
    g_debug_cnt = TIM1->CNT;
    g_debug_dir = (TIM1->CR1 & TIM_CR1_DIR) ? 1 : 0;

    // 1. 電流取得 & [A]変換

     float Iu_l = ((float)ADC1->JDR1 - offset_u) * (3.3f / 4095.0f) / DRV8311_GAIN;
     float Iv_l = ((float)ADC1->JDR2 - offset_v) * (3.3f / 4095.0f) / DRV8311_GAIN;
     float Iw_l = ((float)ADC1->JDR3 - offset_w) * (3.3f / 4095.0f) / DRV8311_GAIN;

    //float Iu_l = (offset_u - (float)ADC1->JDR1) * (3.3f / 4095.0f) / DRV8311_GAIN;
    //float Iv_l = (offset_v - (float)ADC1->JDR2) * (3.3f / 4095.0f) / DRV8311_GAIN;
    //float Iw_l = (offset_w - (float)ADC1->JDR3) * (3.3f / 4095.0f) / DRV8311_GAIN;
    // オフセット微調整 (3相合計が0になるように)
    float Iavg = (Iu_l + Iv_l + Iw_l) / 3.0f;
    Iu_l -= Iavg;
    Iv_l -= Iavg;
    Iw_l -= Iavg;

    // グローバル変数へ反映(ログ用)
    g_Iu = Iu_l; g_Iv = Iv_l; g_Iw = Iw_l;

    // 2. 座標変換 (Clarke & Park)
    // メインループで更新された最新の角度を使う
    float th = g_theta_elec;
    float c = cosf(th);
    float s = sinf(th);

    float I_alpha = Iu_l;
    float I_beta  = (Iu_l + 2.0f * Iv_l) * 0.57735f; // 1/sqrt(3)

    float Id_l =  I_alpha * c + I_beta * s;
    float Iq_l = -I_alpha * s + I_beta * c;

    g_Id = Id_l; g_Iq = Iq_l;

    // ① 目標は「累積角度 0」 (起動した場所)
        float target = 0.0f;

        // ② ズレを計算 (単純な引き算)
        // ★重要: ここで if(> PI)... のような正規化をしてはいけません！
        // 遠くに行けば行くほど値を大きくするためです。
        float angle_err = target - g_total_angle;

        // ③ バネ定数 (K)
        float spring_k = 0.5f; // 好みで調整

        // ④ 目標電流
        float Iq_ref = spring_k * angle_err;

        // ★重要: 安全リミットを広げる
        // 遠くまで回した時に強い力を出すため、モーター定格付近まで許可
        if (Iq_ref > 1.5f) Iq_ref = 1.5f;
        if (Iq_ref < -1.5f) Iq_ref = -1.5f;

        // ===== ここから下は「電流PI制御」 (目標に追従させる処理) =====

        // --- d軸 (目標 0A: 常に0にする) ---
        float Id_err = 0.0f - Id_l;
        Id_err_int += Id_err * CONTROL_DT;
        // ... (d軸の積分リミットや計算はそのまま) ...
        float limit_i = VOLTAGE_LIMIT / KI_GAIN; // 定義がない場合は直接数値を書いてもOK
        Id_err_int = clampf(Id_err_int, -limit_i, limit_i);
        float Vd_l = KP_GAIN * Id_err + KI_GAIN * Id_err_int;


        // --- q軸 (目標 Iq_ref: さっき計算したバネの力) ---
        float Iq_err = Iq_ref - Iq_l; // 上で作った Iq_ref を使用
        // ★ここを変更: IQ_TARGET ではなく、計算した Iq_ref を使う
        //float Iq_err = Iq_ref - Iq_l;

        Iq_err_int += Iq_err * CONTROL_DT;
        Iq_err_int = clampf(Iq_err_int, -limit_i, limit_i);

        float Vq_l = KP_GAIN * Iq_err + KI_GAIN * Iq_err_int;

    // 4. 逆変換 & SVPWM
    float Valpha = Vd_l * c - Vq_l * s;
    float Vbeta  = Vd_l * s + Vq_l * c;

    float U_ref = Valpha;
    float V_ref = (-0.5f * Valpha) + (0.866f * Vbeta);
    float W_ref = (-0.5f * Valpha) - (0.866f * Vbeta);

    // ゼロ相注入 (SVPWM)
    float v_max = U_ref;
    if(V_ref > v_max) v_max = V_ref;
    if(W_ref > v_max) v_max = W_ref;

    float v_min = U_ref;
    if(V_ref < v_min) v_min = V_ref;
    if(W_ref < v_min) v_min = W_ref;

    float v_zero = -0.5f * (v_max + v_min);
    U_ref += v_zero;
    V_ref += v_zero;
    W_ref += v_zero;

    // 5. Duty計算 & PWM更新
    float du = (U_ref / V_REF) + 0.5f;
    float dv = (V_ref / V_REF) + 0.5f;
    float dw = (W_ref / V_REF) + 0.5f;

    du = clampf(du, 0.05f, 0.95f);
    dv = clampf(dv, 0.05f, 0.95f);
    dw = clampf(dw, 0.05f, 0.95f);

    g_duty_u = du; // ログ用

    // Mode 2 なので (1-duty)
    TIM1->CCR1 = (uint32_t)((1.0f - du) * PWM_PERIOD);
    TIM1->CCR2 = (uint32_t)((1.0f - dv) * PWM_PERIOD);
    TIM1->CCR3 = (uint32_t)((1.0f - dw) * PWM_PERIOD);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
