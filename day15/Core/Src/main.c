/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Day15 - ADC同期サンプリング検証
  *                   DRV8311H 3相電流センシング（タイミング確認）
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
#define ADC_REF_V      3.3f
#define ADC_RESOLUTION 4095.0f
#define CSA_GAIN_VA    1.0f     // DRV8311H GAIN Hi-Z = 1 V/A
#define PWM_PERIOD     4250.0f  // ARR値 (20kHz Center Aligned)

// -----------------------------------------------------------------------
// モータ駆動モード切替
// コメントアウト → ADC タイミング検証のみ（モータ未接続で OK）
// コメントを外す → オープンループ SVPWM 駆動（モータ接続必須）
// -----------------------------------------------------------------------
// #define ENABLE_MOTOR_DRIVE

#ifdef ENABLE_MOTOR_DRIVE
#define OPEN_LOOP_VOLTAGE 0.3f // オープンループ出力電圧振幅 [0.0~1.0]
                               // ※ FOC の Vq ではなく正弦波指令の振幅
#define ANGLE_STEP     0.02f   // 1ループあたりの角度増分 [rad]
#define LOOP_DELAY_MS  2       // ループ周期 [ms]
#else
#define LOOP_DELAY_MS  100     // タイミング検証モードの表示間隔 [ms]
#endif
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
volatile float g_Iu = 0.0f, g_Iv = 0.0f, g_Iw = 0.0f;
volatile uint32_t g_debug_cnt = 0;

float offset_u = 2048.0f;
float offset_v = 2048.0f;
float offset_w = 2048.0f;

#ifdef ENABLE_MOTOR_DRIVE
float g_angle = 0.0f;
#endif
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
  printf("=== Day15: ADC Sync Sampling Verification ===\r\n");

  // PWM 50%固定で起動 (Mode2: CCR=ARR/2 で50%)
  TIM1->CCR1 = (uint32_t)(PWM_PERIOD * 0.5f);
  TIM1->CCR2 = (uint32_t)(PWM_PERIOD * 0.5f);
  TIM1->CCR3 = (uint32_t)(PWM_PERIOD * 0.5f);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  // ADCキャリブレーション
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  // ゼロ電流オフセット計測 (1000サンプル平均)
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
  printf("ADC started. Monitoring...\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef ENABLE_MOTOR_DRIVE
      // --- オープンループ SVPWM 駆動 ---
      // Va/Vb/Vc を正弦波で生成し、common-mode injection（SVPWM 零相補正）を適用
      float Va = OPEN_LOOP_VOLTAGE * sinf(g_angle);
      float Vb = OPEN_LOOP_VOLTAGE * sinf(g_angle - 2.0f * (float)M_PI / 3.0f);
      float Vc = OPEN_LOOP_VOLTAGE * sinf(g_angle + 2.0f * (float)M_PI / 3.0f);

      float v_max = fmaxf(fmaxf(Va, Vb), Vc);
      float v_min = fminf(fminf(Va, Vb), Vc);
      float cm = (v_max + v_min) * 0.5f;   // common-mode 成分

      // PWM Mode 2: duty_HIGH = (ARR - CCR) / ARR なので Va 正 → CCR 小（反転）
      // CCR = (1 - (V - cm)) / 2 * ARR
      TIM1->CCR1 = (uint32_t)((1.0f - Va + cm) * 0.5f * PWM_PERIOD);
      TIM1->CCR2 = (uint32_t)((1.0f - Vb + cm) * 0.5f * PWM_PERIOD);
      TIM1->CCR3 = (uint32_t)((1.0f - Vc + cm) * 0.5f * PWM_PERIOD);

      g_angle += ANGLE_STEP;
      if (g_angle >= 2.0f * (float)M_PI) g_angle -= 2.0f * (float)M_PI;

      // CSV 出力（カウンタ付き4列）: count,Iu,Iv,Iw
      static uint32_t s_cnt = 0;
      printf("%lu,%.4f,%.4f,%.4f\r\n", s_cnt++, g_Iu, g_Iv, g_Iw);
#else
      // --- ADC タイミング検証モード（モータ未接続） ---
      printf("Iu=%6.3f Iv=%6.3f Iw=%6.3f CNT=%lu\r\n",
             g_Iu, g_Iv, g_Iw, g_debug_cnt);
#endif
      HAL_Delay(LOOP_DELAY_MS);
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
