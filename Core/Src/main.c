/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h> // for atoi
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* === 사용자 튜닝 매크로 === */
#define RELAY_ACTIVE_HIGH        1       // 릴레이 High=ON이면 1, Low-트리거면 0
#define HUMID_REPORT_PERIOD_MS   2000U   // H:xx 전송 주기
#define CONTROL_PERIOD_MS        100U    // 제어/점검 주기
#define PUMP_MAX_ON_MS           10000U  // 펌프 최대 연속 구동 시간 (10초로 수정)
#define HUMIDITY_LOW_TH          35U     // 이하면 물 주기 시작
#define HUMIDITY_HIGH_TH         45U     // 이 이상이면 물 주기 종료(히스테리시스)
#define DEBOUNCE_MS              30U     // 플로트 스위치 디바운스(ms)

/* ADC 보정(Tera Term으로 실시간 튜닝 가능) */
#define ADC_WET_REF_DEFAULT      1090U   // 충분히 젖은 상태 ADC 평균값
#define ADC_DRY_REF_DEFAULT      4095U   // 마른 상태 ADC 평균값

/* PC 통신용 버퍼 */
#define PC_RX_BUF_SIZE 64
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline uint32_t now_ms(void){ return HAL_GetTick(); }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* 인터럽트 디바운싱 플래그 */
static volatile uint8_t  g_db_pending = 0;
static volatile uint8_t  g_db_target_empty = 0;
static volatile uint32_t g_db_due_ms = 0;

/* 현재 확정된 탱크 상태(EMPTY=물 부족) */
static bool g_tank_empty = false;

/* 타이밍 */
static uint32_t lastHumTx = 0;
static uint32_t lastCtrl  = 0;

/* 펌프 상태 보조 */
static uint32_t pump_on_ms = 0;

/* ADC 캘리브레이션 변수 */
static uint16_t g_adc_wet_ref = ADC_WET_REF_DEFAULT;
static uint16_t g_adc_dry_ref = ADC_DRY_REF_DEFAULT;
static uint16_t g_last_adc_raw = 0; // PC 보고용

/* PC UART 수신 관련 변수 */
static uint8_t  g_pc_rx_buf[PC_RX_BUF_SIZE];
static uint8_t  g_pc_rx_char;
static uint16_t g_pc_rx_idx = 0;
static volatile bool g_pc_cmd_received = false;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* 유틸/보조 함수 프로토타입 */
static void uart_send_str(const char *s);
static void pc_log(const char *s);
static void pump_set(bool on);
static bool pump_get(void);
static bool tank_empty_raw(void);
static uint16_t adc_read_average(uint8_t samples);
static uint8_t humidity_percent_from_adc(uint16_t adc);
static void send_H(uint8_t pct);
static void send_F_empty(void);
static void send_F_full(void);
static void send_W_ok(void);
static void process_pc_command(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ==== 구현부 ==== */
/* ESP32(USART1) + PC(USART2) 동시 송신 */
static void uart_send_str(const char *s){
    uint16_t len = (uint16_t)strlen(s);
    HAL_UART_Transmit(&huart1, (uint8_t*)s, len, 10);
    HAL_UART_Transmit(&huart2, (uint8_t*)s, len, 10);
}
/* PC 터미널(USART2) 전용 로그 */
static void pc_log(const char *s){
    HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 10);
}

static void pump_set(bool on){
#if RELAY_ACTIVE_HIGH
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

static bool pump_get(void){
#if RELAY_ACTIVE_HIGH
  return (HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin) == GPIO_PIN_SET);
#else
  return (HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin) == GPIO_PIN_RESET);
#endif
}

/* Pull-up: LOW면 EMPTY(물 부족) */
static bool tank_empty_raw(void){
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET);
}

/* ADC 다중 샘플 평균 */
static uint16_t adc_read_average(uint8_t samples){
  uint32_t acc = 0;
  for(uint8_t i=0;i<samples;i++){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    acc += HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    HAL_Delay(2);
  }
  return (uint16_t)(acc / samples);
}

/* ADC → % (선형 맵핑, 튜닝된 값 사용) */
static uint8_t humidity_percent_from_adc(uint16_t adc){
  int32_t wet = g_adc_wet_ref, dry = g_adc_dry_ref;
  if(dry == wet) return 0;
  int32_t pct = ((int32_t)dry - (int32_t)adc) * 100 / ((int32_t)dry - (int32_t)wet);
  if(pct < 0) pct = 0; if(pct > 100) pct = 100;
  return (uint8_t)pct;
}

/* 메시지 송신 */
static void send_H(uint8_t pct){
  if(pct > 99) pct = 99; // 두 자리 고정
  char buf[16];
  snprintf(buf, sizeof(buf), "H:%02u\r\n", pct);
  uart_send_str(buf);
}
static void send_F_empty(void){ uart_send_str("F:0\r\n"); } // 물 부족
static void send_F_full(void) { uart_send_str("F:1\r\n"); } // 물 충분 복구
static void send_W_ok(void)   { uart_send_str("W:OK\r\n"); }

/* EXTI 콜백: PB12 Rising/Falling */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(GPIO_Pin == GPIO_PIN_12){
    g_db_target_empty = tank_empty_raw() ? 1 : 0;
    g_db_due_ms = now_ms() + DEBOUNCE_MS;
    g_db_pending = 1;
  }
}

/* PC UART 수신 완료 콜백 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        if (g_pc_rx_idx < PC_RX_BUF_SIZE - 1) {
            if (g_pc_rx_char == '\n' || g_pc_rx_char == '\r') {
                if (g_pc_rx_idx > 0) { // 내용이 있을 때만 처리
                    g_pc_rx_buf[g_pc_rx_idx] = '\0';
                    g_pc_cmd_received = true;
                }
                g_pc_rx_idx = 0; // 인덱스 리셋
            } else {
                g_pc_rx_buf[g_pc_rx_idx++] = g_pc_rx_char;
            }
        } else { // 버퍼 오버플로우 방지
            g_pc_rx_idx = 0;
        }
        // 다음 1바이트 수신 대기
        HAL_UART_Receive_IT(&huart2, &g_pc_rx_char, 1);
    }
}

/* PC 명령어 처리 함수 */
void process_pc_command(void) {
    char buf[64];
    char* cmd = (char*)g_pc_rx_buf;

    if (strncmp(cmd, "SET_WET=", 8) == 0) {
        uint16_t val = (uint16_t)atoi(cmd + 8);
        if (val > 0 && val < 4096) {
            g_adc_wet_ref = val;
            snprintf(buf, sizeof(buf), "OK: WET reference set to %u\r\n", g_adc_wet_ref);
            pc_log(buf);
        } else {
            pc_log("ERR: Value out of range (1-4095)\r\n");
        }
    } else if (strncmp(cmd, "SET_DRY=", 8) == 0) {
        uint16_t val = (uint16_t)atoi(cmd + 8);
        if (val > 0 && val < 4096) {
            g_adc_dry_ref = val;
            snprintf(buf, sizeof(buf), "OK: DRY reference set to %u\r\n", g_adc_dry_ref);
            pc_log(buf);
        } else {
            pc_log("ERR: Value out of range (1-4095)\r\n");
        }
    } else if (strcmp(cmd, "GET_ADC") == 0) {
        snprintf(buf, sizeof(buf), "Current RAW ADC: %u\r\n", g_last_adc_raw);
        pc_log(buf);
        snprintf(buf, sizeof(buf), "Calibrated Refs: WET=%u, DRY=%u\r\n", g_adc_wet_ref, g_adc_dry_ref);
        pc_log(buf);
    } else {
        pc_log("ERR: Unknown command\r\n");
    }
    g_pc_cmd_received = false; // 플래그 클리어
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* 부트 메시지(ESP32 + PC 동시) */
  uart_send_str("BOOT: ready\r\n");
  pc_log("Commands: GET_ADC, SET_WET=xxxx, SET_DRY=xxxx\r\n");

  /* 초기 상태 */
  pump_set(false);
  g_tank_empty = tank_empty_raw();   // 부팅 시점 상태만 확정(알림 X)
  lastHumTx = lastCtrl = now_ms();

  /* PC UART 수신 인터럽트 시작 */
  HAL_UART_Receive_IT(&huart2, &g_pc_rx_char, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t t = now_ms();

    /* 0) PC 명령어 처리 */
    if(g_pc_cmd_received){
        process_pc_command();
    }

    /* 1) EXTI 디바운싱 확정 */
    if (g_db_pending && ((int32_t)(t - g_db_due_ms) >= 0)) {
      bool nowEmpty = tank_empty_raw();
      if ((uint8_t)nowEmpty == g_db_target_empty) {
        bool prev = g_tank_empty;
        g_tank_empty = nowEmpty;

        if (!prev && g_tank_empty){
          /* 충분 -> 부족 변화 확정 */
          send_F_empty();
          pump_set(false); // 안전: 부족이면 즉시 OFF
        } else if (prev && !g_tank_empty){
          /* 부족 -> 충분 변화 확정 */
          send_F_full();
          // *** 중요: 여기서 펌프를 켜지 않음! ***
          // 펌프는 아래 제어 로직에서 습도 조건까지 만족해야 켜짐
        }
      }
      g_db_pending = 0;
    }

    /* 2) 주기 제어/리포트 */
    if ((t - lastCtrl) >= CONTROL_PERIOD_MS) {
      lastCtrl = t;

      g_last_adc_raw = adc_read_average(16);
      uint8_t  humid_pct = humidity_percent_from_adc(g_last_adc_raw);

      if ((t - lastHumTx) >= HUMID_REPORT_PERIOD_MS) {
        lastHumTx = t;
        send_H(humid_pct);
      }

      bool pump_on = pump_get();

      if (!pump_on) {
        // OFF -> ON 조건: 물 충분하고(AND) 습도 낮음
        if (!g_tank_empty && (humid_pct <= HUMIDITY_LOW_TH)) {
          pump_set(true);
          pump_on_ms = t;
          send_W_ok(); // 급수 시작 시점에만 전송
        }
      } else {
        // ON 상태 종료 조건: 시간초과 또는 습도회복 또는 물부족
        bool time_exceeded     = ((t - pump_on_ms) >= PUMP_MAX_ON_MS);
        bool humidity_recovery = (humid_pct >= HUMIDITY_HIGH_TH);
        if (time_exceeded || humidity_recovery || g_tank_empty) {
          pump_set(false);
        }
      }
    }

    /* 3) 하트비트(PC 전용) + LD2 토글 */
    static uint32_t lastBeat = 0;
    if ((t - lastBeat) >= 1000) {
        lastBeat = t;
        char beatMsg[32];
        snprintf(beatMsg, sizeof(beatMsg), "BEAT (ADC:%u)\r\n", g_last_adc_raw);
        pc_log(beatMsg); // PC 터미널에서 ADC 값과 함께 즉시 확인
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // 보드 LED 1초마다 토글
    }

    /* 4) 유휴: Sleep (다음 인터럽트/틱까지 대기) */
    //__WFI();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

// ... 이 아래는 기존 코드와 동일합니다 ...
// SystemClock_Config, MX_ADC1_Init, MX_USART1_UART_Init, MX_USART2_UART_Init,
// MX_GPIO_Init, Error_Handler 등은 수정할 필요가 없습니다.
// main 함수와 USER CODE 0, PV, PD, PFP 섹션 위주로 수정되었습니다.

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{
  /* USER CODE BEGIN USART1_Init 0 */
  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
  /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* 필요 시 추가 사용자 함수 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1) { }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
