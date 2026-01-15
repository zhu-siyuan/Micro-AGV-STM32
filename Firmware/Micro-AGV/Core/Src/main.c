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
#include <stdlib.h> // 用于 abs() 函数
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void Control_Loop(void);
void Set_Motor_Speed(int speed_L, int speed_R);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// --- 硬件调试参数 ---
// 如果发现轮子转动方向相反，或者 PID 导致飞车（正反馈），将对应的 1 改为 -1
#define MOTOR_L_POLARITY 1
#define MOTOR_R_POLARITY 1

// --- 电机引脚控制宏 ---
#define MOTOR_L_FWD()                                                          \
  {                                                                            \
    HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, 1);                          \
    HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, 0);                          \
  }
#define MOTOR_L_BWD()                                                          \
  {                                                                            \
    HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, 0);                          \
    HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, 1);                          \
  }
#define MOTOR_L_STOP()                                                         \
  {                                                                            \
    HAL_GPIO_WritePin(L_IN1_GPIO_Port, L_IN1_Pin, 0);                          \
    HAL_GPIO_WritePin(L_IN2_GPIO_Port, L_IN2_Pin, 0);                          \
  }

#define MOTOR_R_FWD()                                                          \
  {                                                                            \
    HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 1);                          \
    HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 0);                          \
  }
#define MOTOR_R_BWD()                                                          \
  {                                                                            \
    HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 0);                          \
    HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 1);                          \
  }
#define MOTOR_R_STOP()                                                         \
  {                                                                            \
    HAL_GPIO_WritePin(R_IN1_GPIO_Port, R_IN1_Pin, 0);                          \
    HAL_GPIO_WritePin(R_IN2_GPIO_Port, R_IN2_Pin, 0);                          \
  }

// --- PWM 与死区设置 ---
#define PWM_MAX_VALUE 1799
#define MOTOR_DEAD_ZONE 200 // 电机死区补偿值 (根据实测调整，通常 150-300)

// 设置电机速度 (输入范围: -1799 到 1799)
void Set_Motor_Speed(int pwm_L, int pwm_R) {
  // --- 左轮控制 ---
  int out_L = pwm_L;
  // 死区补偿：如果有输出，则加上死区值
  if (out_L > 0)
    out_L += MOTOR_DEAD_ZONE;
  else if (out_L < 0)
    out_L -= MOTOR_DEAD_ZONE;

  // 限幅
  if (out_L > PWM_MAX_VALUE)
    out_L = PWM_MAX_VALUE;
  if (out_L < -PWM_MAX_VALUE)
    out_L = -PWM_MAX_VALUE;

  // 驱动硬件
  if (out_L > 0)
    MOTOR_L_FWD()
  else if (out_L < 0)
    MOTOR_L_BWD()
  else
    MOTOR_L_STOP();

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, abs(out_L));

  // --- 右轮控制 ---
  int out_R = pwm_R;
  if (out_R > 0)
    out_R += MOTOR_DEAD_ZONE;
  else if (out_R < 0)
    out_R -= MOTOR_DEAD_ZONE;

  if (out_R > PWM_MAX_VALUE)
    out_R = PWM_MAX_VALUE;
  if (out_R < -PWM_MAX_VALUE)
    out_R = -PWM_MAX_VALUE;

  if (out_R > 0)
    MOTOR_R_FWD()
  else if (out_R < 0)
    MOTOR_R_BWD()
  else
    MOTOR_R_STOP();

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, abs(out_R));
}

// --- 物理参数 ---
#define PULSE_PER_TURN 2060.0f // 编码器每圈脉冲数
#define WHEEL_DIAMETER 0.064f  // 轮子直径 64mm
#define WHEEL_CIRCUMFERENCE (3.14159f * WHEEL_DIAMETER)
#define CONTROL_PERIOD 50 // 控制周期 50ms

// 速度转换系数: (脉冲数 -> m/s)
const float SPEED_RATIO =
    (WHEEL_CIRCUMFERENCE / PULSE_PER_TURN) * (1000.0f / CONTROL_PERIOD);

// 编码器历史值
int16_t last_count_L = 0;
int16_t last_count_R = 0;

// --- 速度控制变量 ---
// final_speed: 蓝牙下达的最终目标速度
volatile float final_speed_L = 0.0f;
volatile float final_speed_R = 0.0f;
// ramped_speed: 送给 PID 的平滑后目标速度 (解决起步猛冲)
float ramped_speed_L = 0.0f;
float ramped_speed_R = 0.0f;
// current_speed: 实际测量速度
float current_speed_L = 0.0f;
float current_speed_R = 0.0f;

// --- 平滑启动参数 ---
// 加速度限制: 每次 50ms 周期速度变化不超过 0.02m/s
// 相当于 0.4 m/s^2 的加速度，起步会很柔和
#define ACCEL_STEP 0.02f

// --- 安全看门狗 ---
uint32_t last_rx_time = 0; // 上一次收到串口数据的时间戳
#define COM_TIMEOUT 500    // 超时时间 500ms (验收标准要求具备自动急停)

// PID 结构体
typedef struct {
  float Kp, Ki, Kd;
  float prev_error;
  float integral;
  float i_limit;
} PID_t;

// PID 参数 (针对 JGA25-370 调整)
// Kp: 响应力度, Ki: 消除静态误差(跑直线的关键), Kd: 抑制震荡
PID_t pid_L = {600.0f, 78.0f, 0.0f, 0.0f, 0.0f, 1500.0f};
PID_t pid_R = {600.0f, 80.0f, 0.0f, 0.0f, 0.0f, 1500.0f};

uint32_t last_time = 0;

// 串口变量
uint8_t rx_buffer[1];
uint8_t data_buffer[10];
uint8_t data_index = 0;
#define STATE_WAIT_HEADER 0
#define STATE_RECEIVE_BODY 1
uint8_t rx_state = STATE_WAIT_HEADER;

// 电机方向定义 (根据实际接线调整 1 或 -1)
#define MOTOR_L_POLARITY 1
#define MOTOR_R_POLARITY 1

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  // --- 强制初始化 PA0 和 PA1 为复用推挽输出 (PWM模式) ---
  // 1. 开启 GPIOA 时钟
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 2. 配置 PA0 (TIM2_CH1) 和 PA1 (TIM2_CH2)
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // 关键：复用推挽输出
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // 1. 开启 PWM 输出
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  // 2. 开启编码器计数模式
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // 开启串口中断
  HAL_UART_Receive_IT(&huart1, rx_buffer, 1);

  // 初始化看门狗时间
  last_rx_time = HAL_GetTick();

  // 初始化编码器计数器为0，防止启动时有随机值
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  // 5. 初始化时间戳
  last_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
  // GPIOA->CRL &= 0xFFFFFFF0; GPIOA->CRL |= 0x00000003; GPIOA->ODR |= 1;
  while (1) {
    // 50ms 控制周期
    if (HAL_GetTick() - last_time >= CONTROL_PERIOD) {
      last_time = HAL_GetTick();
      Control_Loop();
    }

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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1799;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }

  // ===============================================
  // 重点修正：在这里直接由硬件工程师“手写”引脚配置，确保生效！
  // ===============================================

  // 1. 开启 GPIOA 时钟 (非常重要，漏了这个就全完了)
  __HAL_RCC_GPIOA_CLK_ENABLE();

  // 2. 定义 GPIO 结构体
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // 3. 配置 PA0 和 PA1
  // 注意：Mode 必须是 GPIO_MODE_AF_PP (复用推挽输出)
  // 只有这样，定时器的信号才能通过引脚发出来
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  // ===============================================

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }

  // 这一行可以注释掉，也可以留着，反正我们在上面已经手动执行过真正的初始化了
  HAL_TIM_MspPostInit(&htim2);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, L_IN1_Pin | L_IN2_Pin | R_IN1_Pin | R_IN2_Pin,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : L_IN1_Pin L_IN2_Pin R_IN1_Pin R_IN2_Pin */
  GPIO_InitStruct.Pin = L_IN1_Pin | L_IN2_Pin | R_IN1_Pin | R_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// --- 串口接收回调 ---
// 串口接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    // 喂狗：只要有中断进来，就更新时间戳
    // 注意：更严格的做法是校验成功后再更新，但防断连这样够用了
    last_rx_time = HAL_GetTick();

    uint8_t received_byte = rx_buffer[0];

    switch (rx_state) {
    case STATE_WAIT_HEADER:
      if (received_byte == 0xA5) { // 帧头检测
        data_index = 0;
        data_buffer[data_index++] = received_byte;
        rx_state = STATE_RECEIVE_BODY;
      }
      break;

    case STATE_RECEIVE_BODY:
      if (data_index < sizeof(data_buffer)) {
        data_buffer[data_index++] = received_byte;
        if (data_index >= 8) {          // 接收完整一帧
          if (data_buffer[1] == 0x01) { // 功能位校验
            int8_t speed_cmd = (int8_t)data_buffer[2];
            float target = speed_cmd / 100.0f; // 解析速度

            // 更新最终目标速度
            final_speed_L = target;
            final_speed_R = target;
          }
          data_index = 0;
          rx_state = STATE_WAIT_HEADER;
        }
      } else {
        data_index = 0;
        rx_state = STATE_WAIT_HEADER;
      }
      break;
    }
    HAL_UART_Receive_IT(&huart1, rx_buffer, 1);
  }
}

// --- PID 计算核心 ---
// --- PID 计算函数 ---
// 输入：目标速度，当前速度，PID参数包
// 输出：PWM值 (-1000 ~ 1000)
int16_t PID_Compute(float target, float current, PID_t *pid) {
  float error = target - current;

  // 积分项 (消除静差)
  pid->integral += error;

  // 积分限幅 (使用 struct 中的 i_limit)
  if (pid->integral > pid->i_limit)
    pid->integral = pid->i_limit;
  if (pid->integral < -pid->i_limit)
    pid->integral = -pid->i_limit;

  // 微分项 (阻尼作用)
  float derivative = error - pid->prev_error;

  // PID 公式
  float output =
      (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);

  pid->prev_error = error;

  // PWM 限幅 (应与 PWM_MAX_VALUE 匹配)
  if (output > PWM_MAX_VALUE)
    output = PWM_MAX_VALUE;
  if (output < -PWM_MAX_VALUE)
    output = -PWM_MAX_VALUE;

  return (int16_t)output;
}

// --- 速度斜坡函数 (实现平稳起步的关键) ---
// 将 current 向 target 靠近，但每次变化量不超过 step
float Apply_Ramp(float current, float target, float step) {
  if (current < target) {
    current += step;
    if (current > target)
      current = target; // 防止超调
  } else if (current > target) {
    current -= step;
    if (current < target)
      current = target;
  }
  return current;
}

void Control_Loop(void) {
  // 1. 安全检查：通信超时自动急停 (如果你想测试长跑，把 COM_TIMEOUT 改大，比如 5000)
  if (HAL_GetTick() - last_rx_time > COM_TIMEOUT) {
    final_speed_L = 0.0f;
    final_speed_R = 0.0f;
  }

  // 2. 速度平滑处理 (斜坡算法)
  ramped_speed_L = Apply_Ramp(ramped_speed_L, final_speed_L, ACCEL_STEP);
  ramped_speed_R = Apply_Ramp(ramped_speed_R, final_speed_R, ACCEL_STEP);

  // 3. 读取编码器并计算真实速度
  int16_t cur_cnt_L = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
  int16_t cur_cnt_R = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

  int16_t diff_L = (cur_cnt_L - last_count_L) * MOTOR_L_POLARITY;
  int16_t diff_R = (cur_cnt_R - last_count_R) * MOTOR_R_POLARITY;

  last_count_L = cur_cnt_L;
  last_count_R = cur_cnt_R;

  current_speed_L = (float)diff_L * SPEED_RATIO;
  current_speed_R = (float)diff_R * SPEED_RATIO;

  // 4. PID 计算 (新增：静止死锁逻辑，解决抽搐问题)
  int16_t pwm_out_L = 0;
  int16_t pwm_out_R = 0;

  // --- 左轮处理 ---
  // 如果目标速度极小（认为是要停车），强制关断
  if (fabs(ramped_speed_L) < 0.001f) {
      pwm_out_L = 0;
      pid_L.integral = 0.0f;     // 清除积分，防止再次启动时猛冲
      pid_L.prev_error = 0.0f;   // 清除微分历史
  } else {
      // 只有在需要动的时候，才启动 PID
      pwm_out_L = PID_Compute(ramped_speed_L, current_speed_L, &pid_L);
  }

  // --- 右轮处理 ---
  if (fabs(ramped_speed_R) < 0.001f) {
      pwm_out_R = 0;
      pid_R.integral = 0.0f;
      pid_R.prev_error = 0.0f;
  } else {
      pwm_out_R = PID_Compute(ramped_speed_R, current_speed_R, &pid_R);
  }

  // 5. 执行输出
  Set_Motor_Speed(pwm_out_L, pwm_out_R);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
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