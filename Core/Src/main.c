/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PWM 1000
#define SPEED 220 // Hạ tốc độ từ 400 xuống 220 để xe 4WD có đủ thời gian bám vạch trên sa bàn ngắn
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim3; // Khai báo bộ định thời phần cứng Timer 3 cho ngắt chu kỳ 2ms (500 Hz)

// Mảng lưu giá trị cực tiểu (nền trắng) và cực đại (vạch đen) của 8 cảm biến khi calib
volatile uint16_t sensor_min[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
volatile uint16_t sensor_max[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Ngưỡng phân định màu Trắng và Đen riêng biệt cho từng mắt cảm biến
// Khởi tạo ngưỡng mặc định an toàn: Chân 4 được đặt vùng trễ siêu hẹp (3895 / 3945) quanh điểm giữa để phản hồi digital tức thì
volatile uint16_t threshold_white[8] = {3200, 3650, 3200, 3200, 3895, 3200, 3200, 3200};
volatile uint16_t threshold_black[8] = {3500, 3850, 3500, 3500, 3945, 3500, 3500, 3500};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM3_Init(void); // Khai báo nguyên mẫu hàm khởi tạo Timer 3
void calibrate_sensors(void);   // Khai báo nguyên mẫu hàm hiệu chuẩn (calib) cảm biến

// Thêm từ khóa volatile cho các biến dùng chung giữa ngắt (EXTI/DMA/TIM3) và vòng lặp chính
// để tránh trình biên dịch tối ưu hóa lưu biến vào thanh ghi CPU gây mất phản hồi
volatile uint8_t is_avoiding = 0;
volatile uint16_t adc[8];
volatile uint16_t sensor_digital[8]; // uint16_t để STM32CubeIDE hiển thị số 0 và 1 thuần túy

float error = 0.0f;
float last_error = 0.0f;
float I = 0.0f;
float Kp = 110.0f, Ki = 0.0f, Kd = 80.0f; // Tăng Kp = 110 để tạo lực lái mạnh mẽ cho khung xe 4WD khi vào cua gắt
volatile float PID_value = 0.0f; // volatile để biến được đồng bộ tức thì giữa ngắt và main

void delay_us(uint16_t us) {								// Hàm delay theo us để gửi xung cho cảm biến siêu âm
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

void pulse(GPIO_TypeDef *Port, uint16_t pin, uint8_t t){    // Tạo xung kích hoạt (Trigger) cho cảm biến siêu âm
	HAL_GPIO_WritePin(Port , pin, GPIO_PIN_RESET);
	delay_us(t/10);
	HAL_GPIO_WritePin(Port, pin, GPIO_PIN_SET);
	delay_us(t);
	HAL_GPIO_WritePin(Port, pin, GPIO_PIN_RESET);
}

void Set_Motor_Speed(int16_t speed_left, int16_t speed_right) {
    static int16_t prev_speed_left = 0x7FFF;
    static int16_t prev_speed_right = 0x7FFF;

    // Giới hạn giá trị PWM trong dải [-MAX_PWM, MAX_PWM]
    if (speed_left > MAX_PWM) speed_left = MAX_PWM;
    else if (speed_left < -MAX_PWM) speed_left = -MAX_PWM;
    if (speed_right > MAX_PWM) speed_right = MAX_PWM;
    else if (speed_right < -MAX_PWM) speed_right = -MAX_PWM;

    // Tối ưu hóa: Nếu tốc độ không đổi so với lần xuất trước, thoát ngay lập tức
    // Tránh spam ghi thanh ghi GPIO/TIM hàng triệu lần/giây, giải phóng bus và giảm nhiệt vi điều khiển
    if (speed_left == prev_speed_left && speed_right == prev_speed_right) {
        return;
    }
    prev_speed_left = speed_left;
    prev_speed_right = speed_right;

    uint16_t pwm_left = 0;
    uint16_t pwm_right = 0;

    // Điều khiển chiều quay và độ rộng xung động cơ trái (PB12, PB13, TIM1_CH1)
    // Đảo ngược chiều để quay TIẾN khi speed_left >= 0 (PB12=RESET, PB13=SET)
    if (speed_left >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
        pwm_left = (uint16_t)speed_left;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
        pwm_left = (uint16_t)(-speed_left);
    }

    // Điều khiển chiều quay và độ rộng xung động cơ phải (PB14, PB15, TIM1_CH2)
    // Đảo ngược chiều để quay TIẾN khi speed_right >= 0 (PB14=SET, PB15=RESET)
    if (speed_right >= 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        pwm_right = (uint16_t)speed_right;
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        pwm_right = (uint16_t)(-speed_right);
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_left);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm_right);
}

void pid_calculate(){
	float setpoint = 3.5f; // Điểm cân bằng trung tâm của 6 cảm biến (kênh 1 đến 6): (1 + 6) / 2 = 3.5
	uint32_t sum_value = 0;
	uint32_t weighted_sum = 0;
	float position = 0.0f;

	// Bỏ qua 2 kênh ngoài cùng (kênh 0 và kênh 7), chỉ tính toán trên 6 kênh bên trong từ 1 đến 6
	for (int i = 1; i <= 6; i++) {
	    if (sensor_digital[i] == 1) {
	        weighted_sum += i;
	        sum_value++;
	    }
	}

	if (sum_value > 0) {
	    position = (float)weighted_sum / (float)sum_value;
	    error = setpoint - position;
	} else {
	    // Khi mất line: Bẻ lái theo hướng lệch gần nhất để xe nhanh chóng quay lại line
	    if (last_error > 0.0f) {
	        error = 2.5f;
	    } else if (last_error < 0.0f) {
	        error = -2.5f;
	    } else {
	        error = 0.0f; // Vừa khởi động chưa thấy line: Giữ thẳng tìm line
	    }
	}

	float P = error;
	float D = error - last_error; // Khâu D chống rung lắc xe
	I += error;

	// Giới hạn chống bão hòa tích phân (Anti-windup)
	if (I > 1000.0f) I = 1000.0f;
	else if (I < -1000.0f) I = -1000.0f;

	PID_value = (Kp * P) + (Ki * I) + (Kd * D);
	last_error = error; // Cập nhật last_error cho chu kỳ tiếp theo
}
void avoid(void) {
    static uint8_t state = 0;
    static uint32_t tick = 0;

    switch (state) {
        case 0:
            Set_Motor_Speed(-400, -400);
            tick = HAL_GetTick();
            state = 1;
            break;
        case 1:
            if (HAL_GetTick() - tick >= 300) {
                Set_Motor_Speed(500, -500);
                tick = HAL_GetTick();
                state = 2;
            }
            break;
        case 2:
            if (HAL_GetTick() - tick >= 400) {
                Set_Motor_Speed(400, 400);
                tick = HAL_GetTick();
                state = 3;
            }
            break;
        case 3:
            if (HAL_GetTick() - tick >= 600) {
                Set_Motor_Speed(-500, 500);
                tick = HAL_GetTick();
                state = 4;
            }
            break;
        case 4:
            if (HAL_GetTick() - tick >= 400) {
                Set_Motor_Speed(400, 400);
                tick = HAL_GetTick();
                state = 5;
            }
            break;
        case 5:
            if (HAL_GetTick() - tick >= 800) {
                Set_Motor_Speed(-500, 500);
                tick = HAL_GetTick();
                state = 6;
            }
            break;
        case 6:
            if (HAL_GetTick() - tick >= 350) {
                Set_Motor_Speed(350, 350);
                tick = HAL_GetTick(); // Lưu tick bắt đầu để đếm thời gian cho state 7
                state = 7;
            }
            break;
        case 7:
            // Tối ưu hóa: Dùng trực tiếp giá trị đã lọc Schmitt Trigger sensor_digital thay vì so sánh ADC thô
            if (sensor_digital[3] == 1 || sensor_digital[4] == 1) {
                Set_Motor_Speed(500, -500);
                tick = HAL_GetTick();
                state = 8;
            } else if (HAL_GetTick() - tick >= 2000) {
                // Quá 2 giây không tìm thấy line, dừng xe an toàn
                Set_Motor_Speed(0, 0);
                is_avoiding = 0;
                state = 0;
            }
            break;
        case 8:
            if (HAL_GetTick() - tick >= 150) {
                Set_Motor_Speed(0, 0);
                tick = HAL_GetTick();
                state = 9;
            }
            break;
        case 9:
            if (HAL_GetTick() - tick >= 100) {
                state = 0;
                is_avoiding = 0;
            }
            break;
    }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if (GPIO_Pin == GPIO_PIN_11) {
		if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) == GPIO_PIN_SET) {
			__HAL_TIM_SET_COUNTER(&htim2, 0); // Sườn lên: Reset bộ đếm Timer 2 để bắt đầu đo thời gian
		}
		else {
	        uint32_t time_val = __HAL_TIM_GET_COUNTER(&htim2); // Sườn xuống: Lấy độ rộng xung Echo (micro-giây)
	        // Vận tốc âm thanh: 340 m/s = 0.034 cm/us. Khoảng cách (cm) = time * 0.034 / 2
	        float distance = (float)time_val * 0.034f / 2.0f;

	        // Lọc nhiễu khoảng cách ảo (bỏ qua giá trị <= 2cm do phản xạ gần) và phát hiện vật cản < 15cm
	        if (distance > 2.0f && distance < 15.0f) {
	        	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	        	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	            is_avoiding = 1;
	        }
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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  MX_TIM3_Init(); // Khởi tạo bộ định thời Timer 3
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  __HAL_TIM_MOE_ENABLE(&htim1);

  // Hiệu chuẩn phần cứng ADC trước khi kích hoạt DMA để loại bỏ sai số offset trên STM32F1
  HAL_ADCEx_Calibration_Start(&hadc1);

  // Khởi động ADC đọc liên tục 8 kênh qua DMA vào mảng adc[8]
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc, 8);
  HAL_Delay(50); // Chờ DMA ổn định vài vòng quét ban đầu

  // THỰC HIỆN CALIB CẢM BIẾN (3.5 giây, nhấp nháy đèn LED PC13 để người dùng rê xe qua vạch line)
  calibrate_sensors();

  // Khởi động ngắt định thời cứng siêu tốc 2ms (500 Hz) trên Timer 3
  HAL_TIM_Base_Start_IT(&htim3);

  uint32_t ultrasonic_tick = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (is_avoiding == 1) {
		  avoid();
	  }
	  else {
		  // Tách chu kỳ phát xung siêu âm riêng (mỗi 60ms) theo chuẩn cảm biến HC-SR04
		  if (HAL_GetTick() - ultrasonic_tick >= 60) {
		      ultrasonic_tick = HAL_GetTick();
		      pulse(GPIOB, GPIO_PIN_10, 10);
		  }

		  // Hàm chính CHỈ xuất giá trị PID (đã được tính sẵn từ ngắt 2ms) ra 2 động cơ
		  int16_t speed_left = (int16_t)(SPEED - PID_value);
		  int16_t speed_right = (int16_t)(SPEED + PID_value);

		  // Giới hạn tốc độ không âm khi dò line: Bánh trong chỉ hãm dừng lại làm tâm quay (không quay lùi giật xe)
		  if (speed_left < 0) speed_left = 0;
		  if (speed_right < 0) speed_right = 0;

		  Set_Motor_Speed(speed_left, speed_right);
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
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
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB10 PB12 PB13 PB14
                           PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief TIM3 Initialization Function (Ngắt định thời chu kỳ 2ms = 500Hz - Siêu tốc)
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  // Timer 3 cấp từ APB1 (72 MHz).
  // Prescaler = 71  -> Tần số đếm = 72 MHz / (71 + 1) = 1 MHz (mỗi tick = đúng 1 micro-giây)
  // Period = 1999   -> Chu kỳ tràn = (1999 + 1) * 1 us = 2,000 us = 2 ms (Tần số ngắt 500 Hz)
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  /* Cấu hình ưu tiên ngắt NVIC cho TIM3 (ưu tiên 2, sau EXTI và DMA) */
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/**
  * @brief Hàm phục vụ ngắt phần cứng của Timer 3
  */
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

/**
  * @brief Callback ngắt chu kỳ Timer 3: Chạy chuẩn xác mỗi 2ms (500 Hz)
  *        Đọc cảm biến ADC, chuyển đổi sang Digital 0/1 và tính toán PID siêu tốc
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    // Bỏ qua 2 kênh ngoài cùng: Cố định mức 0 cho kênh 0 và kênh 7
    sensor_digital[0] = 0;
    sensor_digital[7] = 0;

    // 1. Chỉ đọc và chuyển đổi 6 kênh bên trong (từ kênh 1 đến kênh 6)
    // Caching adc[i] vào biến cục bộ val để tránh đọc lặp lại từ RAM/DMA
    for (int i = 1; i <= 6; i++) {
      uint16_t val = adc[i];
      if (val < threshold_white[i]) {
        sensor_digital[i] = 0; // Màu trắng (nhỏ hơn ngưỡng trắng của mắt i)
      } else if (val > threshold_black[i]) {
        sensor_digital[i] = 1; // Màu đen (lớn hơn ngưỡng đen của mắt i)
      }
      // Nằm trong khoảng [threshold_white[i], threshold_black[i]]: Giữ nguyên trạng thái cũ (chống rung mép vạch)
    }

    // 2. Tính toán PID luôn trong ngắt với tốc độ cao 500 lần/giây (chu kỳ 2ms)
    if (is_avoiding == 0) {
      pid_calculate();
    }
  }
}

/**
  * @brief Hàm hiệu chuẩn (calib) cảm biến quang:
  *        - Chạy trong 3.5 giây ngay khi bật nguồn.
  *        - Nhấp nháy đèn LED PC13 để báo người dùng rê xe qua vạch line.
  *        - Bỏ qua 2 kênh ngoài cùng (0 và 7), chỉ calib 6 kênh từ 1 đến 6.
  *        - Đo giá trị Min (nền trắng) và Max (vạch đen) riêng biệt cho từng cảm biến.
  *        - Tự động tính toán threshold_white và threshold_black cho từng kênh.
  *        - Xử lý đặc biệt cho chân 1 và 4 (ít nhạy với màu trắng hơn các mắt khác).
  */
void calibrate_sensors(void) {
    // 1. Cấu hình chân PC13 điều khiển đèn LED báo trạng thái
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 2. Khởi tạo mảng lưu giá trị min và max cho 6 kênh sử dụng (từ 1 đến 6)
    for (int i = 1; i <= 6; i++) {
        sensor_min[i] = 4095;
        sensor_max[i] = 0;
    }

    uint32_t start_time = HAL_GetTick();

    // 3. Vòng lặp calib kéo dài 3.5 giây: Đọc liên tục ADC cho 6 kênh từ 1 đến 6
    while (HAL_GetTick() - start_time < 3500) {
        for (int i = 1; i <= 6; i++) {
            uint16_t current_val = adc[i];
            if (current_val < sensor_min[i]) sensor_min[i] = current_val;
            if (current_val > sensor_max[i]) sensor_max[i] = current_val;
        }

        // Nhấp nháy đèn PC13 mỗi 100ms (Active Low: 0 = sáng, 1 = tắt)
        if (((HAL_GetTick() - start_time) / 100) % 2 == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }
        HAL_Delay(2);
    }

    // 4. Tính toán ngưỡng Schmitt Trigger riêng biệt cho 6 mắt cảm biến (1 đến 6)
    for (int i = 1; i <= 6; i++) {
        uint16_t span = (sensor_max[i] > sensor_min[i]) ? (sensor_max[i] - sensor_min[i]) : 0;

        // Hạ ngưỡng kiểm tra xuống span > 60 để nhận diện được các cảm biến dải đo hẹp (như chân 4)
        if (span > 60) {
            uint16_t mid = sensor_min[i] + (span / 2);

            // Riêng chân 4: Kém nhạy màu trắng và dải đo hẹp -> Dùng vùng trễ siêu hẹp quanh điểm giữa mid
            // để chuyển đổi trạng thái digital tức thì (Fast Response), không còn bị trễ
            if (i == 4) {
                uint16_t delta = (span * 6) / 100; // Vùng trễ chỉ 6% dải đo (khoảng 15-20 đơn vị ADC)
                if (delta < 15) delta = 15;
                if (delta > 25) delta = 25;
                threshold_white[4] = mid - delta;
                threshold_black[4] = mid + delta;
            } else if (i == 1 || sensor_min[i] > 3200) {
                // Chân 1 hoặc mắt có mức trắng cao: Vùng trễ hẹp quanh điểm giữa
                uint16_t delta = (span * 10) / 100;
                threshold_white[i] = mid - delta;
                threshold_black[i] = mid + delta;
            } else {
                // Các chân bình thường: Điểm giữa ± 15% span
                uint16_t delta = (span * 15) / 100;
                threshold_white[i] = mid - delta;
                threshold_black[i] = mid + delta;
            }
        } else {
            // Trường hợp mắt không quét qua vạch: Gán ngưỡng mặc định siêu nhạy
            if (i == 4) {
                threshold_white[4] = 3895;
                threshold_black[4] = 3945;
            } else if (i == 1) {
                threshold_white[1] = 3680;
                threshold_black[1] = 3780;
            }
        }
    }

    // 5. Báo hiệu hoàn tất: LED PC13 sáng liên tục 500ms rồi tắt
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_Delay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
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
#ifdef USE_FULL_ASSERT
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
