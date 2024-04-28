/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include"stdio.h"
#include"string.h"

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
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define ENABLE_GPIO GPIOA
#define ENABLE_PIN GPIO_PIN_8

#define DIR1_GPIO GPIOB
#define DIR1_PIN GPIO_PIN_10
#define DIR2_GPIO GPIOB
#define DIR2_PIN GPIO_PIN_4
#define DIR3_GPIO GPIOB
#define DIR3_PIN GPIO_PIN_5
#define DIR4_GPIO GPIOB
#define DIR4_PIN GPIO_PIN_3

#define STEP_GPIO GPIOA
#define STEP_PIN GPIO_PIN_7

#define HB1_GPIO  GPIOC
#define HB2_GPIO  GPIOC
#define HB3_GPIO  GPIOC
#define HB4_GPIO  GPIOC
#define HB1_PIN  GPIO_PIN_0
#define HB2_PIN GPIO_PIN_1
#define HB3_PIN GPIO_PIN_2
#define HB4_PIN GPIO_PIN_3
char answer = 0;
char answers[2] = {0};
int answerOffset = 1;

int stepState = 0;
int PINCH_FORWARD_CCR = 205;
int PINCH_BACKWARD_CCR = 320;
int PAN_STOP = 302;
int PAN_UP = 310;
int PAN_DOWN = 295;


void HAL_GPIO_EXTI_Callback(uint16_t pin){
	if(pin == GPIO_PIN_12 || pin == GPIO_PIN_13 || pin == GPIO_PIN_14 || pin == GPIO_PIN_15){
		HAL_TIM_Base_Start_IT(&htim5);
	}
}

void step(int num)
{
	HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 1);
	HAL_Delay(5);
	HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 0);
	HAL_Delay(5);
}

void set_forward_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 1);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 1);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-1);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-1);
}
void set_backward_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-0);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-0);
}
void set_left_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 1);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-1);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-0);
}
void set_right_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 1);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-0);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-1);
}
void set_ccw_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-1);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-1);
}
void set_cw_dir()
{
	HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 1);
	HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 1);
	HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 1-0);
	HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 1-0);
}

static int steps_left = 0;
static int disabled = 0;
void enable()
{
	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 0);
}
void disable()
{
	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 1);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	static int dir = 0;

    if (htim->Instance == TIM11) {
    	int disable_next = 0;
    	if (steps_left > 0)
    	{
    		enable();
    		if (stepState)
    		{
    			steps_left--;
    			if (steps_left == 0)
    			{
    				disable_next = 1;
    			}
    		}
    		switch(dir)
    		{
    		case 0:
    			set_forward_dir();
    			break;
    		case 1:
    			set_backward_dir();
    			break;
    		case 2:
    			set_left_dir();
    			break;
    		case 3:
    			set_right_dir();
    			break;
    		}
    	}
		HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, stepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
    	stepState = !stepState;
    	if (disable_next)
    	{
    		disable();
    		disabled = 1;
    		HAL_TIM_Base_Stop_IT(&htim11);
    	}
    }
    else if (htim->Instance == TIM10)
    {
    	TIM2->CCR1 = PINCH_BACKWARD_CCR;
    	HAL_TIM_Base_Stop_IT(&htim10);
    }
    else if(htim->Instance == TIM5){
		GPIO_PinState ultra1 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
		GPIO_PinState ultra2 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
		GPIO_PinState ultra3 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
		GPIO_PinState ultra4 = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15);
		if (steps_left == 0)
		{
			if(ultra1){
				 set_forward_dir();
				 dir = 0;
				 steps_left = 50;
				printf("1\n");
				HAL_TIM_Base_Start_IT(&htim11);
			}
			else if(ultra2){
				set_left_dir();
				dir = 3;
				steps_left = 50;
				printf("2\n");
				HAL_TIM_Base_Start_IT(&htim11);
			}
			else if(ultra3){
				set_backward_dir();
				dir = 1;
				steps_left = 50;
				printf("3\n");
				HAL_TIM_Base_Start_IT(&htim11);
			}
			else if(ultra4)
			{
				set_right_dir();
				dir = 2;
				steps_left = 50;
				printf("4\n");
				HAL_TIM_Base_Start_IT(&htim11);
			}
		}
		HAL_TIM_Base_Stop_IT(&htim5);
	}
}




uint16_t number = 0;
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
  MX_USART6_UART_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM10_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  // HAL_TIM_Base_Start_IT(&htim10);
  HAL_UART_Receive_IT(&huart6, &answer, 1);
  HAL_UART_Receive_IT(&huart2, &number, 1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  // HAL_TIM_Base_Start_IT(&htim11);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 1);
  HAL_GPIO_WritePin(DIR1_GPIO, DIR1_PIN, 0);
  HAL_GPIO_WritePin(DIR2_GPIO, DIR2_PIN, 0);
  HAL_GPIO_WritePin(DIR3_GPIO, DIR3_PIN, 0);
  HAL_GPIO_WritePin(DIR4_GPIO, DIR4_PIN, 0);

  disable();
  while (1)
  {
	  printf("hello world\n");
	  // HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 1);
	  // HAL_Delay(100);
	  // HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 0);
	  // HAL_Delay(100);
	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 419;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 419;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 839;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 83;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 3999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 4500;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 9000;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 39;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 3999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA7 PA8 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef* hadc) {

	if (hadc == &huart2)
	{
		static char numbers[4] = {0};
		static int offset = 0;


		if (number != '\n')
		{
			printf("received: %c\n", number);
			numbers[offset++] = number;

			if (offset == 3)
			{
				int intNum = atoi(numbers);
				printf("set: %d\n", intNum);
				// TIM2->CCR1 = intNum;
				TIM3->CCR1 = intNum;
				offset = 0;
			}
		}
		HAL_UART_Receive_IT(&huart2, &number, 1);

	}
	else
	{

		answers[answerOffset--] = answer;
		if (answerOffset == -1)
		{
			answerOffset = 1;
		}
		else
		{
			HAL_UART_Receive_IT(&huart6, &answer, 1);
			return;
		}
		printf("received: %d\n", *(uint16_t*)(&answers));
		static int hold = 0;
		switch(*(uint16_t*)(&answers))
		{
		case 1:
			set_ccw_dir();
			break;
		case 2:
			set_cw_dir();
			break;
		case 4:
			set_forward_dir();
			break;
		case 8:
			set_backward_dir();
			break;
		case 16:
			HAL_TIM_Base_Start_IT(&htim10);
			TIM2->CCR1 = PINCH_FORWARD_CCR;
			break;
		case 32:
			static int spinning = 0;

			if (!hold)
			{
				hold = 1;
				spinning = 1-spinning;
			}
			if (!spinning)
			{
				HAL_GPIO_WritePin(HB1_GPIO, HB1_PIN, 0);
				HAL_GPIO_WritePin(HB2_GPIO, HB2_PIN, 0);
				HAL_GPIO_WritePin(HB3_GPIO, HB3_PIN, 0);
				HAL_GPIO_WritePin(HB4_GPIO, HB4_PIN, 0);
			}
			else
			{
				HAL_GPIO_WritePin(HB1_GPIO, HB1_PIN, 0);
				HAL_GPIO_WritePin(HB2_GPIO, HB2_PIN, 1);
				HAL_GPIO_WritePin(HB3_GPIO, HB3_PIN, 1);
				HAL_GPIO_WritePin(HB4_GPIO, HB4_PIN, 0);
			}
			break;

		}

		if ((answer & 0b1111) == 0 && !disabled && steps_left == 0)
		{
			disabled = 1;
			disable();
			HAL_TIM_Base_Stop_IT(&htim11);
		}
		else if ((answer & 0b1111) != 0 && disabled)
		{
			disabled = 0;
			enable();
			HAL_TIM_Base_Start_IT(&htim11);
		}
		disabled = (answer & 0b1111) == 0;
		if ((answer & 32) == 0)
		{
			hold = 0;
		}
		 HAL_UART_Receive_IT(&huart6, &answer, 1);
	}
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
