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
#include "stdbool.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef void (*action_func)(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIM4_ADDR 0x40000800
#define TIM3_ADDR 0x40000400
#define TIM_CCR2_OFFSET 0x38
#define CCR_MASK 0xFFFF


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN_PAN 60
#define IDLE_PAN 155
#define MAX_PAN 240
#define STEP_PAN 5
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
static uint8_t flag = 0;
volatile bool shouldSwitch = false;
volatile uint32_t*  tim4_ccr2 = (uint32_t*)(TIM4_ADDR + TIM_CCR2_OFFSET);
action_func currentAct;

// transmit and receive buffer for Pixy2 cam
uint8_t tx_buffer[32] = {
		0xae,
		0xc1,
		0x0e,
		0x00
};

uint8_t rx_buffer[32];


// receive buffer for PS2 control
uint8_t answer;

// status flag for stepper driving
static uint8_t stepState = 0;

static uint8_t tiltState = 0;

static uint8_t panState = 0;

static uint8_t shootState = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

GPIO_TypeDef* H_Bridge_A1_GPIO = GPIOC; //PC12
uint16_t H_Bridge_A1_PIN = GPIO_PIN_12;


GPIO_TypeDef* H_Bridge_A2_GPIO = GPIOD; //PD2
uint16_t H_Bridge_A2_PIN = GPIO_PIN_2;


GPIO_TypeDef* ENABLE_GPIO =  GPIOA;// PA8 D7
uint16_t ENABLE_PIN = GPIO_PIN_8;

GPIO_TypeDef* DIR_FRONT_LEFT_GPIO = GPIOB; // PB1 D6
uint16_t DIR_FRONT_LEFT_PIN = GPIO_PIN_1;


GPIO_TypeDef* DIR_BACK_LEFT_GPIO = GPIOB; // PB4 D5
uint16_t DIR_BACK_LEFT_PIN = GPIO_PIN_4;


GPIO_TypeDef* DIR_FRONT_RIGHT_GPIO = GPIOB; // PB5 D4
uint16_t DIR_FRONT_RIGHT_PIN = GPIO_PIN_5;


GPIO_TypeDef* DIR_BACK_RIGHT_GPIO = GPIOB; // PB3 D3
uint16_t DIR_BACK_RIGHT_PIN = GPIO_PIN_3;

GPIO_TypeDef* STEP_GPIO = GPIOA; // PA_7(D11)
uint16_t STEP_PIN = GPIO_PIN_7;


// Flywheel control sets
void run_wheelA(){
	HAL_GPIO_WritePin(H_Bridge_A1_GPIO, H_Bridge_A1_PIN, 1);
	HAL_GPIO_WritePin(H_Bridge_A2_GPIO, H_Bridge_A2_PIN, 0);
}


// Stepper control sets
void set_forward_dir()
{
	HAL_GPIO_WritePin(DIR_FRONT_LEFT_GPIO, DIR_FRONT_LEFT_PIN, 1);
	HAL_GPIO_WritePin(DIR_BACK_LEFT_GPIO, DIR_BACK_LEFT_PIN, 1);
	HAL_GPIO_WritePin(DIR_FRONT_RIGHT_GPIO, DIR_FRONT_RIGHT_PIN, 1);
	HAL_GPIO_WritePin(DIR_BACK_RIGHT_GPIO, DIR_BACK_RIGHT_PIN, 1);
}
void set_backward_dir()
{
	HAL_GPIO_WritePin(DIR_FRONT_LEFT_GPIO, DIR_FRONT_LEFT_PIN, 0);
	HAL_GPIO_WritePin(DIR_BACK_LEFT_GPIO, DIR_BACK_LEFT_PIN, 0);
	HAL_GPIO_WritePin(DIR_FRONT_RIGHT_GPIO, DIR_FRONT_RIGHT_PIN, 0);
	HAL_GPIO_WritePin(DIR_BACK_RIGHT_GPIO, DIR_BACK_RIGHT_PIN, 0);
}
void set_left_dir()
{
	HAL_GPIO_WritePin(DIR_FRONT_LEFT_GPIO, DIR_FRONT_LEFT_PIN, 0);
	HAL_GPIO_WritePin(DIR_BACK_LEFT_GPIO, DIR_BACK_LEFT_PIN, 1);
	HAL_GPIO_WritePin(DIR_FRONT_RIGHT_GPIO, DIR_FRONT_RIGHT_PIN, 1);
	HAL_GPIO_WritePin(DIR_BACK_RIGHT_GPIO, DIR_BACK_RIGHT_PIN, 0);
}
void set_right_dir()
{
	HAL_GPIO_WritePin(DIR_FRONT_LEFT_GPIO, DIR_FRONT_LEFT_PIN, 1);
	HAL_GPIO_WritePin(DIR_BACK_LEFT_GPIO, DIR_BACK_LEFT_PIN, 0);
	HAL_GPIO_WritePin(DIR_FRONT_RIGHT_GPIO, DIR_FRONT_RIGHT_PIN, 0);
	HAL_GPIO_WritePin(DIR_BACK_RIGHT_GPIO, DIR_BACK_RIGHT_PIN, 1);
}
void set_ccw_dir()
{
	HAL_GPIO_WritePin(DIR_FRONT_LEFT_GPIO, DIR_FRONT_LEFT_PIN, 0);
		HAL_GPIO_WritePin(DIR_BACK_LEFT_GPIO, DIR_BACK_LEFT_PIN, 0);
		HAL_GPIO_WritePin(DIR_FRONT_RIGHT_GPIO, DIR_FRONT_RIGHT_PIN, 1);
		HAL_GPIO_WritePin(DIR_BACK_RIGHT_GPIO, DIR_BACK_RIGHT_PIN, 1);
}
void set_cw_dir()
{
	HAL_GPIO_WritePin(DIR_FRONT_LEFT_GPIO, DIR_FRONT_LEFT_PIN, 1);
		HAL_GPIO_WritePin(DIR_BACK_LEFT_GPIO, DIR_BACK_LEFT_PIN, 1);
		HAL_GPIO_WritePin(DIR_FRONT_RIGHT_GPIO, DIR_FRONT_RIGHT_PIN, 0);
		HAL_GPIO_WritePin(DIR_BACK_RIGHT_GPIO, DIR_BACK_RIGHT_PIN, 0);
}
void move(int steps, void(*func)() ) {
	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 0);
	func();
	for(int i = 0;i < steps;i++)
	{
        HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 1);


        HAL_Delay(1); // Adjust delay for desired speed
        HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, 0);
        HAL_Delay(1); // Adjust delay for desired speed
	}
	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 1);
}
void move_forward(steps)
{
	move(steps, set_forward_dir);
}
void move_backward(steps)
{
	move(steps, set_backward_dir);
}
void move_left(steps)
{
	move(steps, set_left_dir);
}
void move_right(steps)
{
	move(steps, set_right_dir);
}
void move_ccw(steps)
{
	move(steps, set_ccw_dir);
}
void move_cw(steps)
{
	move(steps, set_cw_dir);
}


void move_car(void){


	while(!shouldSwitch){

	}
	printf("Finish moving\n");
	shouldSwitch = false;
}

void move_pan(void){

//	*tim4_ccr2 = MIN_PAN;
//	while(!shouldSwitch){
//		if(*tim4_ccr2 <= MAX_PAN){
//			HAL_Delay(1);
//			*tim4_ccr2 = *tim4_ccr2 +  STEP_PAN;
//		}
//
//
//	}

	printf("Tilting finished\n");
	shouldSwitch = false;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM3) {
        // move the wheels

     	HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 0);
        HAL_GPIO_WritePin(STEP_GPIO, STEP_PIN, stepState ? GPIO_PIN_SET : GPIO_PIN_RESET);
        stepState = !stepState;
    }
}


// pan&tilt coordinate with wheels callback function
void HAL_GPIO_EXTI_Callback(uint16_t pin){
	flag = !flag;
	shouldSwitch = true;

	if(!flag)
		currentAct = move_car;
	else
		currentAct = move_pan;

}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	currentAct = move_car;


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
  MX_TIM4_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  //  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  //  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_UART_Receive_IT(&huart6, &answer, 1);
  // start PWM function for timer4
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


 // HAL_Delay(10000);
  while(1){

	  if(stepState==1)
		  move_forward(1);
	  else if (stepState==2)
		  move_backward(1);
	  else if (stepState==3)
		  move_ccw(1);
	  else if (stepState==4)
		  move_cw(1);

	  if(tiltState == 1){
//			if(*tim4_ccr2 <= MAX_PAN){
//				HAL_Delay(1);
//				*tim4_ccr2 = *tim4_ccr2 + STEP_PAN;
//			}
		  *tim4_ccr2 = MIN_PAN;
	  }
	  else if (tiltState==2){
//			if(*tim4_ccr2 >= MIN_PAN){
//				HAL_Delay(1);
//				*tim4_ccr2 = *tim4_ccr2 - STEP_PAN;
//			}
		  *tim4_ccr2 = MAX_PAN;
	  }
	  else{
		  *tim4_ccr2 = IDLE_PAN;
	  }

	  if(shootState){
		  run_wheelA();
	  }

  }

//  while (1)
//  {
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 39;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
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
  htim4.Init.Prescaler = 419;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
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
  HAL_GPIO_WritePin(GPIOA, STEP_GPIO_Pin|STEPPER_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FRONT_LEFT_DIR_Pin|BACK_RIGHT_DIR_Pin|BACK_LEFT_DIR_Pin|BACK_RIGHT_DIIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(USART_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEP_GPIO_Pin STEPPER_EN_Pin */
  GPIO_InitStruct.Pin = STEP_GPIO_Pin|STEPPER_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FRONT_LEFT_DIR_Pin BACK_RIGHT_DIR_Pin BACK_LEFT_DIR_Pin BACK_RIGHT_DIIR_Pin */
  GPIO_InitStruct.Pin = FRONT_LEFT_DIR_Pin|BACK_RIGHT_DIR_Pin|BACK_LEFT_DIR_Pin|BACK_RIGHT_DIIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */

/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool check_recv_bit(uint8_t bit_num){
	bool is_one = (((answer&(0b1<< bit_num)) >> bit_num)==1);
	return is_one;
}

bool isTIM3_EXTI_Enabled(void) {
   // bool nvicEnabled = HAL_NVIC_GetEnableIRQ(TIM3_IRQn) != 0;
    bool timerInterruptEnabled = (TIM3->DIER & TIM_DIER_UIE) != 0U;
    return timerInterruptEnabled;
}

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef* hadc) {


	 // read the controller message and modify status bits; USART 6

	 	 // stepper ctrl
         if (check_recv_bit(0)) {
        	 // move left

//        	 if(!isTIM3_EXTI_Enabled()){
//        		 HAL_TIM_Base_Start_IT(&htim3);
//                 set_ccw_dir();
//        	 }


             stepState = 3;
         }
         else if(check_recv_bit(1)) {
        	 // move right

//        	 if(!isTIM3_EXTI_Enabled()){
//        		 HAL_TIM_Base_Start_IT(&htim3);
//                 set_cw_dir();
//        	 }


        	 stepState = 4;
         }
         else if(check_recv_bit(2)) {
        	 // move forward

//        	 if(!isTIM3_EXTI_Enabled()){
//        		 HAL_TIM_Base_Start_IT(&htim3);
//                 set_forward_dir();
//        	 }


        	 stepState = 1;
         }
         else if(check_recv_bit(3)) {
        	 // move backward

//        	 if(!isTIM3_EXTI_Enabled()){
//        		 HAL_TIM_Base_Start_IT(&htim3);
//                 set_backward_dir();
//        	 }


        	 stepState = 2;
         }
         else {

//        	 if(isTIM3_EXTI_Enabled()){
//            	 HAL_TIM_Base_Stop_IT(&htim3);
//            	 __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
//        	 }

        	 HAL_GPIO_WritePin(ENABLE_GPIO, ENABLE_PIN, 1);
        	 stepState = 0;
         }

         //pan & tilt ctrl
         if(check_recv_bit(4)){
        	 tiltState = 1;
         }
         else if(check_recv_bit(5)){
        	 tiltState = 2;
         }
         else if(check_recv_bit(6)){
        	 shootState = 1;
         }
         else{
        	 tiltState = 0;
         }


	shouldSwitch = true;


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
