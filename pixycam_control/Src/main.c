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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef enum {
	IDLE,
	LOST,
	NOT_DETECTED,
	TO_LEFT,
	TO_RIGHT,
	READY
} Track_State;

Track_State state;
int enable = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t tx_blocks_buff[]={0xae,0xc1,32,2,1,1};
uint8_t rx_blocks_buff[20];

void detect_object_location(void);
void capture_and_update(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	state = IDLE;
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  enable = 1;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	capture_and_update();
	if (state == READY) {
		enable = 0;
		break;
	}
	HAL_Delay(100);
  }
  printf("It's ready!");
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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

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

void detect_object_location()
{
	const int sig = 1;
	const int tolerate_x = 30;   // tolerance in x direction in pixels
	const int tolerate_y = 20;   // tolerance in x direction in pixels
	int x_pos, y_pos;

	HAL_UART_Transmit(&huart1, tx_blocks_buff, 6, 1000);
	HAL_UART_Receive(&huart1, rx_blocks_buff, 20, 1000);
	HAL_Delay(10);
	int sanity = 0;
	int if_detected = 0;
	sanity = (rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33);
	if_detected = (rx_blocks_buff[3] == 14);

	for(int i = 0; i< 20; i++){
		printf("%d   ", rx_blocks_buff[i]);
	}
	printf("\n\r");

	// light a led for debug if pixy cam is offline
	if (sanity == 1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);  // switch on the led
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		return;
	}

	if (if_detected == 1) {
		x_pos = rx_blocks_buff[8];
		y_pos = rx_blocks_buff[10];
		printf("target detected at (%d,%d)! \n\r", x_pos, y_pos);
		//
	} else {
		printf("target not detected! \n\r");
		// write to gpio to trigger a interrupt
		return;
	}

	HAL_Delay(100);
}

void capture_and_update() {
	HAL_UART_Transmit(&huart1, tx_blocks_buff, 6, 1000);
	HAL_UART_Receive(&huart1, rx_blocks_buff, 20, 1000);
	HAL_Delay(5);

	for(int i = 0; i< 20; i++){
		printf("%d   ", rx_blocks_buff[i]);
	}

	const int tolerate_x = 30;   // tolerance in x direction in pixels
	const int ref_x = 157;
	const int tolerate_y = 20;   // tolerance in x direction in pixels

	switch (state) {
	case IDLE:
		if (enable) {
			state = LOST;
		}
		break;
	case LOST:
		if ((rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33)) {
			state = NOT_DETECTED;
		}
		if (!enable) {
			state = IDLE;
		}
		break;
	case NOT_DETECTED:
//		if (!enable) {
//			state = IDLE;
//			break;
//		}
//		if ((rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33)) {
//			if (rx_blocks_buff[3] == 14) {
//				// detected
//				int x_pos = rx_blocks_buff[8];
//				int y_pos = rx_blocks_buff[10];
//				printf("target detected at (%d,%d)! \n\r", x_pos, y_pos);
//				if (x_pos > ref_x + tolerate_x) {
//					state = TO_RIGHT;
//				} else if (x_pos < ref_x - tolerate_x) {
//					state = TO_LEFT;
//				} else {
//					state = READY;
//				}
//			} else {
//				state = NOT_DETECTED;
//			}
//		} else {
//			state = LOST;
//		}
//		break;
	case TO_RIGHT:
//			if (!enable) {
//				state = IDLE;
//				break;
//			}
//			if ((rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33)) {
//				if (rx_blocks_buff[3] == 14) {
//					// detected
//					int x_pos = rx_blocks_buff[8];
//					if (x > ref_x + tolerate_x) {
//						state = TO_RIGHT;
//					} else if (x < ref_x - tolerate_x) {
//						state = TO_LEFT;
//					} else {
//						state = READY;
//					}
//				} else {
//					state = NOT_DETECTED;
//				}
//			} else {
//				state = LOST;
//			}
//			break;
	case TO_LEFT:
			if (!enable) {
				state = IDLE;
				break;
			}
			if ((rx_blocks_buff[0] == 175) && (rx_blocks_buff[1] == 193) && (rx_blocks_buff[2] == 33)) {
				if (rx_blocks_buff[3] == 14) {
					// detected
					int x_pos = rx_blocks_buff[8];
					int y_pos = rx_blocks_buff[10];
					printf("target detected at (%d,%d)! \n\r", x_pos, y_pos);
					if (x_pos > ref_x + tolerate_x) {
						state = TO_RIGHT;
					} else if (x_pos < ref_x - tolerate_x) {
						state = TO_LEFT;
					} else {
						state = READY;
					}
				} else {
					state = NOT_DETECTED;
				}
			} else {
				state = LOST;
			}
			break;
	case READY:
		// set the gpio for launching the ball
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		enable = 0;
		state = IDLE;
		break;
	}

	printf("current state is %d \n\r", state);

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
