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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1415926535
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEG_TO_RAD(x)	(x * PI / 180.0)
#define RAD_TO_DEG(x)	(x * 180.0 / PI)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t LinePort[18] = {
		(uint32_t)(LS1_GPIO_Port),
		(uint32_t)(LS18_GPIO_Port),
		(uint32_t)(LS17_GPIO_Port),
		(uint32_t)(LS16_GPIO_Port),
		(uint32_t)(LS15_GPIO_Port),
		(uint32_t)(LS14_GPIO_Port),
		(uint32_t)(LS13_GPIO_Port),
		(uint32_t)(LS12_GPIO_Port),
		(uint32_t)(LS11_GPIO_Port),
		(uint32_t)(LS10_GPIO_Port),
		(uint32_t)(LS9_GPIO_Port),
		(uint32_t)(LS8_GPIO_Port),
		(uint32_t)(LS7_GPIO_Port),
		(uint32_t)(LS6_GPIO_Port),
		(uint32_t)(LS5_GPIO_Port),
		(uint32_t)(LS4_GPIO_Port),
		(uint32_t)(LS3_GPIO_Port),
		(uint32_t)(LS2_GPIO_Port)
};

uint16_t LinePin[18] = {
		(uint16_t)(LS1_Pin),
		(uint16_t)(LS18_Pin),
		(uint16_t)(LS17_Pin),
		(uint16_t)(LS16_Pin),
		(uint16_t)(LS15_Pin),
		(uint16_t)(LS14_Pin),
		(uint16_t)(LS13_Pin),
		(uint16_t)(LS12_Pin),
		(uint16_t)(LS11_Pin),
		(uint16_t)(LS10_Pin),
		(uint16_t)(LS9_Pin),
		(uint16_t)(LS8_Pin),
		(uint16_t)(LS7_Pin),
		(uint16_t)(LS6_Pin),
		(uint16_t)(LS5_Pin),
		(uint16_t)(LS4_Pin),
		(uint16_t)(LS3_Pin),
		(uint16_t)(LS2_Pin),
};

uint8_t rx_buf[10];
uint8_t return_id[2] = {0xF3, 0x03};
uint8_t return_data[2] = {0xFF, 0xFF};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  double x_val = 0.0;
	  double y_val = 0.0;

	  for (int i = 0; i < 18; i++) {
		  uint8_t LineState = HAL_GPIO_ReadPin((GPIO_TypeDef*)(LinePort[i]), LinePin[i]);

		  if (LineState == 0) {
			  x_val += cos(DEG_TO_RAD((360.0 / (double)(i * 20))));
			  y_val += sin(DEG_TO_RAD((360.0 / (double)(i * 20))));
		  }
	  }

	  if (x_val == 0.0 && y_val == 0.0) {
		  return_data[0] = 0xFF;
		  return_data[1] = 0xFF;
	  } else {
		  int line_angle = RAD_TO_DEG(atan2(y_val, x_val));
		  return_data[0] = (line_angle < 0) ? 1 : 0;
		  return_data[1] = (line_angle < 0) ? -line_angle : line_angle;
	  }

	  HAL_UART_Receive_IT(&huart1, rx_buf, 2);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LS8_Pin LS9_Pin LS10_Pin LS11_Pin
                           LS12_Pin LS13_Pin LS14_Pin LS15_Pin
                           LS16_Pin LS17_Pin LS18_Pin */
  GPIO_InitStruct.Pin = LS8_Pin|LS9_Pin|LS10_Pin|LS11_Pin
                          |LS12_Pin|LS13_Pin|LS14_Pin|LS15_Pin
                          |LS16_Pin|LS17_Pin|LS18_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LS1_Pin LS2_Pin LS3_Pin LS4_Pin
                           LS5_Pin LS6_Pin LS7_Pin */
  GPIO_InitStruct.Pin = LS1_Pin|LS2_Pin|LS3_Pin|LS4_Pin
                          |LS5_Pin|LS6_Pin|LS7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LD1_Pin */
  GPIO_InitStruct.Pin = LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);

	if (rx_buf[0] == 0xF3 && rx_buf[1] == 0x03) {
		HAL_UART_Transmit(&huart1, return_id, 2, 1000);
	} else if (rx_buf[0] == 0xF0 && rx_buf[1] == 0x33) {
		HAL_UART_Transmit(&huart1, return_data, 2, 1000);
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
