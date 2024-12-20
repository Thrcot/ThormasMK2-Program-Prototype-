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
#include <math.h>
#include "ssd1306_hal.h"
#include "DFPlayer_hal.h"
#include "BMX055_hal.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STANDBY_MODE,
	DEBUG_MODE,
	GAME_MODE
} Robot_State_t;

typedef enum {
	LINE_CHECK,
	BALL_CHECK,
	BMX_CHECK,
	MOTOR_CHECK,
} Debug_Select_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */
double gz_offset = 0.0;

uint8_t DFPlayer_enable = 0;
uint8_t Camera_enable = 0;
uint8_t WiFi_enable = 0;

uint8_t check_line[2] = {0xF3, 0x03};
uint8_t request_line[2] = {0xF0, 0x33};

uint16_t Ball_data[8] = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART5_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/*Init function*/
void Thrcot_Init(void);

/*Debug function*/
void Debug_Mode(Debug_Select_t debug_kind);
uint8_t Line_Conenct_Test(void);
/*switch control function*/
int SW1_keep_state(void);
int SW2_keep_state(void);
int SW3_keep_state(void);
int StartSW_keep_state(void);

/*ADC and Ball sensor function*/


/*motor control function*/
void Motor_Init(void);
void M1_control(int speed);
void M2_control(int speed);
void M3_control(int speed);
void M4_control(int speed);
void Stright_control(double angle, int speed);
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  volatile uint32_t start = 0;
  volatile uint32_t stop = 0;

  double duration = 0.0;

  int gz = 0;
  double angle = 0;

  Thrcot_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static Robot_State_t robot_state = STANDBY_MODE;
	  uint8_t sw1_state, sw2_state, sw3_state, start_sw_state;

	  switch (robot_state) {
		case STANDBY_MODE:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

			OLED_DataClear();
			OLED_Char_Print("Stand-by Mode...", 0, 0);
			OLED_Char_Print("Debug Mode : SW2", 0, 12);
			OLED_Char_Print("Game Mode : SW3", 0, 20);
			OLED_Display(&hi2c2);

			while (1) {
				if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {
					robot_state = DEBUG_MODE;
					break;
				} else if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0){
					robot_state = GAME_MODE;
					break;
				}
			}
			break;

		case DEBUG_MODE:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

			OLED_AllClear(&hi2c2);
			OLED_Char_Print("Debug Mode...", 0, 0);
			OLED_Char_Print("SW1 : Stand-by Mode", 0, 8);
			OLED_Display(&hi2c2);
			HAL_Delay(1500);

			while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
				static Debug_Select_t debug = 0;
				sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
				start_sw_state = HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin);

				OLED_DataClear();
				OLED_Char_Print("Select to SW2", 0, 0);
				OLED_Char_Print("Decide to StartSW", 0, 8);
				OLED_Char_Print("Stand-by to SW1", 0, 16);
				OLED_Char_Print(" Line check", 0, 24);
				OLED_Char_Print(" Ball check", 0, 32);
				OLED_Char_Print(" BMX check", 0, 40);
				OLED_Char_Print(" Motor check", 0, 48);
				OLED_Char_Print(">", 0, debug * 8 + 24);
				OLED_Display(&hi2c2);

				if (sw2_state == 0) {
					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
					debug = (debug == MOTOR_CHECK) ? LINE_CHECK : debug + 1;
				}

				if (start_sw_state == 0) {
					while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);
					Debug_Mode(debug);
				}
			}

			robot_state = STANDBY_MODE;
			break;

		case GAME_MODE:
			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

			OLED_DataClear();
			OLED_Char_Print("Game Mode...", 0, 0);
			OLED_Char_Print("Please press StartSW", 0, 8);
			OLED_Display(&hi2c2);

			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 1);
			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);

			OLED_DataClear();
			OLED_Char_Print("Running...", 0, 0);
			OLED_Display(&hi2c2);

			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin)) {

			}

			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);
			robot_state = STANDBY_MODE;

			break;

		default:
			break;
	}
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1023;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1023;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1023;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LD1_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Shot_GPIO_Port, Shot_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Start_sw_Pin */
  GPIO_InitStruct.Pin = Start_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Start_sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin SW1_Pin SW3_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|SW1_Pin|SW3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Shot_Pin */
  GPIO_InitStruct.Pin = Shot_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Shot_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Thrcot_Init(void)
{
	uint8_t Error_Data = 0;
	uint8_t loop_flug = 0;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Ball_data, 8);
	hdma_adc1.Instance -> CR &= ~(DMA_IT_TC | DMA_IT_HT);

	OLED_Init(&hi2c2, OLED_Init_Data, sizeof(OLED_Init_Data));
	OLED_Thrcot_Large_Logo_Display(&hi2c2);
	HAL_Delay(2000);
	OLED_AllClear(&hi2c2);

	OLED_Char_Print("BMX055...", 0, 0);
	OLED_Display(&hi2c2);
	do {
		Error_Data = BMX055_Init(&hi2c2, 2, 500);
	} while (Error_Data != 0);

	OLED_Char_Print("         OK", 0, 0);
	OLED_Display(&hi2c2);
	HAL_Delay(550);

	OLED_Char_Print("Calibration...", 0, 8);
	OLED_Display(&hi2c2);

	gz_offset = Gyro_Offset_Z(1000);

	OLED_Char_Print("BMX055 Init Complete!", 0, 16);
	OLED_Display(&hi2c2);
	HAL_Delay(2000);

	do {
		uint8_t sw2_state = 1;
		uint8_t sw3_state = 1;

		OLED_DataClear();
		OLED_Char_Print("Do you use Camera?", 0, 0);
		OLED_Char_Print("Y : SW2  N : SW3", 0, 8);
		OLED_Display(&hi2c2);

		while (sw2_state == 1 && sw3_state == 1) {
			sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
			sw3_state = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
		}
		while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0 || HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0);

		if (sw2_state == 0) {
			Camera_enable = 1;
			OLED_Char_Print("Camera enable, OK?", 0, 16);
		} else {
			Camera_enable = 0;
			OLED_Char_Print("Camera disable, OK?", 0, 16);
		}

		OLED_Char_Print("Y : SW2  N : SW3", 0, 24);
		OLED_Display(&hi2c2);

		sw2_state = sw3_state = 1;

		while (sw2_state == 1 && sw3_state == 1) {
			sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
			sw3_state = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
		}

		while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0 || HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0);

		if (sw2_state == 0) {
			loop_flug = 1;
		} else {
			  loop_flug = 0;
		}
	} while(loop_flug == 0);

	loop_flug = 0;

	do {
		uint8_t sw2_state = 1;
		uint8_t sw3_state = 1;

		OLED_DataClear();
		OLED_Char_Print("Do you use Speaker?", 0, 0);
		OLED_Char_Print("Y : SW2  N : SW3", 0, 8);
		OLED_Display(&hi2c2);

		while (sw2_state == 1 && sw3_state == 1) {
			sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
		  	sw3_state = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
		}
		while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0 || HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0);

		if (sw2_state == 0) {
			DFPlayer_enable = 1;
			OLED_Char_Print("Speaker enable, OK?", 0, 16);
		} else {
			DFPlayer_enable = 0;
			OLED_Char_Print("Speaker disable, OK?", 0, 16);
		}

		OLED_Char_Print("Y : SW2  N : SW3", 0, 24);
		OLED_Display(&hi2c2);

		sw2_state = sw3_state = 1;

		while (sw2_state == 1 && sw3_state == 1) {
		   	sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
		   	sw3_state = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
		}
		while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0 || HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0);

		if (sw2_state == 0) {
			loop_flug = 1;
		} else {
			loop_flug = 0;
		}
	} while (loop_flug == 0);

	if (DFPlayer_enable == 1) {
		OLED_DataClear();
		OLED_Char_Print("DFPlayer Init...", 0, 0);
		OLED_Display(&hi2c2);

		DFP_Init(&huart6);

		OLED_Char_Print("Complete!", 0, 8);
		OLED_Display(&hi2c2);
		HAL_Delay(1500);
	}

	Motor_Init();
}

void Debug_Mode(Debug_Select_t debug_kind)
{
	uint8_t __select_num = 0;
	uint8_t sw2_state, start_sw_state;

	switch (debug_kind) {
		case LINE_CHECK:
			while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
				sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
				start_sw_state = HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin);

				OLED_DataClear();
				OLED_Char_Print("Select by SW2", 0, 0);
				OLED_Char_Print("Decide by StartSW", 0, 8);
				OLED_Char_Print("Back by SW1", 0, 16);
				OLED_Char_Print(" Connecting test", 0, 24);
				OLED_Char_Print(" Reading Line", 0, 32);
				OLED_Char_Print(">", 0, __select_num * 8 + 24);
				OLED_Display(&hi2c2);

				if (sw2_state == 0) {
					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
					__select_num = (__select_num == 1) ? 0 : 1;
				}

				if (start_sw_state == 0) {
					while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);

					if (__select_num == 0) {
						OLED_DataClear();
						OLED_Char_Print("Line Connect test...", 0, 0);
						for (int i = 0; i < 5; i++) {
							OLED_Char_Print("Connecting...", 0, i * 8 + 8);
							if (Line_Conenct_Test() == 0) {
								OLED_Char_Print("Success!", 0, (i + 1) * 8 + 8);
								OLED_Display(&hi2c2);
								HAL_Delay(1000);

								break;
							} else if (i == 4) {
								OLED_Char_Print("Not found!", 0, (i + 1) * 8 + 8);
								OLED_Display(&hi2c2);
								HAL_Delay(1000);
							}

							OLED_Display(&hi2c2);
						}
					}
				}
			}

			break;

		case BALL_CHECK:
			while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
				sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
				start_sw_state = HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin);

				OLED_DataClear();
				OLED_Char_Print("Select by SW2", 0, 0);
				OLED_Char_Print("Decide by StartSW", 0, 8);
				OLED_Char_Print("Back by SW1", 0, 16);
				OLED_Char_Print(" Read all channels", 0, 24);
				OLED_Char_Print(" Ball angle check", 0, 32);
				OLED_Char_Print(">", 0, debug_kind * 8 + 24);
				OLED_Display(&hi2c2);

				if (sw2_state == 0) {
					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
					debug_kind = (debug_kind == 1) ? 0 : 1;
				}

				if (start_sw_state == 0) {
					while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);

					while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
						OLED_DataClear();
						OLED_Char_Print("Read ADC channel...", 0, 0);
						OLED_Char_Print("Back by SW1", 0, 8);

						if (debug_kind == 0) {
							OLED_Char_Print("ch:", 0, 16);	OLED_Int_Print(Ball_data[0], 18, 16);
						} else {

						}

						OLED_Display(&hi2c2);
					}

					while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);
				}
			}

		default:
			break;
	}

	while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);
}

uint8_t Line_Conenct_Test(void)
{
	uint8_t __receive_buf[2];

	HAL_UART_Transmit(&huart2, check_line, 2, 1000);
	if (HAL_UART_Receive(&huart2, __receive_buf, 2, 1000) == HAL_OK) {
		if (__receive_buf[0] == 0xF3 && __receive_buf[1] == 0x03) {
			return 0;
		} else {
			return 1;
		}
	} else {
		return 1;
	}
}

int StartSW_keep_state(void)
{
	static int pre_state = 0;
	static int keep_state = 0;
	int SW_state = 1 - HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin);

	if (SW_state == 1 && pre_state == 0) {
		keep_state = 1 - keep_state;
	}

	pre_state = SW_state;

	return keep_state;
}

int SW1_keep_state(void)
{
	static int pre_state = 0;
	static int keep_state = 0;
	int SW_state = 1 - HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);

	if (SW_state == 1 && pre_state == 0) {
		keep_state = 1 - keep_state;
	}

	pre_state = SW_state;

	return keep_state;
}

int SW2_keep_state(void)
{
	static int pre_state = 0;
	static int keep_state = 0;
	int SW_state = 1 - HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);

	if (SW_state == 1 && pre_state == 0) {
		keep_state = 1 - keep_state;
	}

	pre_state = SW_state;

	return keep_state;
}

int SW3_keep_state(void)
{
	static int pre_state = 0;
	static int keep_state = 0;
	int SW_state = 1 - HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);

	if (SW_state == 1 && pre_state == 0) {
		keep_state = 1 - keep_state;
	}

	pre_state = SW_state;

	return keep_state;
}

void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
}

void M1_control(int speed)
{
	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	} else if (speed > 0 && speed < 1024) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	} else if (speed < 0 && speed > -1024) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1023);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1023);
	}
}

void M4_control(int speed)
{
	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	} else if (speed > 0 && speed < 1024) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
	} else if (speed < 0 && speed > -1024) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, -speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1023);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1023);
	}
}

void M3_control(int speed)
{
	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	} else if (speed > 0 && speed < 1024) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	} else if (speed < 0 && speed > -1024) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1023);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1023);
	}
}

void M2_control(int speed)
{
	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	} else if (speed > 0 && speed < 1024) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	} else if (speed < 0 && speed > -1024) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1023);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1023);
	}
}

void Stright_control(double angle, int speed)
{
	int M1_speed = sin((angle - 45) * PI / 180) * speed;
	int M2_speed = sin((angle - 135) * PI / 180) * speed;
	int M3_speed = sin((angle - 225) * PI / 180) * speed;
	int M4_speed = sin((angle - 315) * PI / 180) * speed;

	M1_control(M1_speed);
	M2_control(M2_speed);
	M3_control(M3_speed);
	M4_control(M4_speed);
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
