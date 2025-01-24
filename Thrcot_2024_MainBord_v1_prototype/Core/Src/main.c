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
#include <stdlib.h>
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
	AUDIO_CHECK,
} Debug_Select_t;

typedef enum {
	M1,
	M2,
	M3,
	M4
} MOTOR_NUM;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define P_GAIN	27.5
#define I_GAIN	0
#define D_GAIN	0

#define ROUND_P_1	30.0
#define ROUND_P_2	50.0
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

uint8_t speaker_volume = 20;

uint8_t check_line[2] = {0xF3, 0x03};
uint8_t request_line[2] = {0xF0, 0x33};

uint16_t Ball_data[8] = {0};
double Sensor_angle[8] = {0.0, 0.785398, 1.570796, 2.356194, 3.141592, 3.926990, 4.712388, 5.497787};

uint8_t __receive_buf[3] = {0xFF, 0xFF, 0xFF};

int8_t m1_offset = 1;
int8_t m2_offset = -1;
int8_t m3_offset = -1;
int8_t m4_offset = 1;
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
void Thrcot_Init_Fast(void);
/*Debug function*/
void Debug_Mode(Debug_Select_t debug_kind);

/*Ball sensor function*/
double Ball_Angle(void);

/*Line sensor function*/
uint8_t Line_Conenct_Test(void);
double Line_Get(void);

/*motor control function*/
void Motor_Init(void);
int Return_Speed(MOTOR_NUM Mx, double angle, int speed);
void M1_control(int speed);
void M2_control(int speed);
void M3_control(int speed);
void M4_control(int speed);
void Stright_control(double angle, int speed);

/*other functions*/
double Return_Pval(double p);

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

  long long gz = 0;
  double angle = 0;

  if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {
	  Thrcot_Init_Fast();
	  while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
  } else {
	  Thrcot_Init();
  }
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
				OLED_Char_Print(" Audio check", 0, 56);
				OLED_Char_Print(">", 0, debug * 8 + 24);
				OLED_Display(&hi2c2);

				if (sw2_state == 0) {
					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
					debug = (debug == AUDIO_CHECK) ? LINE_CHECK : debug + 1;
				}

				if (start_sw_state == 0) {
					while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);
					Debug_Mode(debug);
				}
			}

			robot_state = STANDBY_MODE;
			break;

		case GAME_MODE:
			double Line_angle = 0.0;
			double Ball_angle = 0.0;

			angle = 0.0;

			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);

			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 1) {
				start = DWT -> CYCCNT;

				gz = Gyro_Get_Z() - gz_offset;
				angle += -gz * duration * (2000.0 / 32767.0);

				OLED_DataClear();
				OLED_Char_Print("Game Mode...", 0, 0);
				OLED_Char_Print("Please press StartSW", 0, 8);
				OLED_Int_Print(gz, 0, 16);
				OLED_Double_Print(duration, 0, 24);
				OLED_Double_Print(angle, 0, 32);
				OLED_Display(&hi2c2);

				stop = DWT -> CYCCNT;
				duration = (double)(stop - start) / 180000000.0;
			}

			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);

			HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);

			OLED_DataClear();
			OLED_Char_Print("Running...", 0, 0);
			OLED_Display(&hi2c2);

			while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin)) {
				start = DWT -> CYCCNT;

				double P_val = 0.0;
				double D_val = 0.0;
				static double pre_P_val = 0.0;
				double spin_speed = 0.0;

				double move_angle = 0.0;
				int M1_speed = 0;
				int M2_speed = 0;
				int M3_speed = 0;
				int M4_speed = 0;
				int all_speed[4];
				int max_speed = 0;
				double speed_offset = 0.0;

				gz = Gyro_Get_Z() - gz_offset;
				angle += -1 * gz * duration * (2000.0 / 32767.0);
				if (angle > 180.0) {
					angle = angle - 360.0;
				} else if (angle < -180.0) {
					angle = angle + 360.0;
				}

				P_val = 0.0 - angle;
				D_val = (P_val - pre_P_val) * duration;
				spin_speed = Return_Pval(P_val) + D_val * D_GAIN;

				Ball_angle  = Ball_Angle();

				if (angle >= -20.0 && angle <= 20.0) {
					//M1_speed = M2_speed = M3_speed = M4_speed = spin_speed / 2.0;

					Ball_angle *= 180.0 / PI;

					if (Ball_angle <= 5.0 && Ball_angle >= -5.0) {
						move_angle = Ball_angle;
					} else {
						move_angle = (Ball_angle < 0.0) ? Ball_angle - 45.0 : Ball_angle + 45.0;
					}

					/*M1_speed += Return_Speed(M1, move_angle, 1023);
					M2_speed += Return_Speed(M2, move_angle, 1023);
					M3_speed += Return_Speed(M3, move_angle, 1023);
					M4_speed += Return_Speed(M4, move_angle, 1023);

					all_speed[0] = abs(M1_speed);
					all_speed[1] = abs(M2_speed);
					all_speed[2] = abs(M3_speed);
					all_speed[3] = abs(M4_speed);

					max_speed = all_speed[0];
					for (uint8_t i = 1; i < 4; i++) {
						max_speed = (max_speed < all_speed[i]) ? all_speed[i] : max_speed;
					}

					speed_offset = 1023.0 / max_speed;

					M1_speed *= speed_offset;
					M2_speed *= speed_offset;
					M3_speed *= speed_offset;
					M4_speed *= speed_offset;*/

					Stright_control(move_angle, 1023);
				} else {
					M1_speed = M2_speed = M3_speed = M4_speed = spin_speed;

					M1_control(M1_speed);
					M2_control(M2_speed);
					M3_control(M3_speed);
					M4_control(M4_speed);
				}

				pre_P_val = P_val;

				stop = DWT -> CYCCNT;
				duration = (double)(stop - start) / 180000000.0;
			}
			OLED_DataClear();
			OLED_Double_Print(duration, 0, 0);
			OLED_Display(&hi2c2);

			M1_control(0);
			M2_control(0);
			M3_control(0);
			M4_control(0);

			HAL_Delay(2000);

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
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
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
		Error_Data = BMX055_Init(&hi2c2, 2, 2000);
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
		DFP_Volume(speaker_volume);

		OLED_Char_Print("Complete!", 0, 8);
		OLED_Display(&hi2c2);
		HAL_Delay(1500);
	}

	Motor_Init();
}

void Thrcot_Init_Fast(void)
{
	uint8_t Error_Data = 0;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)Ball_data, 8);
	hdma_adc1.Instance -> CR &= ~(DMA_IT_TC | DMA_IT_HT);

	OLED_Init(&hi2c2, OLED_Init_Data, sizeof(OLED_Init_Data));
	OLED_Thrcot_Large_Logo_Display(&hi2c2);
	HAL_Delay(1000);
	OLED_AllClear(&hi2c2);

	do {
		Error_Data = BMX055_Init(&hi2c2, 2, 2000);
	} while (Error_Data != 0);

	gz_offset = Gyro_Offset_Z(1000);

	Camera_enable = 1;
	DFPlayer_enable = 1;

	DFP_Init(&huart6);
	DFP_Volume(speaker_volume);

	Motor_Init();
}

void Debug_Mode(Debug_Select_t debug_kind)
{
	uint8_t __select_num = 0;
	uint8_t sw2_state, sw3_state, start_sw_state;

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
				OLED_Char_Print(" Reading Line angle", 0, 32);
				OLED_Char_Print(" Reading Line data", 0, 40);
				OLED_Char_Print(">", 0, __select_num * 8 + 24);
				OLED_Display(&hi2c2);

				if (sw2_state == 0) {
					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
					__select_num = (__select_num == 2) ? 0 : __select_num + 1;
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
					} else if (__select_num == 1){
						while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
							double line_angle = Line_Get();

							OLED_DataClear();
							OLED_Char_Print("Reading Line Angle...", 0, 0);
							OLED_Char_Print("Back by SW1", 0, 8);
							OLED_Char_Print("angle:", 0, 56);
							if (line_angle == 300.0) {
								OLED_Char_Print("NonData", 36, 56);
							} else {
								OLED_Double_Print(line_angle, 36, 56);
							}
							OLED_Display(&hi2c2);
						}

						while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);
					} else {
						while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
							Line_Get();

							OLED_DataClear();
							OLED_Char_Print("Reading Line Angle...", 0, 0);
							OLED_Char_Print("Back by SW1", 0, 8);
							OLED_Int_Print(__receive_buf[0], 0, 16);
							OLED_Int_Print(__receive_buf[1], 0, 24);
							OLED_Int_Print(__receive_buf[2], 0, 32);
							OLED_Display(&hi2c2);
						}

						while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);
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
						OLED_Char_Print("Back by SW1", 0, 8);

						if (debug_kind == 0) {
							OLED_Char_Print("Read ADC channel...", 0, 0);
							OLED_Char_Print("ch1:", 0, 24);		OLED_Int_Print(4095 - Ball_data[0], 24, 24);
							OLED_Char_Print("ch2:", 54, 24); 	OLED_Int_Print(4095 - Ball_data[1], 78, 24);
							OLED_Char_Print("ch3:", 0, 32); 	OLED_Int_Print(4095 - Ball_data[2], 24, 32);
							OLED_Char_Print("ch4:", 54, 32);	OLED_Int_Print(4095 - Ball_data[3], 78, 32);
							OLED_Char_Print("ch5:", 0, 40);		OLED_Int_Print(4095 - Ball_data[4], 24, 40);
							OLED_Char_Print("ch6;", 54, 40);	OLED_Int_Print(4095 - Ball_data[5], 78, 40);
							OLED_Char_Print("ch7:", 0, 48);		OLED_Int_Print(4095 - Ball_data[6], 24, 48);
							OLED_Char_Print("ch8:", 54, 48);	OLED_Int_Print(4095 - Ball_data[7], 78, 48);
						} else {
							double Ball_angle = Ball_Angle();
							double angle = Ball_angle * 180.0 / PI;
							int x_pos = 0.0;
							int y_pos = 0.0;

							x_pos = 63.0 + (20.0 * sin(Ball_angle));
							y_pos = 39.0 - (20.0 * cos(Ball_angle));

							OLED_Char_Print("Ball Angle...", 0, 0);
							OLED_Char_Print("angle:", 0, 56);
							OLED_Double_Print(angle, 36, 56);
							OLED_Vartical_Display(16, 63, 63);
							OLED_Horizontal_Display(0, 127, 39);
							OLED_Circle_Draw(63, 39, 20);
							OLED_Circle_Draw(x_pos, y_pos, 5);
						}

						OLED_Display(&hi2c2);
					}

					while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);
				}
			}

			break;

		case MOTOR_CHECK:
			while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
				sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
				sw3_state = HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin);
				start_sw_state = HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin);

				OLED_DataClear();
				OLED_Char_Print("Select by SW2", 0, 0);
				OLED_Char_Print("Forward by StartSW", 0, 8);
				OLED_Char_Print("Back by SW1", 0, 16);
				OLED_Char_Print(" Motor1", 0, 24);
				OLED_Char_Print((m1_offset == 1) ? "forward" : "backward", 50, 24);
				OLED_Char_Print(" Motor2", 0, 32);
				OLED_Char_Print((m2_offset == 1) ? "forward" : "backward", 50, 32);
				OLED_Char_Print(" Motor3", 0, 40);
				OLED_Char_Print((m3_offset == 1) ? "forward" : "backward", 50, 40);
				OLED_Char_Print(" Motor4", 0, 48);
				OLED_Char_Print((m4_offset == 1) ? "forward" : "backward", 50, 48);
				OLED_Char_Print(" All Motor", 0, 56);
				OLED_Char_Print(">", 0, __select_num * 8 + 24);
				OLED_Display(&hi2c2);

				if (sw2_state == 0) {
					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
					__select_num = (__select_num == 4) ? 0 : __select_num + 1;
				}

				if (sw3_state == 0) {
					while (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0);

					switch (__select_num) {
						case 0:
							m1_offset *= -1;
							break;

						case 1:
							m2_offset *= -1;
							break;

						case 2:
							m3_offset *= -1;
							break;

						case 3:
							m4_offset *= -1;
							break;
					}
				}

				if (start_sw_state == 0) {
					switch (__select_num) {
						case 0:
							M1_control(1023);
							M2_control(0);
							M3_control(0);
							M4_control(0);
							break;

						case 1:
							M1_control(0);
							M2_control(1023);
							M3_control(0);
							M4_control(0);
							break;

						case 2:
							M1_control(0);
							M2_control(0);
							M3_control(1023);
							M4_control(0);
							break;

						case 3:
							M1_control(0);
							M2_control(0);
							M3_control(0);
							M4_control(1023);
							break;

						case 4:
							M1_control(1023);
							M2_control(1023);
							M3_control(1023);
							M4_control(1023);
							break;

						default:
							break;
					}
				} else {
					M1_control(0);
					M2_control(0);
					M3_control(0);
					M4_control(0);
				}
			}

			break;

		case AUDIO_CHECK:
			while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
				static uint8_t play_flug = 0;

				sw2_state = HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin);
				start_sw_state = HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin);

				if (DFPlayer_enable == 0) {
					OLED_DataClear();
					OLED_Char_Print("DFPlayer is disabled.", 0, 0);
					OLED_Char_Print("Enable DFPlayer?", 0, 8);
					OLED_Char_Print("Y : SW2 N : SW3", 0, 16);
					OLED_Display(&hi2c2);

					while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 1 && HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 1);

					if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {
						DFPlayer_enable = 1;

						OLED_Char_Print("DFPlayer Init...", 0, 24);
						OLED_Display(&hi2c2);

						DFP_Init(&huart6);
						DFP_Volume(speaker_volume);

						OLED_Char_Print("Complete!", 0, 32);
						OLED_Display(&hi2c2);
						HAL_Delay(1500);
					} else {
						break;
					}
				} else {
					OLED_DataClear();
					OLED_Char_Print("Select by SW2", 0, 0);
					OLED_Char_Print("Decide by StartSW", 0, 8);
					OLED_Char_Print("Back by SW1", 0, 16);
					OLED_Char_Print(" Play Thomas", 0, 24);
					OLED_Char_Print(" Volume change", 0, 32);
					OLED_Char_Print(">", 0, __select_num * 8 + 24);
					OLED_Display(&hi2c2);

					if (sw2_state == 0) {
						while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
						__select_num = (__select_num == 1) ? 0 : __select_num + 1;
					}

					if (start_sw_state == 0) {
						while (HAL_GPIO_ReadPin(Start_sw_GPIO_Port, Start_sw_Pin) == 0);

						if (__select_num == 0) {
							if (play_flug == 0) {
								DFP_Play(7);
								play_flug = 1;
							} else {
								DFP_Pause();
								play_flug = 0;
							}
						} else if (__select_num == 1) {
							while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 1) {
								OLED_DataClear();
								OLED_Char_Print("Save and change : SW1", 0, 0);
								OLED_Char_Print("UP : SW2 DOWN : SW3", 0, 8);
								OLED_Char_Print("volume : ", 0, 16);
								OLED_Int_Print(speaker_volume, 54, 16);
								OLED_Line_Display(19, 35, 109, 35);
								OLED_Circle_Draw(19 + speaker_volume * 3, 35, 4);
								OLED_Display(&hi2c2);

								if (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0) {
									while (HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == 0);
									speaker_volume = (speaker_volume == 30) ? 30 : speaker_volume + 1;
								} else if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0) {
									while (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == 0);
									speaker_volume = (speaker_volume == 0) ? 0 : speaker_volume - 1;
								}
							}

							while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);

							DFP_Volume(speaker_volume);
						}
					}
				}
			}

		default:
			break;
	}

	while (HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == 0);
}

double Ball_Angle(void)
{
	double x_val = 0.0;
	double y_val = 0.0;

	for (int i = 0; i < 8; i++) {
		x_val += cos(Sensor_angle[i]) * (4095 - Ball_data[i]);
		y_val += sin(Sensor_angle[i]) * (4095 - Ball_data[i]);
	}

	if (x_val == 0.0 && y_val == 0.0) {
		return 300.0;
	} else {
		return atan2(y_val, x_val);
	}
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

double Line_Get(void)
{
	//uint8_t __receive_buf[3];

	double x_val = 0.0;
	double y_val = 0.0;

	HAL_UART_Transmit(&huart2, request_line, 2, 1000);
	if (HAL_UART_Receive(&huart2, __receive_buf, 2, 100) == HAL_OK) {
		if (__receive_buf[0] == 0xFF && __receive_buf[1] == 0xFF && __receive_buf[2] == 0xFF) {
			return 300.0;
		} else {
			x_val += cos(0.0) * ((__receive_buf[0] & (1 << 1)) >> 1);
			y_val += sin(0.0) * ((__receive_buf[0] & (1 << 1)) >> 1) ;
			x_val += cos(20.0 * PI / 180.0) * (__receive_buf[0] & (1 << 0));
			y_val += sin(20.0 * PI / 180.0) * (__receive_buf[0] & (1 << 0));

			for (int i = 0; i < 8; i++) {
				x_val += cos((20.0 * (i + 2)) * PI / 180.0) * ((__receive_buf[1] & (1 << i)) >> i);
				y_val += sin((20.0 * (i + 2)) * PI / 180.0) * ((__receive_buf[1] & (1 << i)) >> i);
				x_val += cos((20.0 * (i + 10)) * PI / 180.0) * ((__receive_buf[2] & (1 << i)) >> i);
				x_val += sin((20.0 * (i + 10)) * PI / 180.0) * ((__receive_buf[2] & (1 << i)) >> i);
			}

			return atan2(y_val, x_val) * 180.0 / PI;
		}
	} else {
		return 300.0;
	}
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

int Return_Speed(MOTOR_NUM Mx, double angle, int speed)
{
	int return_speed;

	switch (Mx) {
		case M1:
			return_speed = sin((angle - 45.0) * PI / 180.0) * speed;
			break;

		case M2:
			return_speed = sin((angle - 135.0) * PI / 180.0) * speed;
			break;

		case M3:
			return_speed = sin((angle - 225.0) * PI / 180.0) * speed;
			break;

		case M4:
			return_speed = sin((angle - 315.0) * PI / 180.0) * speed;
			break;
	}

	return return_speed;
}

void M1_control(int speed)
{
	speed *= m1_offset;

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

void M2_control(int speed)
{
	speed *= m2_offset;

	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	} else if (speed > 0 && speed < 1024) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	} else if (speed < 0 && speed > -1024) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1023);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1023);
	}
}

void M3_control(int speed)
{
	speed *= m3_offset;

	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	} else if (speed > 0 && speed < 1024) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	} else if (speed < 0 && speed > -1024) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -speed);
	} else {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1023);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1023);
	}
}

void M4_control(int speed)
{
	speed *= m4_offset;

	if (speed == 0) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
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

void Stright_control(double angle, int speed)
{
	int max = 0;
	int speed_offset = 0;

	int M1_speed = sin((angle - 45) * PI / 180) * speed;
	int M2_speed = sin((angle - 135) * PI / 180) * speed;
	int M3_speed = sin((angle - 225) * PI / 180) * speed;
	int M4_speed = sin((angle - 315) * PI / 180) * speed;

	max = abs(M1_speed);
	if (max < abs(M2_speed)) {
		max = abs(M2_speed);
	}

	if (max < abs(M3_speed)) {
		max = abs(M3_speed);
	}

	if (max < abs(M4_speed)) {
		max = abs(M4_speed);
	}

	speed_offset = speed / max;
	speed_offset = (speed_offset < 0) ? -speed_offset : speed_offset;

	M1_control(M1_speed * speed_offset);
	M2_control(M2_speed * speed_offset);
	M3_control(M3_speed * speed_offset);
	M4_control(M4_speed * speed_offset);
}

double Return_Pval(double p)
{
	if (p <= -20.0) {
		return -P_GAIN * 0.125 * (p + 180.0) + 6.25 * p + 125.0;
	} else if (p >= 20.0) {
		return -P_GAIN * 0.125 * (p - 180.0) + 6.25 * p - 125.0;
	} else {
		return p * P_GAIN;
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
