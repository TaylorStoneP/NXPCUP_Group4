/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32_utils.h"
#include "control.h"
#include "mathops.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * CHANGE THESE VALUES FOR CONTROL OVER CARS OPERATION
 */
#define DISTANCE_SETPOINT 110	//distance car will stop infront of object in mm.
#define DISTANCE_RANGE 800 		//distance car will detect object in mm and initiate distance based speed control.
#define STEERING_SETPOINT -250	//alignment on track in arbitrary units, don't change if not needed. (+=left, -=right, 0=centre)
#define MAX_SPEED 150			//max speed on straight.
#define MIN_SPEED 100			//slower speed on turns.
#define HARSH_STEER 60
/*
 * PID GAINS - CHANGE IS NOT RECOMMENDED
 */
//DISTANCE SENSOR PID GAINS
#define P_GAIN_DISTANCE	3000
#define I_GAIN_DISTANCE	5
#define D_GAIN_DISTANCE	3000

//STEERING CONTROL PID GAINS
#define P_GAIN_STEERING	400
#define I_GAIN_STEERING	0
#define D_GAIN_STEERING	2800
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint32_t LDR_read[N_LDR_NODES];
PID_Handle hpid_distance;
PID_Handle hpid_steer;
uint8_t distance_override = False;
uint8_t ADC_Pause = False;
uint8_t done = False;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TaskScheduleHandler(){
SCHEDULE_HANDLE(SCH_CALIBRATION_CPLT)
	ScheduleTaskStop(SCH_LED_TOGGLE);
	ScheduleTask(SCH_LDR_CHECK, 20, True, 0);
}
SCHEDULE_HANDLE(SCH_LED_TOGGLE)
	HAL_GPIO_TogglePin(LED_BRD_GPIO_Port, LED_BRD_Pin);
}
SCHEDULE_HANDLE(SCH_LEDTOP)
	HAL_GPIO_WritePin(LED_LDR_L3_GPIO_Port, LED_LDR_L3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LDR_R3_GPIO_Port, LED_LDR_R3_Pin, GPIO_PIN_SET);
	ScheduleTask(SCH_LEDMID, 200, 0, 0);
}
SCHEDULE_HANDLE(SCH_LEDMID)
	HAL_GPIO_WritePin(LED_LDR_L2_GPIO_Port, LED_LDR_L2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LDR_R2_GPIO_Port, LED_LDR_R2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LDR_L3_GPIO_Port, LED_LDR_L3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_LDR_R3_GPIO_Port, LED_LDR_R3_Pin, GPIO_PIN_RESET);
	ScheduleTask(SCH_LEDBOT, 200, 0, 0);
}
SCHEDULE_HANDLE(SCH_LEDBOT)
	HAL_GPIO_WritePin(LED_LDR_L1_GPIO_Port, LED_LDR_L1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LDR_R1_GPIO_Port, LED_LDR_R1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED_LDR_L2_GPIO_Port, LED_LDR_L2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_LDR_R2_GPIO_Port, LED_LDR_R2_Pin, GPIO_PIN_RESET);
	ScheduleTask(SCH_LEDOFF, 200, 0, 0);
}
SCHEDULE_HANDLE(SCH_LEDOFF)
	HAL_GPIO_WritePin(LED_LDR_L1_GPIO_Port, LED_LDR_L1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_LDR_R1_GPIO_Port, LED_LDR_R1_Pin, GPIO_PIN_RESET);
}
SCHEDULE_HANDLE(SCH_LDR_CHECK)
	ADC_Pause = True;

//===STEERING CONTROL===
	int32_t left = (LDR_nodes[2].hrave.average);	//Left LDR adc reading, black is higher value.
	int32_t right = (LDR_nodes[5].hrave.average);	//Right LDR adc reading.
	int32_t position = (left-right);	//Difference between sides, 0 means even light so centre. <0 means right is darker. >0 meaning left is darker.
	int32_t pid = clamp(PID_Compute(&hpid_steer, position), -500, 500);	//Process position in PID controller. Clamped to min and max pwm values.
	MotorControls.steer = 500+pid;	//Set steering variable to mid point (500) + response from
	SetSteer(MotorControls.steer);
//======================

//===SPEED CONTROL===
	int32_t previous_speed = MotorControls.speed;	//Keep track of previous speed for later...
	MotorControls.speed = 500 + MAX_SPEED;			//Set new speed.
	if(distance_override==False){					//If no object in front...
		if(MotorControls.steer>(500+HARSH_STEER) || MotorControls.steer<(500-HARSH_STEER)){	//If steering harshly...
		  MotorControls.speed = clamp(MotorControls.speed, 0, 500+MIN_SPEED);	//Clamp speed to minimum.
		}else{													//Else steering softly...
		  MotorControls.speed = clamp(MotorControls.speed, 0, 1000);	//Clamp speed to absolute pwm maximum. Will never go above Max speed anyway.
		}

		/*
		 * If there is a change in direction, the ESC must pause briefly at 1.5ms pwm pule width before it will act on applying force
		 * in the other diretion or it will become unresponsive. The following code checks to see if there is a change in direction
		 * and if so, sets the speed to 1.5ms pwm pulse width and sets the desired speed after a delay of 5ms.
		 */
		if((previous_speed<500 && MotorControls.speed>500) || (previous_speed>500 && MotorControls.speed<500)){
		  SetSpeed(500);
		  ScheduleTask(SCH_SETSPEED, 5, False, 0);
		}else{
		  ScheduleTask(SCH_SETSPEED, 0, False, 0);
		}
	}
//===================

	//Reset all LDR averaging counts.
	for(int i = 0;i<N_LDR_NODES;i++){
		RUNTIME_AVERAGE_Reset(&LDR_nodes[i].hrave);
	}

	//Begin LDR readings again.
	ADC_Pause = False;
	HAL_ADC_Start_DMA(&hadc1,LDR_read,N_LDR_NODES);
}
SCHEDULE_HANDLE(SCH_SETSPEED)
	SetSpeed(MotorControls.speed);
}

SCHEDULE_HANDLE(SCH_PULSE)
HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
HAL_TIM_Base_Start_IT(&htim4);
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&SOFTCLK_TIMER_TYPE);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);

  ScheduleTask(SCH_LED_TOGGLE, 50, True, 0);
  ScheduleTask(SCH_CALIBRATION_CPLT, 1000, True, 0);

  PID_SetGain(&hpid_distance, P_GAIN_DISTANCE, I_GAIN_DISTANCE, D_GAIN_DISTANCE);
  PID_UpdateSetpoint(&hpid_distance, DISTANCE_SETPOINT);

  PID_SetGain(&hpid_steer, P_GAIN_STEERING, I_GAIN_STEERING, D_GAIN_STEERING);
  PID_UpdateSetpoint(&hpid_steer,STEERING_SETPOINT);

  SetSteer(500);	//>500 left, <500 right.
  SetSpeed(500);	//>500 forward, <500 reverse.

  HAL_Delay(3000);

  ScheduleTask(SCH_PULSE, 40, True, 0);	//Schedule recurring ultrasonic pulse every 40ms.
  HAL_ADC_Start_DMA(&hadc1,LDR_read,N_LDR_NODES);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  TaskScheduleSoftClock();
	  TaskScheduleHandler();
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50000-1;
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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10-1;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BRD_GPIO_Port, LED_BRD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, US_TRIG_Pin|LED_LDR_R3_Pin|LED_LDR_R2_Pin|LED_LDR_R1_Pin
                          |LED_LDR_L3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_LDR_L2_Pin|LED_LDR_L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BRD_Pin */
  GPIO_InitStruct.Pin = LED_BRD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BRD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : US_TRIG_Pin LED_LDR_R3_Pin LED_LDR_R2_Pin LED_LDR_R1_Pin
                           LED_LDR_L3_Pin */
  GPIO_InitStruct.Pin = US_TRIG_Pin|LED_LDR_R3_Pin|LED_LDR_R2_Pin|LED_LDR_R1_Pin
                          |LED_LDR_L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : US_ECHO_Pin */
  GPIO_InitStruct.Pin = US_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US_ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_LDR_L2_Pin LED_LDR_L1_Pin */
  GPIO_InitStruct.Pin = LED_LDR_L2_Pin|LED_LDR_L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    // Read & Update The ADC Result
	for(int i = 0;i<N_LDR_NODES;i++){
		RUNTIME_AVERAGE_Update(&LDR_nodes[i].hrave, LDR_read[i]);
	}
	if(!ADC_Pause){
		HAL_ADC_Start_DMA(&hadc1,LDR_read,N_LDR_NODES);
	}
}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim) {
	if (htim == &SOFTCLK_TIMER_TYPE) {
		TaskScheduleSoftClock_FlagSet();
	}else if(htim == &htim4){
		HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
// Function runs on state change of HC-SR04 Echo pin
switch(GPIO_Pin){
  case US_ECHO_Pin:
	  if(HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin)==GPIO_PIN_SET){ //If echo goes high...
		  TIM3->CNT = 0; // Reset timer count.
		  HAL_TIM_Base_Start(&htim3); // Start echo tracking timer.
	  }else{ //If echo goes low...
		  uint32_t distance = (0.34f * TIM3->CNT)/2; //Calculate distance in mm.
		  if(TIM3->CNT>30000){ //If echo times out...
			  distance_override = False;	//Distance override disabled.
		  }else{
			  if((distance<DISTANCE_RANGE)&&(distance>15)){	//If distance within certain range...
				  if(distance_override==False){		//If distance override disabled...
					  distance_override = True;		//Enable distance override.
					  PID_Reset(&hpid_distance);	//Reset distance PID controller.
				  }
				  int32_t pid = PID_Compute_Distance(&hpid_distance, distance);	//Compute PID with new process variable.
				  uint32_t previous_speed = MotorControls.speed;		//Keep note of previous speed for later...
				  MotorControls.speed = 500-pid;						//Set speed variable with PID output.

				  if(MotorControls.steer>550 || MotorControls.steer<450){		//If steering harshly...
					  MotorControls.speed = clamp(MotorControls.speed, 0, 550);	//Clamp speed to slow.
				  }else{														//Else steering lightly...
					  MotorControls.speed = clamp(MotorControls.speed, 0, 650);	//Let speed go higher.
				  }
				  //If change in force applied to motor...
				  if((previous_speed<500 && MotorControls.speed>500) || (previous_speed>500 && MotorControls.speed<500)){
					  SetSpeed(500);	//Let car roll momentarily with no applied speed
					  ScheduleTask(SCH_SETSPEED, 5, False, 0);	//Set speed after 5ms delay.
				  }else{										//Else direction is the same.
					  ScheduleTask(SCH_SETSPEED, 0, False, 0);	//Set speed without delay.
				  }

				  if(done==False && MotorControls.speed>490 && MotorControls.speed<510){
					  done=True;
					  ScheduleTask(SCH_LEDTOP, 900, True, 0);
				  }
			  }
			  else {	//Else outside range...
				  distance_override = False;	//Distance override disabled.
			  }
		  }
		  HAL_TIM_Base_Stop(&htim3);	//Stop echo tracking timer.
	  }
	  break;
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
