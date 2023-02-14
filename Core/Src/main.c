/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
//#include "nrf24l01.h"
#include "nrf24.h"
#include "lcd_i2c.h"
#include "vl6180.h"
#include "mpu6050.h"
#include "bmp280.h"
#include "add.h"
#include "radio_type.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	ADC_CHANNEL_BATTERY,
	ADC_CHANNELS_NUM
} adc_channels;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MIN_ESC						 						     80
#define MAX_ESC													 230

#define MIN_PISTON_OFFSET										 3
#define MAX_PISTON_OFFSET										 80
#define GUARD_COUNTER_S(x)										 ((x) * 100)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim3_ch3;

UART_HandleTypeDef huart1;

/* Definitions for motors */
osThreadId_t motorsHandle;
const osThreadAttr_t motors_attributes = {
  .name = "motors",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for radio */
osThreadId_t radioHandle;
const osThreadAttr_t radio_attributes = {
  .name = "radio",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for syringe */
osThreadId_t syringeHandle;
const osThreadAttr_t syringe_attributes = {
  .name = "syringe",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for sensors */
osThreadId_t sensorsHandle;
const osThreadAttr_t sensors_attributes = {
  .name = "sensors",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for radioMessage */
osMessageQueueId_t radioMessageHandle;
const osMessageQueueAttr_t radioMessage_attributes = {
  .name = "radioMessage"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void motorsTask(void *argument);
void radioTask(void *argument);
void syringeTask(void *argument);
void sensorTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint64_t RxpipeAddrs = 0x11223344AA;
char myRxData[32];

uint32_t counter = 0;

mpu6050_t mpu6050;

uint16_t adcData[ADC_CHANNELS_NUM];
uint16_t adcVoltagemV[ADC_CHANNELS_NUM];
float adcVoltage[ADC_CHANNELS_NUM];

uint8_t piston_offset = 0;
int16_t temp_inside = 0;
uint32_t pressure_inside = 0;

int32_t temp_outside = 0;
uint64_t pressure_outside = 0;

volatile uint8_t rx_irq = 0;
uint8_t nrf_data[32] = {0};

//const uint64_t pipe0 = 0x787878787878;
const uint64_t pipe1 = 0xE8E8F0F0E2LL; // адрес первой трубы
//const uint64_t pipe2 = 0xE8E8F0F0A2LL;
//const uint64_t pipe3 = 0xE8E8F0F0D1LL;
//const uint64_t pipe4 = 0xE8E8F0F0C3LL;
const uint64_t pipe5 = 0xE8E8F0F0E7LL;

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
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  power_on_waiting();

/*
  NRF24_begin(GPIOA, GPIO_PIN_4, GPIO_PIN_3, hspi1);
  nrf24_DebugUART_Init(huart1);


  NRF24_setAutoAck(true);
  NRF24_setChannel(52);
  NRF24_setPayloadSize(32);
  NRF24_openReadingPipe(1, RxpipeAddrs);
  NRF24_enableDynamicPayloads();
  NRF24_enableAckPayload();
  NRF24_startListening();
*/

    nrf_init(&hspi1, RF24_PA_MAX, RF24_250KBPS, 120, &huart1);

	nrf_openReadingPipe(1, pipe1);
	nrf_openReadingPipe(5, pipe5);
	nrf_startListening();
/*
  lcd_init(&hi2c1);
  lcd_clear();
*/
  mpu6050_init(&hi2c1);

  BMP280_Init(&hi2c1);

  bmp280_config conf;
  conf.filter = FILTER_16;
  conf.inactive = INACTIVE_125;
  conf.mode = MODE_NORMAL;
  conf.osrs_p = OVERSAMPLING_16;
  conf.osrs_t = OVERSAMPLING_16;
  BMP280_SetConfig(conf);

  vl6180_init(&hi2c1);

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of radioMessage */
  radioMessageHandle = osMessageQueueNew (8, 32, &radioMessage_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motors */
  motorsHandle = osThreadNew(motorsTask, NULL, &motors_attributes);

  /* creation of radio */
  radioHandle = osThreadNew(radioTask, NULL, &radio_attributes);

  /* creation of syringe */
  syringeHandle = osThreadNew(syringeTask, NULL, &syringe_attributes);

  /* creation of sensors */
  sensorsHandle = osThreadNew(sensorTask, NULL, &sensors_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim2.Init.Prescaler = 1000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BUILDIN_GPIO_Port, LED_BUILDIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRF_CE_Pin|NRF_CSN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, WATER_PUMP_A_Pin|WATER_PUMP_B_Pin|CAMERA_Pin|POWER_DEVICE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILDIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILDIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILDIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_BUILDIN_Pin */
  GPIO_InitStruct.Pin = BUTTON_BUILDIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_BUILDIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : WATER_PUMP_A_Pin WATER_PUMP_B_Pin CAMERA_Pin POWER_DEVICE_Pin */
  GPIO_InitStruct.Pin = WATER_PUMP_A_Pin|WATER_PUMP_B_Pin|CAMERA_Pin|POWER_DEVICE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : HALL_SENSOR_Pin */
  GPIO_InitStruct.Pin = HALL_SENSOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(HALL_SENSOR_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  if(hadc->Instance == ADC1)
  {
/*    for (uint8_t i = 1; i < ADC_CHANNELS_NUM+1; i++)
    {
      adcVoltage[i-1] = adcData[i-1] * 3.3 / 4095;
    }*/
	  adcVoltagemV[0] = adcData[0] * 3300 / 4095 * 375 / 75;

	  HAL_ADC_Stop_DMA(&hadc1);
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == NRF_IRQ_EXTI_IRQn)
	{
		HAL_UART_Transmit(&huart1, (uint8_t*)"IRQ\n", strlen("IRQ\n"), 1000);
		rx_irq = 1;
	}
}

void set_pump_direction(pump_states_t state){
	if (state == PUMP_PULL){
		HAL_GPIO_WritePin(GPIOB, WATER_PUMP_A_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, WATER_PUMP_B_Pin, GPIO_PIN_RESET);
	} else if (state == PUMP_PUSH){
		HAL_GPIO_WritePin(GPIOB, WATER_PUMP_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, WATER_PUMP_B_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, WATER_PUMP_A_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, WATER_PUMP_B_Pin, GPIO_PIN_RESET);
	}
}

void power_on_waiting(void){
	if (HAL_GPIO_ReadPin(GPIOB, HALL_SENSOR_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_WritePin(GPIOB, POWER_DEVICE_Pin, GPIO_PIN_RESET);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_motorsTask */
/**
  * @brief  Function implementing the motors thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_motorsTask */
void motorsTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  static uint8_t left_motor = MIN_ESC;
  static uint8_t right_motor = MIN_ESC;
/*
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MAX_ESC);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MAX_ESC);

  osDelay(6000);
*/
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, MIN_ESC);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MIN_ESC);

  osDelay(6000);

  /* Infinite loop */
  for(;;)
  {

	left_motor = map(nrf_data[LEFT_MOTOR], 0, 255, 80, 230);
	right_motor = map(nrf_data[RIGHT_MOTOR], 0, 255, 80, 230);

//	htim2.Instance->CCR1 = counter;
//	htim3.Instance->CCR1 = counter;
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, left_motor);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, right_motor);

	if (nrf_data[CAMERA_COMMAND] == CAMERA_ON){
		HAL_GPIO_WritePin(GPIOB, CAMERA_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOB, CAMERA_Pin, GPIO_PIN_RESET);
	}

    osDelay(100);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_radioTask */
/**
* @brief Function implementing the radio thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_radioTask */
void radioTask(void *argument)
{
  /* USER CODE BEGIN radioTask */
	char str[64];
	//pressure p;
  //char radioMessage[32] = {0};
  //osMessageQueueGet(radioMessageHandle, &radioMessage, NULL, 200);
  /* Infinite loop */
  for(;;)
  {

	  	uint8_t nrf_ack[32] = {0};
	  	uint8_t pipe_num = 0;

	  	if(nrf_available(&pipe_num)/* && rx_irq == 1*/) // проверяем пришло ли что-то
	  	{
	  		rx_irq = 0;

	  	  //clear array
	  	  for (uint8_t i = 0; i < 32; i++){
	  		  nrf_data[i] = 0;
	  	  }

			for (uint8_t i = 0; i < 32; i++){
				nrf_ack[i] = 0;
			}

			//TODO: try to call nrf_read before nrf_writeAckPayload
			nrf_ack[TEMP_INSIDE_MSB] = nrf_data[TEST_BYTE];

			nrf_ack[TEMP_INSIDE_MSB] = temp_inside;
			nrf_ack[TEMP_INSIDE_LSB] = temp_inside >> 8;

			nrf_ack[PRESSURE_INSIDE_0] = (uint8_t)(pressure_inside >>  0);
			nrf_ack[PRESSURE_INSIDE_1] = (uint8_t)(pressure_inside >>  8);
			nrf_ack[PRESSURE_INSIDE_2] = (uint8_t)(pressure_inside >> 16);
			nrf_ack[PRESSURE_INSIDE_3] = (uint8_t)(pressure_inside >> 24);

			nrf_ack[PISTON_OFFSET] = piston_offset;

			nrf_ack[BATTERY_VOLTAGE_MSB] = adcVoltagemV[ADC_CHANNEL_BATTERY];
			nrf_ack[BATTERY_VOLTAGE_LSB] = adcVoltagemV[ADC_CHANNEL_BATTERY] >> 8;

			nrf_ack[X_AXIS_MSB] = (int16_t)mpu6050.KalmanAngleX;
			nrf_ack[X_AXIS_LSB] = ((int16_t)mpu6050.KalmanAngleX) >> 8;

			nrf_ack[Y_AXIS_MSB] = (int16_t)mpu6050.KalmanAngleY;
			nrf_ack[Y_AXIS_LSB] = ((int16_t)mpu6050.KalmanAngleY) >> 8;


	  		nrf_writeAckPayload(pipe_num, &nrf_ack, sizeof(nrf_ack)); // отправляем полезную нагрузку вместе с подтверждением

	  		if(pipe_num == 0)
	  		{
//	  			HAL_UART_Transmit(&huart1, (uint8_t*)"pipe 0\n", strlen("pipe 0\n"), 1000);
	  		}

	  		else if(pipe_num == 1)
	  		{
//	  			HAL_UART_Transmit(&huart1, (uint8_t*)"pipe 1\n", strlen("pipe 1\n"), 1000);

	  			uint8_t count = nrf_getDynamicPayloadSize(); // смотрим сколько байт прилетело

	  			nrf_read(&nrf_data, 32/*count*/); // Читаем данные в массив nrf_data и указываем сколько байт читать


	  				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  				snprintf(str, 128, "FORWARD=%d BACKWARD=%d LEFT=%d RIGHT=%d DEPHT=%d LEFT_BUT=%d RIGHT_BUT=%d\n", nrf_data[FORWARD], nrf_data[BACKWARD], nrf_data[LEFT_MOTOR], nrf_data[RIGHT_MOTOR], nrf_data[DEPHT_OFFSET], nrf_data[LEFT_BUT], nrf_data[RIGHT_BUT]);
//	  				HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

	  		}
/*
	  		else if(pipe_num == 2)
	  		{
	  			HAL_UART_Transmit(&huart1, (uint8_t*)"pipe 2\n", strlen("pipe 2\n"), 1000);
	  		}

	  		else if(pipe_num == 5)
	  		{
	  			HAL_UART_Transmit(&huart1, (uint8_t*)"pipe 5\n", strlen("pipe 5\n"), 1000);
	  			uint8_t count = nrf_getDynamicPayloadSize(); // смотрим сколько байт прилетело

				nrf_read(&nrf_data, count); // Читаем данные в массив nrf_data и указываем сколько байт читать


					HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
					snprintf(str, 64, "data[0]=%d data[1]=%d data[2]=%d\n", nrf_data[0], nrf_data[1], nrf_data[2]);
					HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), 1000);

	  		}

	  		else
	  		{
	  			while(nrf_availableMy()) // если данные придут от неуказанной трубы, то попадут сюда
	  			{
	  				nrf_read(&nrf_data, sizeof(nrf_data));
	  				HAL_UART_Transmit(&huart1, (uint8_t*)"Unknown pipe\n", strlen("Unknown pipe\n"), 1000);
	  			}
	  		}*/
	  	}





/*
	if(NRF24_available())
	{
	  printStatusReg();

	  printConfigReg();

	  NRF24_read(myRxData, 32);
	  sprintf(myAckPayload, "%d,%d,%d,%d,%d,%d", 1, 0 ,0, piston_offset, temp_inside, pressure_inside);
	  NRF24_writeAckPayload(1, myAckPayload, 32);
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  //myRxData[32] = '\r'; myRxData[32+1] = '\n';
	  HAL_UART_Transmit(&huart1, (uint8_t *)"Received: ", strlen("Received: "), 10);
	  HAL_UART_Transmit(&huart1, (uint8_t *)myRxData, strlen(myRxData), 10);
	  osMessageQueuePut(radioMessageHandle, &myRxData, 0, 200);
	}*/
	osDelay(1);
  }
  /* USER CODE END radioTask */
}

/* USER CODE BEGIN Header_syringeTask */
/**
* @brief Function implementing the syringe thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_syringeTask */
void syringeTask(void *argument)
{
  /* USER CODE BEGIN syringeTask */
  static uint8_t guard_counter = 0;
  /* Infinite loop */
  for(;;)
  {

	 if (piston_offset > MIN_PISTON_OFFSET && piston_offset < MAX_PISTON_OFFSET){
		 if (++guard_counter < GUARD_COUNTER_S(10)){
			 set_pump_direction((pump_states_t)nrf_data[DEPHT_OFFSET]);
		 }
		 else {
			 guard_counter = 0;
		 }
	 } else {
		 set_pump_direction(PUMP_OFF);
		 guard_counter = 0;
	 }


    osDelay(10);
  }
  /* USER CODE END syringeTask */
}

/* USER CODE BEGIN Header_sensorTask */
/**
* @brief Function implementing the sensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorTask */
void sensorTask(void *argument)
{
  /* USER CODE BEGIN sensorTask */
  char str[32] = {0};
  /* Infinite loop */
  for(;;)
  {
	mpu6050_read_all(&mpu6050);

	sprintf(str, "x:%.2f y:%.2f ", mpu6050.KalmanAngleX, mpu6050.KalmanAngleY);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);

	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcData, ADC_CHANNELS_NUM);

	temp_inside = (uint16_t)BMP280_GetTemperature();

	pressure_inside = (uint32_t)(BMP280_GetPressure()/256);

	piston_offset = get_distance_mm();


	sprintf(str, "T: %d, P: %d, d: %d mm\r\n", temp_inside, pressure_inside, piston_offset);
	HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), 100);

	osDelay(100);
  }
  /* USER CODE END sensorTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
