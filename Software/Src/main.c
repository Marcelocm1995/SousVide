/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fonts.h"
#include "ssd1306.h"
#include "test.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define USE_PID
//#define USE_FILTER

#ifdef USE_PID
#define Kp 1.0
#define Ki 1.0
#define Kd 1.0
#define SAMPLE_TIME 1
#endif

#define DS18B20_PORT GPIOA
#define DS18B20_PIN GPIO_PIN_1

//#define DEBUG

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define LED_ON HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED_OFF HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)

#define BEEP_ON HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET)
#define BEEP_OFF HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int TEMPOOO;
int SECOND_TIMER;

int SECOND, MINUTE, HOUR;
int SET_SECOND, SET_MINUTE, SET_HOUR;
int TOTAL_SET_SECOND;
int TOTAL_SET_SECOND_COUNTDOWN;

unsigned int SETPOINT_TEMPERAURE;

float TEMPERATURE, 
			LAST_TEMPERATURE,
			TEMPERATURE_RAW,
			FILTER_FACTOR = 5,
			Ts = 1.0f,
			TEMPERATURE_OFFSET = 0.2f;

float ERROR_MEASURED;
float PROPORTIONAL;
float INTEGRATOR;
float DERIVATIVE;
float PID;

int BLYNK_OLED;

int encoderCount, encoderDirection;
int SW_ALREADY_PRESSED;


char strbuffer[64];

int liga = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

float Temp_Read(void);
void heat_function(void);
void cook_function(void);
void delay_us (uint16_t us);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);
void beep(int delay);

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

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
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(1000);
		
	char strBuffer[] = "Ola\n\r";
	CDC_Transmit_FS((uint8_t*)strBuffer, strlen(strBuffer));
	
	SSD1306_Init();  // initialise
	SSD1306_Clear();
	
	beep(200);
	   
	SSD1306_GotoXY (2,0);
  SSD1306_Puts("Sous", &Font_16x26, 1);
  SSD1306_GotoXY (60, 30);
  SSD1306_Puts("Vide", &Font_16x26, 1);
  SSD1306_UpdateScreen(); //display

	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	__HAL_TIM_SET_AUTORELOAD(&htim4, 4*2);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	
	LED_ON;
	HAL_Delay(2000);
	SSD1306_Clear();
	
	HAL_TIM_Base_Start(&htim2);
	
	LED_OFF;
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		encoderCount = __HAL_TIM_GET_COUNTER(&htim4) / 4;
		SSD1306_GotoXY (22,0);
		SSD1306_Puts("Aquecer", &Font_11x18, 1);
		SSD1306_GotoXY (22,30);
		SSD1306_Puts("Cozinhar", &Font_11x18, 1);
		
		char strBuffer[] = "Ola\n\r";
		CDC_Transmit_FS((uint8_t*)strBuffer, strlen(strBuffer));
		
		if(encoderCount == 1)
		{
			SSD1306_GotoXY (0,0);
			SSD1306_Puts("->", &Font_11x18, 1);
			SSD1306_GotoXY (0,30);
			SSD1306_Puts("  ", &Font_11x18, 1);
		}
		
		if(encoderCount == 0)
		{
			SSD1306_GotoXY (0,30);
			SSD1306_Puts("->", &Font_11x18, 1);
			SSD1306_GotoXY (0,0);
			SSD1306_Puts("  ", &Font_11x18, 1);
		}
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
				SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				HAL_Delay(100);
				if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
				{
					SW_ALREADY_PRESSED = 1;
					beep(200);
					if(encoderCount == 1)
					{
						SSD1306_Clear();
						heat_function();
						__HAL_TIM_SET_COUNTER(&htim4, 0);
					}
					else
					{
						SSD1306_Clear();
						cook_function();
						__HAL_TIM_SET_COUNTER(&htim4, 0);
					}
				}
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
  hi2c1.Init.ClockSpeed = 400000;
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
  htim1.Init.Prescaler = 4800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000-1;
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff-1;
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
  htim3.Init.Prescaler = 4800-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
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
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW_ENC_Pin */
  GPIO_InitStruct.Pin = SW_ENC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW_ENC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void heat_function(void)
{
	int STAGE = 0;
	
	sprintf(strbuffer, "Selecione a");
	SSD1306_GotoXY(2, 0);
	SSD1306_Puts(strbuffer, &Font_11x18, 1);
	
	sprintf(strbuffer, "Temperatura");
	SSD1306_GotoXY(2, 20);
	SSD1306_Puts(strbuffer, &Font_11x18, 1);
	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim4, (4 * 100)-1);
  while(STAGE == 0)
	{
		encoderCount = __HAL_TIM_GET_COUNTER(&htim4)/4;	
		sprintf(strbuffer, "%02i C",encoderCount);
		SSD1306_GotoXY(40, 42);
		SSD1306_Puts(strbuffer, &Font_11x18, 0);
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(100);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				STAGE++;
				SETPOINT_TEMPERAURE = encoderCount;
				SSD1306_Clear();
			}
		}
	}
	
	while(STAGE == 1)
	{
		int t_in = HAL_GetTick();
		
		#ifdef USE_FILTER
		TEMPERATURE_RAW = Temp_Read();
		TEMPERATURE =  ((FILTER_FACTOR * Ts * TEMPERATURE_RAW) + (LAST_TEMPERATURE)) /( 1 + (FILTER_FACTOR * Ts));
    LAST_TEMPERATURE = TEMPERATURE;
		
		#else
		TEMPERATURE = Temp_Read();
		
		#endif
		
		#ifdef USE_PID
    ERROR_MEASURED = SETPOINT_TEMPERAURE - TEMPERATURE;
    
    PROPORTIONAL = Kp * ERROR_MEASURED;
    INTEGRATOR += (Ki * ERROR_MEASURED) * SAMPLE_TIME;
    DERIVATIVE = ((TEMPERATURE - LAST_TEMPERATURE) * Kd) / SAMPLE_TIME;
   
		if(INTEGRATOR > 1000)
			INTEGRATOR = 1000;
	 
		if(INTEGRATOR < 0)
			INTEGRATOR = 0;
    

    PID = PROPORTIONAL + INTEGRATOR - DERIVATIVE;
    
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PID);
    
		LAST_TEMPERATURE = TEMPERATURE;
		
		#else
		if(TEMPERATURE > SETPOINT_TEMPERAURE + TEMPERATURE_OFFSET)
		{
			liga = 0;
			beep(200);
			LED_OFF;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		}
		
		if(TEMPERATURE < SETPOINT_TEMPERAURE - TEMPERATURE_OFFSET)
		{
			liga = 1;
			LED_ON;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
		}
		#endif
		
		HAL_Delay(156);//necessario para dar 1hz no ciclo
		
		BLYNK_OLED = !BLYNK_OLED;
		SSD1306_GotoXY(12, 0);
		SSD1306_Puts("Aquecendo", &Font_11x18, BLYNK_OLED);
		
		SSD1306_GotoXY(10, 35);
		SSD1306_Puts("Atual", &Font_7x10, 1);
		sprintf(strbuffer, "%.1fC ", TEMPERATURE);
		SSD1306_GotoXY(2, 45);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_GotoXY(80, 35);
		SSD1306_Puts("Alvo", &Font_7x10, 1);
		sprintf(strbuffer, "%02iC", SETPOINT_TEMPERAURE);
		SSD1306_GotoXY(80, 45);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_UpdateScreen();
		
		#ifdef DEBUG
		sprintf(strbuffer, "Temperatura atual %.1fC\n\r", TEMPERATURE);
		CDC_Transmit_FS((uint8_t*)strbuffer, strlen(strbuffer));
		sprintf(strbuffer, "Temperatura Alvo %iC\n\r", SETPOINT_TEMPERAURE);
		CDC_Transmit_FS((uint8_t*)strbuffer, strlen(strbuffer));
		#endif
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(1000);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				liga = 0;
				LED_OFF;
				SSD1306_Clear();
				return;
			}
		}
		int t_end = HAL_GetTick();
		TEMPOOO = t_end - t_in;
	}
}

void cook_function()
{
	int STAGE = 0;
	
	sprintf(strbuffer, "Selecione a");
	SSD1306_GotoXY(2, 0);
	SSD1306_Puts(strbuffer, &Font_11x18, 1);
	
	sprintf(strbuffer, "Temperatura");
	SSD1306_GotoXY(2, 20);
	SSD1306_Puts(strbuffer, &Font_11x18, 1);
	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim4, (4 * 100)-1);
  while(STAGE == 0)
	{
		encoderCount = __HAL_TIM_GET_COUNTER(&htim4)/4;
		sprintf(strbuffer, "%02i C",encoderCount);
		SSD1306_GotoXY(40, 42);
		SSD1306_Puts(strbuffer, &Font_11x18, 0);
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(100);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				STAGE++;
				SETPOINT_TEMPERAURE = encoderCount;
			}
		}
	}
	SSD1306_Clear();
	sprintf(strbuffer, "Selecione o");
	SSD1306_GotoXY(2, 0);
	SSD1306_Puts(strbuffer, &Font_11x18, 1);
	
	sprintf(strbuffer, "Tempo       ");
	SSD1306_GotoXY(2, 20);
	SSD1306_Puts(strbuffer, &Font_11x18, 1);
	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim4, (4 * 100)-1);
  while(STAGE == 1)
	{
		encoderCount = __HAL_TIM_GET_COUNTER(&htim4)/4;
		sprintf(strbuffer, "%02ih",encoderCount);
		SSD1306_GotoXY(2, 40);
		SSD1306_Puts(strbuffer, &Font_11x18, 0);
		
		sprintf(strbuffer, ":00m:00s");
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(100);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				STAGE++;
				SET_HOUR = encoderCount;
			}
		}
	}
	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim4, 4 * 60);
  while(STAGE == 2)
	{
		encoderCount = __HAL_TIM_GET_COUNTER(&htim4)/4;
		sprintf(strbuffer, "%02ih:", SET_HOUR);
		SSD1306_GotoXY(2, 40);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		sprintf(strbuffer, "%02im", encoderCount);
		SSD1306_Puts(strbuffer, &Font_11x18, 0);
		
		sprintf(strbuffer, ":00s");
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(100);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				STAGE++;
				SET_MINUTE = encoderCount;
			}
		}
	}
	
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	__HAL_TIM_SET_AUTORELOAD(&htim4, 4 * 60);
  while(STAGE == 3)
	{
		encoderCount = __HAL_TIM_GET_COUNTER(&htim4)/4;
		sprintf(strbuffer, "%02ih:", SET_HOUR);
		SSD1306_GotoXY(2, 40);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		sprintf(strbuffer, "%02im:", SET_MINUTE);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		sprintf(strbuffer, "%02is",encoderCount);
		SSD1306_Puts(strbuffer, &Font_11x18, 0);
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(100);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				STAGE++;
				SET_SECOND = encoderCount;
			}
		}
	}
	
	SSD1306_Clear();
	HOUR = SET_HOUR; MINUTE = SET_MINUTE; SECOND = SET_SECOND;
	TOTAL_SET_SECOND = (SET_HOUR * 3600) + (SET_MINUTE * 60) + SET_SECOND;
	TOTAL_SET_SECOND_COUNTDOWN = TOTAL_SET_SECOND;
	SECOND_TIMER = 0;
	HAL_TIM_Base_Start_IT(&htim1);
	
	while((STAGE == 4) && (TOTAL_SET_SECOND > SECOND_TIMER))
	{
		int t_in = HAL_GetTick();
	
		#ifdef USE_FILTER
		TEMPERATURE_RAW = Temp_Read();
		TEMPERATURE =  ((FILTER_FACTOR * Ts * TEMPERATURE_RAW) + (LAST_TEMPERATURE)) /( 1 + (FILTER_FACTOR * Ts));
    LAST_TEMPERATURE = TEMPERATURE;
		
		#else
		TEMPERATURE = Temp_Read();
		
		#endif
		
		#ifdef USE_PID
    ERROR_MEASURED = SETPOINT_TEMPERAURE - TEMPERATURE;
    
    PROPORTIONAL = Kp * ERROR_MEASURED;
    INTEGRATOR += (Ki * ERROR_MEASURED) * SAMPLE_TIME;
    DERIVATIVE = ((TEMPERATURE - LAST_TEMPERATURE) * Kd) / SAMPLE_TIME;
   
		if(INTEGRATOR > 1000)
			INTEGRATOR = 1000;
	 
		if(INTEGRATOR < 0)
			INTEGRATOR = 0;
    

    PID = PROPORTIONAL + INTEGRATOR - DERIVATIVE;
    
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PID);
    
		LAST_TEMPERATURE = TEMPERATURE;
		#else
		if(TEMPERATURE > SETPOINT_TEMPERAURE + TEMPERATURE_OFFSET)
		{
			liga = 0;
			LED_OFF;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		}
		
		if(TEMPERATURE < SETPOINT_TEMPERAURE - TEMPERATURE_OFFSET)
		{
			liga = 1;
			LED_ON;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
		}
		#endif
		
		HAL_Delay(153);//necessario para dar 1hz no ciclo
		
		HOUR = TOTAL_SET_SECOND_COUNTDOWN / 3600;
		MINUTE = (TOTAL_SET_SECOND_COUNTDOWN % 3600) / 60;
		SECOND = (TOTAL_SET_SECOND_COUNTDOWN % 3600) % 60;
		
		BLYNK_OLED = !BLYNK_OLED;
		SSD1306_GotoXY(10, 0);
		SSD1306_Puts("Tempo faltante", &Font_7x10, BLYNK_OLED);
		sprintf(strbuffer, "%02ih:%02im:%02is", HOUR, MINUTE, SECOND);
		SSD1306_GotoXY(4, 11);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_GotoXY(10, 35);
		SSD1306_Puts("Atual", &Font_7x10, 1);
		sprintf(strbuffer, "%.1fC ", TEMPERATURE);
		SSD1306_GotoXY(2, 45);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_GotoXY(80, 35);
		SSD1306_Puts("Alvo", &Font_7x10, 1);
		sprintf(strbuffer, "%02iC", SETPOINT_TEMPERAURE);
		SSD1306_GotoXY(80, 45);
		SSD1306_Puts(strbuffer, &Font_11x18, 1);
		
		SSD1306_UpdateScreen();
		
		#ifdef DEBUG
		sprintf(strbuffer, "\nTempo faltante %02ih:%02im:%02is\n", HOUR, MINUTE, SECOND);
		CDC_Transmit_FS((uint8_t*)strbuffer, strlen(strbuffer));
		sprintf(strbuffer, "Temperatura atual %.1fC\n", TEMPERATURE);
		CDC_Transmit_FS((uint8_t*)strbuffer, strlen(strbuffer));
		sprintf(strbuffer, "Temperatura alvo %iC\n", SETPOINT_TEMPERAURE);
		CDC_Transmit_FS((uint8_t*)strbuffer, strlen(strbuffer));
		#endif
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(3000);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				SSD1306_Clear();
				HAL_TIM_Base_Stop_IT(&htim1);
				__HAL_TIM_SET_COUNTER(&htim1, 0);
				liga = 0;
				LED_OFF;
				for(int k = 0; k < 3; k++)
				{
					BEEP_ON;
					LED_ON;
					HAL_Delay(200);
					BEEP_OFF;
					LED_OFF;
					HAL_Delay(100);
				}
				return;
			}
		}
		int t_end = HAL_GetTick();
		TEMPOOO = t_end - t_in;
	}	
	HAL_TIM_Base_Stop_IT(&htim1);
	liga = 0;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	HAL_Delay(1000);
	SSD1306_Clear();
	STAGE = 5;
	
	for(int k = 0; k < 3; k++)
	{
		BEEP_ON;
		LED_ON;
		HAL_Delay(200);
		
		BEEP_OFF;
		LED_OFF;
		HAL_Delay(100);
	}
	
	while(STAGE == 5)
	{		
		SSD1306_GotoXY(2, 0);
		SSD1306_Puts("Cozimento", &Font_11x18, 1);
		SSD1306_GotoXY(2, 35);
		SSD1306_Puts("Finalizado", &Font_11x18, 1);
		SSD1306_UpdateScreen();
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 1) && (SW_ALREADY_PRESSED == 1))
			SW_ALREADY_PRESSED = 0;
		
		if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
		{
			HAL_Delay(100);
			if((HAL_GPIO_ReadPin(SW_ENC_GPIO_Port, SW_ENC_Pin) == 0) && (SW_ALREADY_PRESSED == 0))
			{
				SW_ALREADY_PRESSED = 1;
				beep(200);
				SSD1306_Clear();
				return;
			}
		}
	}
}

void beep(int delay)
{
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
	HAL_Delay(delay/2);
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
	HAL_Delay(delay/2);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim1)
{
	SECOND_TIMER++;
	TOTAL_SET_SECOND_COUNTDOWN--;
}

float Temp_Read(void)
{	
	float Temperature = 0;
	int16_t Presence = 0;
	int16_t Temp_byte1, Temp_byte2;
	int16_t TEMP;
	
	Presence = DS18B20_Start();
	HAL_Delay(1);
	DS18B20_Write (0xCC);  // skip ROM
	DS18B20_Write (0x44);  // convert t
	HAL_Delay(800); 

	Presence = DS18B20_Start();
  HAL_Delay(1);
  DS18B20_Write (0xCC);  // skip ROM
  DS18B20_Write (0xBE);  // Read Scratch-pad

  Temp_byte1 = DS18B20_Read();
	Temp_byte2 = DS18B20_Read();
	TEMP = (Temp_byte2<<8)|Temp_byte1;
	Temperature = (float)TEMP/16.0;

	return Temperature;
	/*
	float adcVal;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	adcVal = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adcVal;
	*/
}

void delay_us (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim2) < us);  // wait for the counter to reach the us input in the parameter
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_Start (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set the pin as output
	HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);  // pull the pin low
	delay_us(480);   // delay according to datasheet

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);    // set the pin as input
	delay_us(80);    // delay according to datasheet

	if (!(HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))) Response = 1;    // if the pin is low i.e the presence pulse is detected
	else Response = -1;

	delay_us(400); // 480 us delay totally.

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);  // set as output
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);  // pull the pin LOW
			delay_us(1);  // wait for 1 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
			delay_us(50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);
			HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);  // pull the pin LOW
			delay_us(50);  // wait for 60 us

			Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);

	for (int i=0;i<8;i++)
	{
		Set_Pin_Output(DS18B20_PORT, DS18B20_PIN);   // set as output

		HAL_GPIO_WritePin (DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);  // pull the data pin LOW
		delay_us(1);  // wait for > 1us

		Set_Pin_Input(DS18B20_PORT, DS18B20_PIN);  // set as input
		if (HAL_GPIO_ReadPin (DS18B20_PORT, DS18B20_PIN))  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
		delay_us(50);  // wait for 60 us
	}
	return value;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
