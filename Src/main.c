
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "processing.h"
#include "r_encoder.h"
#include "lcd_manager.h"
#include "buttonconfig.h"
#include "../STM32-LCD-HD44780-I2C/lcd_hd44780_i2c.h"
#include "cui.h"
#include "Tone.h"
#include "I2CFlash.h"
#include "MIDIParser.h"
#include "Sequencer.h"
#include "MidiConfig.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart1;

osThreadId defaultTaskHandle;
osThreadId cdcTaskHandle;
osThreadId accessStorageTaHandle;
osMessageQId queue01Handle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//Editing Parameters
#define READ_SYSTEMCONFIG_FROM_FLASH 1
#define CURR_GEN &synth[SelectedChannel]

static const uint8_t MIDI_SYSRT_MESSAGE_START = 0xFA;
static const uint8_t MIDI_SYSRT_MESSAGE_STOP = 0xFC;
static const uint8_t MIDI_SYSRT_MESSAGE_TIMING_CLOCK = 0xF8;

int SelectedChannel = 0;
int selected_row = 0;
int selected_col = 1;
static int pNo = 0;
static volatile uint8_t tartgetProgramNo = 0;
Sequencer sequencer;
static I2C_EEPROM eeprom;
static MidiConfig midiConfig;
volatile float cv1 = 0, cv2 = 0, cv3 = 0, cv4 = 0;
volatile uint8_t tmp_byte = 0;
volatile uint8_t tmp_tx_byte = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C3_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM13_Init(void);
void StartDefaultTask(void const * argument);
void StartCdcTask(void const * argument);
void StartAccessStorageTask(void const * argument);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void lightsOff(void);
static void updateLed(void);
void revert(void);
void write_LCD_PARAM(const char *name, int value);
void write_LCD_PARAM_NAME(const char *param_name, const char* value);
static void refreshLCD(void);
static void updateLCD(void);
static void updateSelectProgram(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void resetPitchShift() {
	Gen_set_pitchShift(&synth[0], 1.0f);
	Gen_set_pitchShift(&synth[1], 1.0f);
	Gen_set_pitchShift(&synth[2], 1.0f);
	Gen_set_pitchShift(&synth[3], 1.0f);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_I2S3_Init();
  MX_TIM6_Init();
  MX_I2C3_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_TIM7_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
	FastMath_Init();
	InitRotaryEncoders();
	ButtonConfigInit();
	InitSynthesizer();
	InitSequencer(&sequencer);
	InitFactorySetTones();
	I2CFlash_Init(&eeprom, &hi2c1);
	InitMidiConfig(&midiConfig);

	Tone tone;
	InitTone(&tone);
	ToneCopyToGen(&synth[0], &tone);
	ToneCopyToGen(&synth[1], &tone);
	ToneCopyToGen(&synth[2], &tone);
	ToneCopyToGen(&synth[3], &tone);

	InitLcdManager();

	waitUntilReady(&eeprom);
	/*Read System Settngs*/
	if (READ_SYSTEMCONFIG_FROM_FLASH) {
		I2CFlash_Read(&eeprom, ROM_ADDRESS_MIDICONFIG, (uint8_t*)&midiConfig,
				sizeof(MidiConfig));
		waitUntilReady(&eeprom);
	}

	/*Read Temporary Data*/
	for (int i = 0; i < 4; i++) {
		Tone t;
		size_t tone_size = sizeof(Tone);
		I2CFlash_Read(&eeprom, ROM_ADDRESS_TONE_TMP + (i * 64),
				(uint8_t*) &t, tone_size);
		waitUntilReady(&eeprom);
		ToneCopyToGen(&synth[i], &t);
	}
	I2CFlash_LoadSequenceData(&eeprom, &sequencer);

	/*Start Processing*/
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adc_buff, ADC_SAMPLE_LENGTH);
	HAL_TIM_Base_Start_IT(&htim2);

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_I2S_Transmit_DMA(&hi2s3, circularbuffer, AUDIO_BLOCK_SIZE * 2 * 2);

	HAL_UART_Receive_IT(&huart1, (uint8_t*) &tmp_byte, 1); //Start MIDI Driver

	HAL_TIM_Base_Start_IT(&htim11);

	HAL_TIM_Base_Start_IT(&htim13);

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of cdcTask */
  osThreadDef(cdcTask, StartCdcTask, osPriorityLow, 0, 128);
  cdcTaskHandle = osThreadCreate(osThread(cdcTask), NULL);

  /* definition and creation of accessStorageTa */
  osThreadDef(accessStorageTa, StartAccessStorageTask, osPriorityLow, 0, 128);
  accessStorageTaHandle = osThreadCreate(osThread(accessStorageTa), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of queue01 */
/* what about the sizeof here??? cd native code */
  osMessageQDef(queue01, 16, uint16_t);
  queue01Handle = osMessageCreate(osMessageQ(queue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	while (1) {
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S;
  PeriphClkInitStruct.PLLI2S.PLLI2SN = 172;
  PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
  /* EXTI2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  /* EXTI3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);
  /* EXTI4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  /* EXTI9_5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

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
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C2 init function */
static void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2S3 init function */
static void MX_I2S3_Init(void)
{

  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_DISABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1679;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 16800-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 168-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10-1;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM11 init function */
static void MX_TIM11_Init(void)
{

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 1680-1;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 29000-1;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM13 init function */
static void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 840-1;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 2083-1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 31250;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, BLED1_Pin|BLED2_Pin|BLED3_Pin|BLED4_Pin 
                          |BLED5_Pin|BLED6_Pin|BLED7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TRG1_Pin TRG2_Pin TRG3_Pin TRG4_Pin */
  GPIO_InitStruct.Pin = TRG1_Pin|TRG2_Pin|TRG3_Pin|TRG4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : GPIO_EXTSW1_Pin */
  GPIO_InitStruct.Pin = GPIO_EXTSW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIO_EXTSW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_EXTSW2_Pin BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = GPIO_EXTSW2_Pin|BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RoEncA1_Pin RoEncA2_Pin RoEncB1_Pin RoEncB2_Pin */
  GPIO_InitStruct.Pin = RoEncA1_Pin|RoEncA2_Pin|RoEncB1_Pin|RoEncB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RoEncE1_Pin RoEncE2_Pin */
  GPIO_InitStruct.Pin = RoEncE1_Pin|RoEncE2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_ADCSW1_Pin GPIO_IN_PAD1_Pin GPIO_IN_PAD2_Pin GPIO_IN_PAD3_Pin 
                           GPIO_IN_PAD4_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_ADCSW1_Pin|GPIO_IN_PAD1_Pin|GPIO_IN_PAD2_Pin|GPIO_IN_PAD3_Pin 
                          |GPIO_IN_PAD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RoEncC1_Pin RoEncC2_Pin RoEncD1_Pin RoEncD2_Pin 
                           RoEncS1_Pin RoEncS2_Pin */
  GPIO_InitStruct.Pin = RoEncC1_Pin|RoEncC2_Pin|RoEncD1_Pin|RoEncD2_Pin 
                          |RoEncS1_Pin|RoEncS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : GPIO_INPUT_ADCSW2_Pin GPIO_INPUT_ADCSW3_Pin GPIO_INPUT_ADCSW4_Pin */
  GPIO_InitStruct.Pin = GPIO_INPUT_ADCSW2_Pin|GPIO_INPUT_ADCSW3_Pin|GPIO_INPUT_ADCSW4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN9_Pin BTN4_Pin BTN5_Pin BTN6_Pin 
                           BTN7_Pin BTN8_Pin */
  GPIO_InitStruct.Pin = BTN9_Pin|BTN4_Pin|BTN5_Pin|BTN6_Pin 
                          |BTN7_Pin|BTN8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : BLED1_Pin BLED2_Pin BLED3_Pin BLED4_Pin 
                           BLED5_Pin BLED6_Pin BLED7_Pin */
  GPIO_InitStruct.Pin = BLED1_Pin|BLED2_Pin|BLED3_Pin|BLED4_Pin 
                          |BLED5_Pin|BLED6_Pin|BLED7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN3_Pin */
  GPIO_InitStruct.Pin = BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
static void refresh1stLine(void) {
	static char cbuff[17] = { '\0' };
	sprintf(cbuff, "%c:%-14s", 'A' + (char) SelectedChannel,
			LCDM_EDIT_GROUP_TEXT[selected_row]);
	lcdWriteText(0, (char*) cbuff, 16);
}


static void lightsOff(void){
	HAL_GPIO_WritePin(BLED1_GPIO_Port,
	BLED1_Pin |
	BLED2_Pin |
	BLED3_Pin |
	BLED4_Pin |
	BLED5_Pin |
	BLED6_Pin |
	BLED7_Pin, GPIO_PIN_RESET);
}

static void updateLed() {
	lightsOff();
	switch (selected_row) {
	case 6:
		HAL_GPIO_WritePin(BLED1_GPIO_Port, BLED1_Pin, GPIO_PIN_SET);
		break;
	case 5:
		HAL_GPIO_WritePin(BLED2_GPIO_Port, BLED2_Pin, GPIO_PIN_SET);
		break;
	case 4:
		HAL_GPIO_WritePin(BLED3_GPIO_Port, BLED3_Pin, GPIO_PIN_SET);
		break;
	case 3:
		HAL_GPIO_WritePin(BLED4_GPIO_Port, BLED4_Pin, GPIO_PIN_SET);
		break;
	case 2:
		HAL_GPIO_WritePin(BLED5_GPIO_Port, BLED5_Pin, GPIO_PIN_SET);
		break;
	case 1:
		HAL_GPIO_WritePin(BLED6_GPIO_Port, BLED6_Pin, GPIO_PIN_SET);
		break;
	case 0:
		HAL_GPIO_WritePin(BLED7_GPIO_Port, BLED7_Pin, GPIO_PIN_SET);
		break;
	}
}

void onChangeRE_S(int id, int add) {

	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM
			|| LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		return;
	}

	if (is_pressed_key_SHIFT) {
		return;
	}

	if (LcdMenuState != LCD_STATE_DEFAULT) {
		return;
	}

	selected_row = LIMIT(selected_row + (add > 0 ? 1 : -1), 6, 0);
	updateLed();
	refreshLCD();

}

void onChangeRE_A(int id, int add) {

	if (LcdMenuState == LCD_STATE_SEQ_STEP_CFG) {
		showSequencerStepConfig(&sequencer, 0, add);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		showSequencerBeatRepeatConfig(&sequencer, 0, add);
		return;
	}

	if (LcdMenuState != LCD_STATE_DEFAULT) {
		return;
	}

	selected_col = ROTARY_ENCODER_A;
	switch (selected_row) {
	case PARAM_GROUP_OSC:
		//WAVE
	{
		Waveform wf = Gen_get_carr_wave(CURR_GEN);
		wf = LIMIT(wf + add, 2, 0);
		Gen_set_carr_wave(CURR_GEN, wf);
	}
		break;
	case PARAM_GROUP_AMP: {
		if (is_pressed_key_SHIFT) {
			//PAN
			Gen_set_pan(CURR_GEN, synth[SelectedChannel].i_pan + add);
			break;
		} else {
			//LEVEL
			int level = Gen_get_carr_level(CURR_GEN);
			level = LIMIT(level + add, 127, 0);
			Gen_set_carr_level(CURR_GEN, level);
			break;
		}
	}

	case PARAM_GROUP_MODU: {
		//FM FREQ
		int n = Gen_get_fm_harmonics(CURR_GEN);
		n = LIMIT(n + add, 63, -63);
		Gen_set_fm_harmonics(CURR_GEN, n);
	}
		break;
	case PARAM_GROUP_BEND: {
		//AMOUNT
		if (is_pressed_key_SHIFT) {
			int amount = Gen_get_bend_velocity_sense(CURR_GEN);
			amount = LIMIT(amount + add, 127, 0);
			Gen_set_bend_velocity_sense(CURR_GEN, amount);
		} else {
			int amount = Gen_get_bend_amount(CURR_GEN);
			amount = LIMIT(amount + add, 127, 0);
			Gen_set_bend_amount(CURR_GEN, amount);
		}
	}
		break;
	case PARAM_GROUP_NOISE: {
		//NOISE LEVEL
		int level = Gen_get_noise_level(CURR_GEN) + add;
		level = LIMIT(level, 127, 0);
		Gen_set_noise_level(CURR_GEN, level);
	}
		break;
	case PARAM_GROUP_FILTER:
		//FILTER TYPE
	{
		int index = Gen_get_filter_type(CURR_GEN) + (add > 0 ? 1 : -1);
		index = LIMIT(index, 3, 0);
		Gen_set_filter_type(CURR_GEN, index);
	}
		break;
	case PARAM_GROUP_LFO: {
		int d = (int) (LFO_getDest( &(synth[SelectedChannel].lfo) ) + (add > 0 ? 1 : -1));
		d = LIMIT(d, 3, 0);
		Gen_set_lfo_dest(CURR_GEN, d);
	}
	break;
}
	updateLCD();
}

void onChangeRE_B(int id, int add) {

	if (LcdMenuState == LCD_STATE_SEQ_STEP_CFG) {
		showSequencerStepConfig(&sequencer, 1, add);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		showSequencerBeatRepeatConfig(&sequencer, 1, add);
		return;
	}

	if (LcdMenuState != LCD_STATE_DEFAULT) {
		return;
	}

	selected_col = ROTARY_ENCODER_B;
	switch (selected_row) {
	case PARAM_GROUP_OSC: {
		if (is_pressed_key_SHIFT) {
			//fine
			int value = Gen_get_carr_fine(CURR_GEN);
			value = LIMIT(value + add, 63, -63);
			Gen_set_carr_fine(CURR_GEN, value);
		} else {
			//PITCH
			int value = Gen_get_carr_coarse(CURR_GEN);
			value = LIMIT(value + add, 127, 0);
			Gen_set_carr_coarse(CURR_GEN, value);
		}
	}
		break;
	case PARAM_GROUP_AMP: {
		//ATTACK
		int value = Gen_get_carr_attack(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_carr_attack(CURR_GEN, value);
	}
		break;

	case PARAM_GROUP_MODU: {
		//ATTACK
		int value = Gen_get_fm_attack(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_fm_attack(CURR_GEN, value);
		break;
	}

	case PARAM_GROUP_BEND: {
		//ATTACK
		int value = Gen_get_bend_attack(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_bend_attack(CURR_GEN, value);
	}
		break;

	case PARAM_GROUP_NOISE: {
		//NOISE ATTACK
		int value = LIMIT(Gen_get_noise_attack(CURR_GEN) + add, 127, 0);
		Gen_set_noise_attack(CURR_GEN, value);
	}
		break;
	case PARAM_GROUP_FILTER: {
		//FILTER CUTOFF
		int value = LIMIT(Gen_get_filter_cutoff(CURR_GEN) + add, 127, 0);
		Gen_set_filter_cutoff(CURR_GEN, value);
		break;
	}
	case PARAM_GROUP_LFO: {
		//TODO:	implement it!
		synth[SelectedChannel].lfo.waveform =
				(synth[SelectedChannel].lfo.waveform + (add > 0 ? 1 : -1))
						& 0x1;
		LFO_setWave(CURR_GEN.lfo, synth[SelectedChannel].lfo.waveform);
		break;
	}
	}
	updateLCD();
}

void onChangeRE_C(int id, int add) {

	if (LcdMenuState == LCD_STATE_MIDI_RECEIVE_CONFIG) {
		MIDIConfig_ChangeCh(&midiConfig, add >= 0 ? 1 : -1);
		resetPitchShift();
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		showSequencerBeatRepeatConfig(&sequencer, 2, add);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_STEP_CFG) {
		showSequencerStepConfig(&sequencer, 2, add);
		return;
	}

	if (LcdMenuState != LCD_STATE_DEFAULT) {
		return;
	}

	selected_col = ROTARY_ENCODER_C;
	switch (selected_row) {
	case PARAM_GROUP_OSC: {
		//M.Depth
		int value = Gen_get_carr_moddepth(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_carr_moddepth(CURR_GEN, value);
	}
		break;
	case PARAM_GROUP_AMP: {
		if (is_pressed_key_SHIFT) {
			//SLOPE
			int value = Gen_get_carr_slope(CURR_GEN);
			value = LIMIT(value + add, 127, 0);
			Gen_set_carr_slope(CURR_GEN, value);
		} else {
			//HOLD
			int value = Gen_get_carr_hold(CURR_GEN);
			value = LIMIT(value + add, 127, 0);
			Gen_set_carr_hold(CURR_GEN, value);
		}
	}
		break;
	case PARAM_GROUP_MODU: {
		if (is_pressed_key_SHIFT) {
			//SLOPE
			int value = Gen_get_fm_slope(CURR_GEN);
			value = LIMIT(value + add, 127, 0);
			Gen_set_fm_slope(CURR_GEN, value);
		} else {
			//HOLD
			int value = Gen_get_fm_hold(CURR_GEN);
			value = LIMIT(value + add, 127, 0);
			Gen_set_fm_hold(CURR_GEN, value);
		}
	}
		break;
	case PARAM_GROUP_BEND: {
		if (is_pressed_key_SHIFT) {
			//SLOPE
			int value = LIMIT(Gen_get_bend_slope(CURR_GEN) + add, 127, 0);
			Gen_set_bend_slope(CURR_GEN, value);

		} else {
			//HOLD
			int value = Gen_get_bend_hold(CURR_GEN);
			value = LIMIT(value + add, 127, 0);
			Gen_set_bend_hold(CURR_GEN, value);
		}
	}
		break;
	case PARAM_GROUP_NOISE: {
		if (is_pressed_key_SHIFT) {
			//SLOPE
			int value = LIMIT(Gen_get_noise_slope(CURR_GEN) + add, 127, 0);
			Gen_set_noise_slope(CURR_GEN, value);
		} else {
			//NOISE HOLD
			int value = LIMIT(Gen_get_noise_hold(CURR_GEN) + add, 127, 0);
			Gen_set_noise_hold(CURR_GEN, value);
		}
	}
		break;
	case PARAM_GROUP_FILTER: {
		int value = (synth[SelectedChannel]).filter.resonance +=
				add >= 0 ? 1 : -1;
		Gen_set_filter_resonance(CURR_GEN, value);
		break;
	}
		break;
	case PARAM_GROUP_LFO: {
		uint8_t value = LIMIT(LFO_getSpeed(CURR_GEN.lfo) + add, 127, 0);
		LFO_setSpeed(CURR_GEN.lfo, value);
		break;
	}
		break;
	}
	updateLCD();
}

void onChangeRE_D(int id, int add) {

	if (LcdMenuState == LCD_STATE_MIDI_RECEIVE_CONFIG) {
		MIDIConfig_ChangeNt(&midiConfig, add);
		resetPitchShift();
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_STEP_CFG) {
		showSequencerStepConfig(&sequencer, 3, add);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		showSequencerBeatRepeatConfig(&sequencer, 3, add);
		return;
	}

	if (LcdMenuState != LCD_STATE_DEFAULT) {
		return;
	}

	selected_col = ROTARY_ENCODER_D;
	switch (selected_row) {
	case PARAM_GROUP_OSC: {
		int value = (synth[SelectedChannel].modtype) + (add > 0 ? 1 : -1);
		synth[SelectedChannel].modtype = LIMIT(value, (NUMOF_MODTYPE-1), 0);
	}
		break;
	case PARAM_GROUP_AMP: {
		//RELEASE
		int value = Gen_get_carr_release(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_carr_release(CURR_GEN, value);
	}
		break;
	case PARAM_GROUP_MODU: {
		//Release
		int value = Gen_get_fm_release(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_fm_release(CURR_GEN, value);
	}
		break;
	case PARAM_GROUP_BEND: {
		//HOLD
		int value = Gen_get_bend_release(CURR_GEN);
		value = LIMIT(value + add, 127, 0);
		Gen_set_bend_release(CURR_GEN, value);
	}
		break;
	case PARAM_GROUP_NOISE: {
		//NOISE RELEASE
		int value = LIMIT(Gen_get_noise_release(CURR_GEN) + add, 127, 0);
		Gen_set_noise_release(CURR_GEN, value);
	}
		break;
	case PARAM_GROUP_FILTER: {
		if (is_pressed_key_SHIFT) {
			//FILTER DECAY
			Gen_set_filter_decay( CURR_GEN,
					synth[SelectedChannel].decay_filter.i_decay + add);
		} else {
			//ACCENT
			int value = (synth[SelectedChannel]).decay_filter.i_amount + add;
			Gen_set_filter_amount(CURR_GEN, value);
		}
	}
		break;
	case PARAM_GROUP_LFO: {
		int v = LFO_getDepth(CURR_GEN.lfo) + add;
		v = LIMIT(v, 127, 0);
		LFO_setDepth(CURR_GEN.lfo, (uint8_t) v);
	}
	break;
}
	updateLCD();
}

void onChangeRE_E(int id, int add) {

	if (LcdMenuState == LCD_STATE_MENU) {
		SelectMenu(add);
		return;
	}

	if (LcdMenuState == LCD_STATE_MENU) {
		return;
	}

	if (LcdMenuState == LCD_STATE_VELC) {
		MIDIConfig_VelocityCurve(&midiConfig, add >= 0 ? 1 : -1);
		return;
	}

	if(LcdMenuState == LCD_STATE_SEQ_EDIT){
		if (is_pressed_key_SHIFT) {
			ChangeBPM(&sequencer, add);
			ShowSequencerEditMode(&sequencer, 0, midiConfig.syncMode);
		} else {
			ShowSequencerEditMode(&sequencer, add, midiConfig.syncMode);
		}
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		showSequencerBeatRepeatConfig(&sequencer, 4, add);
		return;
	}

	if (LcdMenuState == LCD_STATE_SYNC) {
		if (midiConfig.syncMode == InternalClock) {
			midiConfig.syncMode = ExternalClock;
		} else {
			midiConfig.syncMode = InternalClock;
		}
		MIDIConfig_SyncMode(&midiConfig);
		return;
	}

	if (LcdMenuState == LCD_STATE_ECHOBACK) {
		if (add) {
			midiConfig.echoBack = add > 0 ? 1 : 0;
		}
		MIDIConfig_EchoBack(&midiConfig);
		return;
	}

	if (LcdMenuState == LCD_STATE_MIDI_RECEIVE_CONFIG) {
		MIDIConfig_DisplayChannel(&midiConfig, add >= 0 ? 1 : -1);
		return;
	}

	if (LcdMenuState == LCD_STATE_PROGRAM_MENU) {
		ShowProgramMenu(add >= 0 ? 1 : -1);
		return;
	}

	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM) {
		tartgetProgramNo += add;
		tartgetProgramNo &= 0x7F;
		updateSelectProgram();
		return;
	}

	if (LcdMenuState == LCD_STATE_SAVE_PROGRAM) {

		tartgetProgramNo += add;
		tartgetProgramNo &= 0x7F;
		updateSelectProgram();

		return;
	}

	if (LcdMenuState == LCD_STATE_DEFAULT) {
		if (is_pressed_key_SHIFT) {
			int newc = selected_col + (add > 0 ? 1 : -1);
			if (newc < COL_LEFT_EDGE) {
				newc = COL_RIGHT_EDGE;
			} else if (newc > COL_RIGHT_EDGE) {
				newc = COL_LEFT_EDGE;
			}
			selected_col = newc;
			refreshLCD();
		} else {
			SelectedChannel = LIMIT(SelectedChannel + add, (NUM_OF_VOICES-1),
					0);
			refreshLCD();
		}
	}

}

void ON_PUSH_MENU(void) {
	if (LcdMenuState == LCD_STATE_MENU) {
		SelectMenu(1);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_EDIT) {
		showSequencerStepConfig(&sequencer, -1, 0);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_STEP_CFG) {
		showSequencerBeatRepeatConfig(&sequencer,-1,0);
		return;
	}

	if (LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		ShowSequencerEditMode(&sequencer, 0, midiConfig.syncMode);
		return;
	}

	if (LcdMenuState == LCD_STATE_MIDI_RECEIVE_CONFIG) {
		MIDIConfig_DisplayChannel(&midiConfig,1);
		return;
	}

	if (LcdMenuState == LCD_STATE_SYNC) {
		//NOP
		return;
	}

	if (LcdMenuState == LCD_STATE_VELC) {
		//NOP
		return;
	}

	if (LcdMenuState == LCD_STATE_DEFAULT || LcdMenuState == LCD_STATE_PROGRAM_MENU) {
		lightsOff();
		SelectMenu(0);
		LcdMenuState = LCD_STATE_MENU;
		return;
	}
}

void ON_PUSH_EXIT(void) {

	if (LcdMenuState == LCD_STATE_MENU || LcdMenuState == LCD_STATE_PROGRAM_MENU) {
		LcdMenuState = LCD_STATE_DEFAULT;
		updateLed();
		refreshLCD();
		return;
	}

	if (LcdMenuState == LCD_STATE_SAVE_PROGRAM || LcdMenuState == LCD_STATE_LOAD_PROGRAM || LcdMenuState == LCD_STATE_CONFIRM_REVERT){
		ShowProgramMenu(0);
		return;
	}

	if (LcdMenuState == LCD_STATE_MIDI_RECEIVE_CONFIG
			|| LcdMenuState == LCD_STATE_VELC
			|| LcdMenuState == LCD_STATE_ECHOBACK
			|| LcdMenuState == LCD_STATE_SYNC) {
		LcdMenuState = LCD_STATE_MENU;
		HAL_StatusTypeDef ret = I2CFlash_SaveMidiConfig(&eeprom, &midiConfig);
		if (ret != HAL_OK) {
			lcdWriteText(0, "Err ", 16);
			return;
		}
		SelectMenu(0);
		return;
	}

	if (LcdMenuState == LCD_STATE_MONITOR_CV
			|| LcdMenuState == LCD_STATE_SYSTEM_INFO
			|| LcdMenuState == LCD_STATE_SEQ_EDIT
			|| LcdMenuState == LCD_STATE_SEQ_STEP_CFG
			|| LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		LcdMenuState = LCD_STATE_MENU;
		SelectMenu(0);
		return;
	}

	if (LcdMenuState == LCD_STATE_FACTORY_RESET_CONFIRM) {
		LcdMenuState = LCD_STATE_MENU;
		SelectMenu(0);
		return;
	}

	if (LcdMenuState == LCD_STATE_DEFAULT) {

		//change column
		int newc = selected_col + 1;
		if (newc < COL_LEFT_EDGE) {
			newc = COL_RIGHT_EDGE;
		} else if (newc > COL_RIGHT_EDGE) {
			newc = COL_LEFT_EDGE;
		}
		selected_col = newc;
		refreshLCD();

		return;
	}

}


void ON_PUSH_ENTER(void) {

	/*
	if (LcdMenuState == LCD_STATE_DEFAULT && is_pressed_key_SHIFT) {
		uint16_t mess = (1 << 8) | (uint8_t) SelectedChannel;
		osMessagePut(queue01Handle, mess, 0);
		return;
	}
	*/

	if (LcdMenuState == LCD_STATE_MENU) {
		switch (LcdMenuSelectedItemIndex) {
		case ITEM_INDEX_SEQUENCER:
			LcdMenuState = LCD_STATE_SEQ_EDIT;
			ShowSequencerEditMode(&sequencer,0, midiConfig.syncMode);
			return;
		case ITEM_INDEX_MIDI_MAPPING:
			LcdMenuState = LCD_STATE_MIDI_RECEIVE_CONFIG;
			MIDIConfig_Show(&midiConfig);
			break;
		case ITEM_INDEX_SYNC:
			LcdMenuState = LCD_STATE_SYNC;
			MIDIConfig_SyncMode(&midiConfig);
			break;
		case ITEM_INDEX_VELOCITY_CURVE:
			LcdMenuState = LCD_STATE_VELC;
			MIDIConfig_VelocityCurve(&midiConfig, 0);
			break;
		case ITEM_INDEX_MONITOR_CV:
			LcdMenuState = LCD_STATE_MONITOR_CV;
			CV_Monitor_Show();
			break;
		case ITEM_INDEX_ECHO_BACK:
			LcdMenuState = LCD_STATE_ECHOBACK;
			MIDIConfig_EchoBack(&midiConfig);
			break;
		case ITEM_INDEX_SYSTEM_INFO:
			showSystemVersion();
			break;
		case ITEM_INDEX_FACTORY_RESET:
			LcdMenuState = LCD_STATE_FACTORY_RESET_CONFIRM;
			ConfirmFactoryReset();
			return;
		}
	}

	if (LcdMenuState == LCD_STATE_SEQ_EDIT
			|| LcdMenuState == LCD_STATE_SEQ_STEP_CFG
			|| LcdMenuState == LCD_STATE_SEQ_BEAT_REPEAT) {
		if (sequencer.status == SEQ_IDLING) {
			StartSequencer(&sequencer);
		} else {
			StopSequencer(&sequencer);
		}
	}

	if (LcdMenuState == LCD_STATE_PROGRAM_MENU) {
		switch (ProgramMenuSelectedItemIndex) {
		case ITEM_INDEX_TEMPSAVE:
		{
			Tone t1,t2,t3,t4;
			ToneCopyFromGen(&t1,&synth[0]);
			ToneCopyFromGen(&t2,&synth[1]);
			ToneCopyFromGen(&t3,&synth[2]);
			ToneCopyFromGen(&t4,&synth[3]);

			HAL_StatusTypeDef ret = I2CFlash_Write(&eeprom,
			ROM_ADDRESS_TONE_TMP, (uint8_t*) &t1, sizeof(Tone));
			uint16_t tone_size = sizeof(Tone);
			if (ret != HAL_OK) {
				lcdWriteText(0, "Error ", 16);
				return;
			}
			ret = I2CFlash_Write(&eeprom, ROM_ADDRESS_TONE_TMP + 64,
					(uint8_t*) &t2, tone_size);
			if (ret != HAL_OK) {
				lcdWriteText(0, "Error ", 16);
				return;
			}
			ret = I2CFlash_Write(&eeprom, ROM_ADDRESS_TONE_TMP + 128,
					(uint8_t*) &t3, tone_size);
			if (ret != HAL_OK) {
				lcdWriteText(0, "Error ", 16);
				return;
			}
			ret = I2CFlash_Write(&eeprom, ROM_ADDRESS_TONE_TMP + 192,
					(uint8_t*) &t4, tone_size);
			if (ret != HAL_OK) {
				lcdWriteText(0, "Error ", 16);
				return;
			}

			ret = I2CFlash_SaveSequenceData(&eeprom, &sequencer);
			if (ret != HAL_OK) {
				lcdWriteText(0, "Error ", 16);
			}

			LcdMenuState = LCD_STATE_DEFAULT;
			refreshLCD();
			updateLed();
		}
			return;
		case ITEM_INDEX_STORE:
			LcdMenuState = LCD_STATE_SAVE_PROGRAM;
			updateSelectProgram();
			return;
		case ITEM_INDEX_LOAD:
			LcdMenuState = LCD_STATE_LOAD_PROGRAM;
			updateSelectProgram();
			return;
		case ITEM_INDEX_REVERT:
			LcdMenuState = LCD_STATE_CONFIRM_REVERT;
			showConfirmRevert();
			return;
		}
	}

	if(LcdMenuState == LCD_STATE_CONFIRM_REVERT){
		revert();
		LcdMenuState = LCD_STATE_DEFAULT;
		updateLed();
		refreshLCD();
		return;
	}

	if (LcdMenuState == LCD_STATE_FACTORY_RESET_CONFIRM) {
		lcdWriteText(0, "Initialize......", 16);
		lcdWriteText(1, "................", 16);
		HAL_StatusTypeDef ret = I2CFlash_FactoryReset(&eeprom);
		if(ret == HAL_OK){
			//reboot
			NVIC_SystemReset();
		}else{
			//ERROR
			lcdWriteText(0, "-Error-         ", 16);
			char str[17] = {' '};
			sprintf(str,"CODE=%d           ",ret);
			lcdWriteText(1, str, 16);
		}
		return;
	}

	if (LcdMenuState == LCD_STATE_MONITOR_CV) {
		//NOP
		return;
	}

	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM) {
		Tone t;
		HAL_StatusTypeDef result = ReadTone(&eeprom, tartgetProgramNo, &t);
		if (result == HAL_OK) {
			ToneCopyToGen(CURR_GEN, &t);
			pNo = tartgetProgramNo;
			//refreshLCD();
		} else {
			lcdWriteText(0, "ERROR!", 16);
		}
	}

	if (LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		Tone t;
		InitTone(&t);
		ToneCopyFromGen(&t, &(synth[SelectedChannel]));
		HAL_StatusTypeDef result = SaveTone(&eeprom, tartgetProgramNo, &t);
		switch (result) {
		case HAL_OK:
			pNo = tartgetProgramNo;
			LcdMenuState = LCD_STATE_DEFAULT;
			refreshLCD();
			updateLed();
			break;

		case HAL_ERROR:
			lcdWriteText(0, "ERROR!", 16);
			break;

		case HAL_TIMEOUT:
			lcdWriteText(0, "TIMEOUT!", 16);
			break;

		case HAL_BUSY:
			lcdWriteText(0, "BUSY!", 16);
			break;

		}
	}
}

static void updateSelectProgram() {
	static char buff[17];
	if (LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		lcdWriteText(0, "Store Program   ", 16);
		sprintf(buff, "%c ~ %03d         ", (char) ('A' + SelectedChannel),
				tartgetProgramNo);

		lcdWriteText(1, &buff[0], 16);
	} else if(LcdMenuState == LCD_STATE_LOAD_PROGRAM){
		lcdWriteText(0, "Load Program    ", 16);
		sprintf(buff, "%c  %03d         ", (char) ('A' + SelectedChannel),
				tartgetProgramNo);

		lcdWriteText(1, &buff[0], 16);
	}
}

void ON_PUSH_PROGRAM(void) {
	if(LcdMenuState == LCD_STATE_DEFAULT || LcdMenuState == LCD_STATE_MENU){
		lightsOff();
		ShowProgramMenu(0);
		return;
	}
	if(LcdMenuState == LCD_STATE_PROGRAM_MENU){
		ShowProgramMenu(1);
		return;
	}
}

void ON_PUSH_SHIFT(void) {
	is_pressed_key_SHIFT = 1;
	if (LcdMenuState == LCD_STATE_DEFAULT) {
		refreshLCD();
	}
}

void ON_RELEASE_SHIFT(void) {
	is_pressed_key_SHIFT = 0;
	if (LcdMenuState == LCD_STATE_DEFAULT) {
		refreshLCD();
	}
}

void ON_PUSH_A(void) {
	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM || LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		SelectedChannel = 0;
		updateSelectProgram();
	}

	if(LcdMenuState == LCD_STATE_SEQ_EDIT){
		//step rec
		sequencer.sequenceData[sequencer.cursor_index].a = !sequencer.sequenceData[sequencer.cursor_index].a;
		ShowSequencerEditMode(&sequencer, 0, midiConfig.syncMode);
		if (!is_pressed_key_SHIFT) {
			return;
		}
	}

	Gen_trig(&synth[0], 1.0f);
}

void ON_PUSH_B(void) {
	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM || LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		SelectedChannel = 1;
		updateSelectProgram();
	}

	if (LcdMenuState == LCD_STATE_SEQ_EDIT) {
		//step rec
		sequencer.sequenceData[sequencer.cursor_index].b =
				!sequencer.sequenceData[sequencer.cursor_index].b;
		ShowSequencerEditMode(&sequencer, 0, midiConfig.syncMode);
		if (!is_pressed_key_SHIFT) {
			return;
		}
	}

	Gen_trig(&synth[1], 1.0f);
}

void ON_PUSH_C(void) {
	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM || LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		SelectedChannel = 2;
		updateSelectProgram();
	}

	if (LcdMenuState == LCD_STATE_SEQ_EDIT) {
		//step rec
		sequencer.sequenceData[sequencer.cursor_index].c =
				!sequencer.sequenceData[sequencer.cursor_index].c;
		ShowSequencerEditMode(&sequencer, 0, midiConfig.syncMode);
		if (!is_pressed_key_SHIFT) {
			return;
			}
	}

	Gen_trig(&synth[2], 1.0f);
}

void ON_PUSH_D(void) {
	if (LcdMenuState == LCD_STATE_LOAD_PROGRAM || LcdMenuState == LCD_STATE_SAVE_PROGRAM) {
		SelectedChannel = 3;
		updateSelectProgram();
	}

	if (LcdMenuState == LCD_STATE_SEQ_EDIT) {
		//step rec
		sequencer.sequenceData[sequencer.cursor_index].d =
				!sequencer.sequenceData[sequencer.cursor_index].d;
		ShowSequencerEditMode(&sequencer, 0, midiConfig.syncMode);
		if (!is_pressed_key_SHIFT) {
			return;
		}
	}
	Gen_trig(&synth[3], 1.0f);
}

static void refreshLCD() {
	refresh1stLine();
	updateLCD();
}

static void updateLCD() {

	if (LcdMenuState != LCD_STATE_DEFAULT) {
		return;
	}

	static const char* MOD_TYPE_NAMES[] = { "FM", "AM", "FM+Nz" };

	int param = 0;
	switch (selected_row) {
	case PARAM_GROUP_OSC:
		switch (selected_col) {
		case ROTARY_ENCODER_A:
			write_LCD_PARAM_NAME(
					LCDM_PARAMETER_TEXT[selected_row][selected_col - 1],
					NameOf(Gen_get_carr_wave(CURR_GEN)));
			return;
		case ROTARY_ENCODER_B:
			if (is_pressed_key_SHIFT) {
				param = Gen_get_carr_fine(CURR_GEN);
			} else {
				param = Gen_get_carr_coarse(CURR_GEN);
			}
			break;
		case ROTARY_ENCODER_C:
			param = Gen_get_carr_moddepth(CURR_GEN);
			break;
		case ROTARY_ENCODER_D:
			write_LCD_PARAM_NAME(
					LCDM_PARAMETER_TEXT_SHIFT[selected_row][selected_col - 1],
					MOD_TYPE_NAMES[(int) synth[SelectedChannel].modtype]);
			return;
		}
		break;

	case PARAM_GROUP_AMP:
		switch (selected_col) {
		case ROTARY_ENCODER_A:
			if (is_pressed_key_SHIFT) {
				param = synth[SelectedChannel].i_pan;
			} else {
				param = Gen_get_carr_level(CURR_GEN);
			}
			break;
		case ROTARY_ENCODER_B:
			param = Gen_get_carr_attack(CURR_GEN);
			break;
		case ROTARY_ENCODER_C:
			if (is_pressed_key_SHIFT) {
				param = Gen_get_carr_slope(CURR_GEN);
			} else {
				param = Gen_get_carr_hold(CURR_GEN);
			}
			break;
		case ROTARY_ENCODER_D:
			param = Gen_get_carr_release(CURR_GEN);
			break;
		}
		break;
	case PARAM_GROUP_MODU:
		switch (selected_col) {
		case ROTARY_ENCODER_A:
			param = Gen_get_fm_harmonics(CURR_GEN); //freq
			break;
		case ROTARY_ENCODER_B:
			param = Gen_get_fm_attack(CURR_GEN);  //a
			break;
		case ROTARY_ENCODER_C:
			if (is_pressed_key_SHIFT) {
				param = Gen_get_fm_slope(CURR_GEN);
			} else {
				param = Gen_get_fm_hold(CURR_GEN);  //h
			}
			break;
		case ROTARY_ENCODER_D:
			param = Gen_get_fm_release(CURR_GEN);  //r
			break;
		}
		break;
	case PARAM_GROUP_BEND:
		switch (selected_col) {
		case ROTARY_ENCODER_A:
			if (is_pressed_key_SHIFT) {
				param = Gen_get_bend_velocity_sense(CURR_GEN);
			} else {
				param = Gen_get_bend_amount(CURR_GEN);
			}
			break;
		case ROTARY_ENCODER_B:
			param = Gen_get_bend_attack(CURR_GEN);
			break;
		case ROTARY_ENCODER_C:
			if (is_pressed_key_SHIFT) {
				param = Gen_get_bend_slope(CURR_GEN);
			} else {
				param = Gen_get_bend_hold(CURR_GEN);
			}
			break;
		case ROTARY_ENCODER_D:
			param = Gen_get_bend_release(CURR_GEN);
			break;
		}
		break;

	case PARAM_GROUP_NOISE:
		switch (selected_col) {
		case ROTARY_ENCODER_A:
			param = Gen_get_noise_level(CURR_GEN);
			break;
		case ROTARY_ENCODER_B:
			param = Gen_get_noise_attack(CURR_GEN);
			break;
		case ROTARY_ENCODER_C:
			if (is_pressed_key_SHIFT) {
				param = Gen_get_noise_slope(CURR_GEN);
			} else {
				param = Gen_get_noise_hold(CURR_GEN);
			}

			break;
		case ROTARY_ENCODER_D:
			param = Gen_get_noise_release(CURR_GEN);
			break;
		}
		break;

	case PARAM_GROUP_FILTER:
		switch (selected_col) {

		case ROTARY_ENCODER_A:
			param = Gen_get_filter_type(CURR_GEN);
			write_LCD_PARAM_NAME(
					LCDM_PARAMETER_TEXT[selected_row][selected_col - 1],
					FILTER_TYPE[param]);
			return;

		case ROTARY_ENCODER_B:
			param = Gen_get_filter_cutoff(CURR_GEN);
			break;

		case ROTARY_ENCODER_C:
			param = Gen_get_filter_resonance(CURR_GEN);
			break;

		case ROTARY_ENCODER_D: {
			if (is_pressed_key_SHIFT) {
				param = Gen_get_filter_decay(CURR_GEN);
			} else {
				param = Gen_get_filter_amount(CURR_GEN);
			}
			break;

		}
		}
		break;

	case PARAM_GROUP_LFO:
		switch (selected_col) {
		case ROTARY_ENCODER_A: {
			const char* destName = LFO_getDest_Name(
					&(synth[SelectedChannel].lfo));
			write_LCD_PARAM_NAME(
					LCDM_PARAMETER_TEXT[selected_row][selected_col - 1],
					destName);
			return;
		}

		case ROTARY_ENCODER_B: {
			const char* waveform = LFO_getWave_Name(
					synth[SelectedChannel].lfo.waveform);
			write_LCD_PARAM_NAME(
					LCDM_PARAMETER_TEXT[selected_row][selected_col - 1],
					waveform);
			return;
		}

		case ROTARY_ENCODER_C: {
			param = LFO_getSpeed(&(CURR_GEN)->lfo);
		}
			break;

		case ROTARY_ENCODER_D: {
			param = LFO_getDepth(&(CURR_GEN)->lfo);
		}

			break;

		}
		break;
	}

	const char* str =
			is_pressed_key_SHIFT ?
					LCDM_PARAMETER_TEXT_SHIFT[selected_row][selected_col - 1] :
					LCDM_PARAMETER_TEXT[selected_row][selected_col - 1];

	write_LCD_PARAM(str, param);

}

void write_LCD_PARAM(const char *name, int value) {
	static char str[17];
	sprintf(str, "%-12s %03d", name, value);
	lcdWriteText(1, (char*) str, 16);
}

void write_LCD_PARAM_NAME(const char *param_name, const char* value) {
	static char str[17];
	sprintf(str, "%-5s: %-9s ", param_name, value);
	lcdWriteText(1, (char*) str, 16);
}

void revert() {
	StopSequencer(&sequencer);
	for (int i = 0; i < 4; i++) {
		Tone t;
		size_t tone_size = sizeof(Tone);
		I2CFlash_Read(&eeprom, ROM_ADDRESS_TONE_TMP + (i * 64), (uint8_t*) &t,
				tone_size);
		waitUntilReady(&eeprom);
		ToneCopyToGen(&synth[i], &t);
	}
	I2CFlash_LoadSequenceData(&eeprom, &sequencer);
}

/**
 * GATE IN
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	//HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
	if (USE_EXTERNAL_GATE) {
		switch (GPIO_Pin) {
		case TRG1_Pin:
			Gen_trig(&synth[0], 1.0f);
			break;
		case TRG2_Pin:
			Gen_trig(&synth[1], 1.0f);
			break;
		case TRG3_Pin:
			Gen_trig(&synth[2], 1.0f);
			break;
		case TRG4_Pin:
			Gen_trig(&synth[3], 1.0f);
		}
	}
}

/**
 * On Received MIDI Signal
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		if (tmp_byte == 0xFE) {
			//Ignore Active Sensing
		}else{
			//RECEIVE MIDI
			MidiParser_PushByte(tmp_byte);
		}
	}
	HAL_UART_Receive_IT(&huart1, (uint8_t*) &tmp_byte, 1);
}

void ON_RECEIVE_NOTE_ON(uint8_t ch, uint8_t note, uint8_t velocity) {

	if (velocity == 0) {
		return;
	}

	float v = 1.0f;
	switch (midiConfig.velocityCurve) {
	case Linear:
		v = VelocityCurve_Linear[velocity];
		break;
	case Exponential:
		v = VelocityCurve_Exponential[velocity];
		break;
	case Fixed:
		break;
	}

	if (midiConfig.channel_A == ch) {
		if (midiConfig.noteNo_A == NOTEMAP_SCALE) {
			Gen_trig_note(&synth[0], v, note);
		} else if (midiConfig.noteNo_A == note) {
			Gen_trig_note(&synth[0], v, 60);
		}
	}

	if (midiConfig.channel_B == ch) {
		if (midiConfig.noteNo_B == NOTEMAP_SCALE) {
			Gen_trig_note(&synth[1], v, note);
		} else if (midiConfig.noteNo_B == note) {
			Gen_trig_note(&synth[1], v, 60);
		}
	}

	if (midiConfig.channel_C == ch) {
		if (midiConfig.noteNo_C == NOTEMAP_SCALE) {
			Gen_trig_note(&synth[2], v, note);
		} else if (midiConfig.noteNo_C == note) {
			Gen_trig_note(&synth[2], v, 60);
		}
	}

	if (midiConfig.channel_D == ch) {
		if (midiConfig.noteNo_D == NOTEMAP_SCALE) {
			Gen_trig_note(&synth[3], v, note);
		} else if (midiConfig.noteNo_D == note) {
			Gen_trig_note(&synth[3], v, 60);
		}
	}

}

void ON_RECEIVE_CONTROL_CHANGE(uint8_t ch, uint8_t no, uint8_t value) {

	const uint32_t current_id = selected_row << 8 | (selected_col-1) << 4
			| is_pressed_key_SHIFT;

	for (int i = 0; i < 4; i++) {
		Gen* g = &synth[i];

		switch (i) {
		case 0:
			if (midiConfig.channel_A == ch) {
				break;
			} else {
				continue;
			}
		case 1:
			if (midiConfig.channel_B == ch) {
				break;
			} else {
				continue;
			}
		case 2:
			if (midiConfig.channel_C == ch) {
				break;
			} else {
				continue;
			}
		case 3:
			if (midiConfig.channel_D == ch) {
				break;
			} else {
				continue;
			}
		}

		switch (no) {
		case CCNo_VOLUME:
			//TODO
			break;

		case CCNo_EXPRESSION:
			//TODO
			break;

		case CCNo_OSC_WAVE: {
			Waveform wf;
			if (value < 43) {
				wf = Waveform_SINE;
			} else if (value < 86) {
				wf = Waveform_SAW;
			} else {
				wf = Waveform_PULSE;
			}
			Gen_set_carr_wave(g, wf);
		}
			break;

		case CCNo_OSC_PITCH:
			Gen_set_carr_coarse(g, value);
			if (current_id == ID_Osc_Pitch) {
				updateLCD();
			}
			break;

		case CCNo_OSC_FINE:
			Gen_set_carr_fine(g, ((int) value) - 64);
			if (current_id == ID_Osc_Fine) {
				updateLCD();
			}
			break;

		case CCNo_OSC_MOD_DEPTH:
			Gen_set_carr_moddepth(g, value);
			if (current_id == ID_Osc_ModDpth) {
				updateLCD();
			}
			break;

		case CCNo_OSC_MOD_TYPE:

			if( value <42){
				Gen_set_modtype(g, MODTYPE_FM);
			}else if(value < 84){
				Gen_set_modtype(g, MODTYPE_AM);
			}else{
				Gen_set_modtype(g, MODTYPE_FM_NOISE_ROUTED);
			}
			if (current_id == ID_Osc_ModType) {
				updateLCD();
			}
			break;

		case CCNo_AMP_LEVEL:
			Gen_set_carr_level(g, value);
			if (current_id == ID_Amp_Level) {
				updateLCD();
			}
			break;

		case CCNo_AMP_PANPOT:
			Gen_set_pan(g, ((int) value) - 64); //TODO FIX NEGATIVE VALUE
			if (current_id == ID_Amp_Pan) {
				updateLCD();
			}
			break;

		case CCNo_AMP_ENV_ATK:
			Gen_set_carr_attack(g, value);
			if (current_id == ID_Amp_Atk) {
				updateLCD();
			}
			break;

		case CCNo_AMP_ENV_HLD:
			Gen_set_carr_hold(g, value);
			if (current_id == ID_Amp_Hld) {
				updateLCD();
			}
			break;

		case CCNo_AMP_ENV_SLP:
			Gen_set_carr_slope(g, value);
			if (current_id == ID_Amp_Slp) {
				updateLCD();
			}
			break;

		case CCNo_AMP_ENV_REL:
			Gen_set_carr_release(g, value);
			if (current_id == ID_Amp_Rel) {
				updateLCD();
			}
			break;

		case CCNo_MOD_FREQ:
			Gen_set_fm_harmonics(g, LIMIT((int )value - 64, 63, -63));
			if (current_id == ID_Mod_Freq) {
				updateLCD();
			}
			break;

		case CCNo_MOD_ENV_ATK:

			Gen_set_fm_attack(g, value);
			if (current_id == ID_Mod_Atk) {
				updateLCD();
			}
			break;

		case CCNo_MOD_ENV_HLD:
			Gen_set_fm_hold(g, value);
			if (current_id == ID_Mod_Hld) {
				updateLCD();
			}
			break;

		case CCNo_MOD_ENV_SLP:
			Gen_set_fm_slope(g, value);
			if (current_id == ID_Mod_Slp) {
				updateLCD();
			}
			break;

		case CCNo_MOD_ENV_REL:
			Gen_set_fm_release(g, value);
			if (current_id == ID_Mod_Rel) {
				updateLCD();
			}
			break;

		case CCNo_BEND_AMT:
			Gen_set_bend_amount(g, value);
			if (current_id == ID_Bend_Amt) {
				updateLCD();
			}
			break;

		case CCNo_BEND_VELSENSE:
			Gen_set_bend_velocity_sense(g, value);
			if (current_id == ID_Bend_velSns) {
				updateLCD();
			}
			break;

		case CCNo_BEND_ENV_ATK:
			Gen_set_bend_attack(g, value);
			if (current_id == ID_Bend_Atk) {
				updateLCD();
			}
			break;

		case CCNo_BEND_ENV_HLD:
			Gen_set_bend_hold(g, value);
			if (current_id == ID_Bend_Hld) {
				updateLCD();
			}
			break;

		case CCNo_BEND_ENV_SLP:
			Gen_set_bend_slope(g, value);
			if (current_id == ID_Bend_Slp) {
				updateLCD();
			}
			break;

		case CCNo_BEND_ENV_REL:
			Gen_set_bend_release(g, value);
			if (current_id == ID_Bend_Rel) {
				updateLCD();
			}
			break;

		case CCNo_NOISE_LEVEL:
			Gen_set_noise_level(g, value);
			if (current_id == ID_Noise_Lv) {
				updateLCD();
			}
			break;

		case CCNo_NOISE_ENV_ATK:
			Gen_set_noise_attack(g, value);
			if (current_id == ID_Noise_Atk) {
				updateLCD();
			}
			break;

		case CCNo_NOISE_ENV_HLD:
			Gen_set_noise_hold(g, value);
			if (current_id == ID_Noise_Hld) {
				updateLCD();
			}
			break;

		case CCNo_NOISE_ENV_SLP:
			Gen_set_noise_slope(g, value);
			if (current_id == ID_Noise_Slp) {
				updateLCD();
			}
			break;

		case CCNo_NOISE_ENV_REL:
			Gen_set_noise_release(g, value);
			if (current_id == ID_Noise_Rel) {
				updateLCD();
			}
			break;

		case CCNo_FLT_TYPE:
			Gen_set_filter_type(g, value);
			if (current_id == ID_Flt_Type) {
				updateLCD();
			}
			break;

		case CCNo_FLT_CUTOFF:
			Gen_set_filter_cutoff(g, value);
			if (current_id == ID_Flt_Coff) {
				updateLCD();
			}
			break;

		case CCNo_FLT_RESO:

			Gen_set_filter_resonance(g, value/18);
			if (current_id == ID_Flt_Reso) {
				updateLCD();
			}
			break;

		case CCNo_FLT_AMOUNT:
			Gen_set_filter_amount(g, value);
			if (current_id == ID_Flt_Amt) {
				updateLCD();
			}
			break;

		case CCNo_FLT_DECAY:
			Gen_set_filter_decay(g, value);
			if (current_id == ID_Flt_Dcy) {
				updateLCD();
			}
			break;

		case CCNo_LFO_DEST:
			Gen_set_lfo_dest(g, value);
			if (current_id == ID_Lfo_dest) {
				updateLCD();
			}
			break;

		case CCNo_LFO_WAVE: {
			LFO_WAVEFORM wv;
			switch (value) {
			case LFO_SIN:
				wv = LFO_SIN;
				break;
			case LFO_TRI:
				wv = LFO_TRI;
				break;
			default:
				wv = LFO_SIN;
				break;
			}
			LFO_setWave(&(g->lfo), wv);
			if (current_id == ID_Lfo_Wave) {
				updateLCD();
			}
		}
			break;

		case CCNo_LFO_SPEED:
			Gen_set_lfo_speed(g, value);
			if (current_id == ID_Lfo_Spd) {
				updateLCD();
			}
			break;

		case CCNo_LFO_DEPTH:
			Gen_set_lfo_depth(g, value);
			if (current_id == ID_Lfo_Dpt) {
				updateLCD();
			}
			break;
		}
	}
}


void ON_RECEIVE_PROGRAM_CHANGE(uint8_t ch, uint8_t program) {
	if (program < 0 || program > 127) {
		return;
	}
	Tone t;
	if (midiConfig.channel_A == ch) {
		if (ReadTone(&eeprom, program, &t) == HAL_OK)
			;
		ToneCopyToGen(&synth[0], &t);
	}
	if (midiConfig.channel_B == ch) {
		if (ReadTone(&eeprom, program, &t) == HAL_OK)
			;
		ToneCopyToGen(&synth[1], &t);
	}
	if (midiConfig.channel_C == ch) {
		if (ReadTone(&eeprom, program, &t) == HAL_OK)
			;
		ToneCopyToGen(&synth[2], &t);
	}
	if (midiConfig.channel_D == ch) {
		if (ReadTone(&eeprom, program, &t) == HAL_OK)
			;
		ToneCopyToGen(&synth[3], &t);
	}
}

void MIDI_RAW_MESSAGE_CALLBACK(uint8_t *bytes, uint16_t size) {
	//Echoback
	if (midiConfig.echoBack) {
		HAL_UART_Transmit_IT(&huart1, bytes, size);
	}
}

void SEQUENCER_BEAT_CALLBACK(uint8_t * step_array){
	uint8_t step;
	Notes* n;

	step = sequencer.step[0];
	n = &(sequencer.sequenceData[step]);
	if (n->a) {
		Gen_trig(&synth[0], 1.0f);
		OnBeatRdmzer(&sequencer, 0);
	}

	step = sequencer.step[1];
	n = &(sequencer.sequenceData[step]);
	if (n->b) {
		Gen_trig(&synth[1], 1.0f);

		OnBeatRdmzer(&sequencer, 1);
	}

	step = sequencer.step[2];
	n = &(sequencer.sequenceData[step]);
	if (n->c) {
		Gen_trig(&synth[2], 1.0f);

		OnBeatRdmzer(&sequencer, 2);
	}

	step = sequencer.step[3];
	n = &(sequencer.sequenceData[step]);
	if (n->d) {
		Gen_trig(&synth[3], 1.0f);

		OnBeatRdmzer(&sequencer, 3);
	}

}


void ON_PROGRESS_SEQUENCER_CLOCK() {
	if (midiConfig.syncMode == InternalClock) {
		HAL_UART_Transmit_IT(&huart1,
				(uint8_t*) &MIDI_SYSRT_MESSAGE_TIMING_CLOCK, 1);
	}
}

void ON_START_SEQUENCER() {
	if (midiConfig.syncMode == InternalClock) {
		HAL_UART_Transmit_IT(&huart1, (uint8_t*) &MIDI_SYSRT_MESSAGE_START, 1);
	}
}

void ON_STOP_SEQUENCER() {
	if (midiConfig.syncMode == InternalClock) {
		HAL_UART_Transmit_IT(&huart1, (uint8_t*) &MIDI_SYSRT_MESSAGE_STOP, 1);
	}
}

//// SYNC EXTERNAL MIDI CLOCK

void ON_RECEIVE_CLOCK() {
	if (midiConfig.syncMode == ExternalClock && sequencer.status == SEQ_RUNNING) {
		ClockSequencer(&sequencer);
	}
}

void ON_RECEIVE_START() {
	if (midiConfig.syncMode == ExternalClock && sequencer.status == SEQ_IDLING) {
		StartSequencer(&sequencer);
	}
}

void ON_RECEIVE_STOP() {
	if (midiConfig.syncMode == ExternalClock  && sequencer.status == SEQ_RUNNING) {
		StopSequencer(&sequencer);
	}
}

void ON_RECEIVE_CONTINUE() {
	if (midiConfig.syncMode == ExternalClock&& sequencer.status == SEQ_IDLING) {
		StartSequencer(&sequencer);
	}
}

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN 5 */

	/* Infinite loop */
	lcdInit(&hi2c3, I2C_LCD_ADDRESS, 2, 16);

	//init led
	HAL_GPIO_WritePin(BLED7_GPIO_Port, BLED7_Pin, GPIO_PIN_SET);

	refresh1stLine();
	updateLCD();

	/* Infinite loop */
	for (;;) {
		osDelay(50);
		if (lcd_text_buf1.dirt > 0) {
			lcdSetCursorPosition(0, 0);
			lcdPrintStr((uint8_t*) lcd_text_buf1.text, lcd_text_buf1.length);
			lcd_text_buf1.dirt--;
		}
		if (lcd_text_buf2.dirt > 0) {
			lcdSetCursorPosition(0, 1);
			lcdPrintStr((uint8_t*) lcd_text_buf2.text, lcd_text_buf2.length);
			lcd_text_buf2.dirt--;
		}
	}
  /* USER CODE END 5 */ 
}

/* StartCdcTask function */
void StartCdcTask(void const * argument)
{
  /* USER CODE BEGIN StartCdcTask */
	static osEvent event;
	/* Infinite loop */
	for (;;) {
		osDelay(100);
		event = osMessageGet(queue01Handle, osWaitForever);
		if (event.status == osEventMessage) {
			Tone t;
			InitTone(&t);
			ToneCopyFromGen(&t, &synth[event.value.v & 0xF]);
			printTone(&t);
		}
	}
  /* USER CODE END StartCdcTask */
}

/* StartAccessStorageTask function */
void StartAccessStorageTask(void const * argument)
{
  /* USER CODE BEGIN StartAccessStorageTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
  }
  /* USER CODE END StartAccessStorageTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM10 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (midiConfig.syncMode == InternalClock && htim->Instance == TIM13) {
		ClockSequencer(&sequencer);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM10) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
