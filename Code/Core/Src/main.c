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
#include "buzzer.h"
#include "display.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

//#define noteSpace 24
//#define noteSpace2 12
//#define _16thNote 110
//#define dotted16th 165
//#define _8thNote 220
//#define dotted8th 330
#define _4thNote 440
#define dotted4th 660
#define halfNote 880
#define dottedHalf 1320
#define wholeNote 1760

#define noteSpace 12
#define noteSpace2 6
#define _16thNote 55
#define dotted16th 83
#define _8thNote 110
#define dotted8th 165


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void writeLED(uint8_t LED, uint8_t state);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t LED_index = 0;



//static const uint8_t smiley[64] = {
//0, 0, 0, 0, 0, 0, 0, 0,
//0, 0, 1, 0, 0, 1, 0, 0,
//0, 0, 1, 0, 0, 1, 0, 0,
//0, 0, 0, 0, 0, 0, 0, 0,
//0, 0, 0, 0, 0, 0, 0, 0,
//1, 0, 0, 0, 0, 0, 0, 1,
//0, 1, 1, 1, 1, 1, 1, 0,
//0, 0, 0, 0, 0, 0, 0, 0
//};
//


//Easy way to see if data being processed properly
static const uint32_t rows_test[8] = {
0xFFFFFFFF,
0x0,
0xFFFFFFFF,
0x0,
0xFFFFFFFF,
0x0,
0xFFFFFFFF,
0x0,
};



//static const uint32_t cross_test[64] = {
//1, 0, 0, 0, 0, 0, 0, 1,
//0, 1, 0, 0, 0, 0, 1, 0,
//0, 0, 1, 0, 0, 1, 0, 0,
//0, 0, 0, 1, 1, 0, 0, 0,
//0, 0, 0, 1, 1, 0, 0, 0,
//0, 0, 1, 0, 0, 1, 0, 0,
//0, 1, 0, 0, 0, 0, 1, 0,
//1, 0, 0, 0, 0, 0, 0, 1
//};
//
//static const uint8_t all_on[64] = {
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1,
//1, 1, 1, 1, 1, 1, 1, 1
//};




display_HandleTypeDef display;
buzzer_HandleTypeDef buzzer;
//2D array, frequency and duration

uint16_t darude_bassline[][2] = {
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace },
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace },
		{ NOTE_B2, _8thNote }, { 0, _8thNote + noteSpace },
		{ NOTE_B2, dotted8th}, { 0, dotted8th + noteSpace },
		{ NOTE_B2, dotted8th}, { 0, dotted8th + noteSpace },
		{ NOTE_E3, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_E3, dotted8th }, { 0, dotted8th + noteSpace },
		{ NOTE_E3, dotted8th }, { 0, dotted8th + noteSpace },
		{ NOTE_D3, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_D3, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_D3, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_A2, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_E3, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_D4, _8thNote }, { 0, _8thNote + noteSpace},
};

uint16_t darude_drop[][2] = {
		{ NOTE_B4, _16thNote }, { 0, _16thNote + noteSpace2 },
		{ NOTE_B4, _16thNote }, { 0, _16thNote + noteSpace2 },
		{ NOTE_B4, _16thNote }, { 0, _16thNote + noteSpace2 },
		{ NOTE_B4, _16thNote }, { 0, _16thNote + noteSpace2 },
		{ NOTE_B4, _8thNote}, { 0, _8thNote + noteSpace2 },
		{ NOTE_B2, _8thNote}, { 0, _8thNote + noteSpace },
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_B2, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_E3, _8thNote }, { 0, _8thNote + noteSpace },
		{ NOTE_E3, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_E3, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_D3, _8thNote }, { 0, _8thNote + noteSpace},
		{ NOTE_D3, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_D3, dotted8th }, { 0, dotted8th + noteSpace},
		{ NOTE_A2, _8thNote }, { 0, _8thNote + noteSpace},
};
/*


tone(7,nA2,_8thNote);
delay(_8thNote + noteSpace);
*/



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
  MX_RTC_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, 1);
  HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
  HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
  HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
  HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);
  HAL_GPIO_WritePin(R5_GPIO_Port, R5_Pin, 1);
  HAL_GPIO_WritePin(R6_GPIO_Port, R6_Pin, 1);
  HAL_GPIO_WritePin(R7_GPIO_Port, R7_Pin, 1);

  HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, 0);
  HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, 0);
  HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, 0);
  HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, 0);
  HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, 0);
  HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, 0);
  HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, 0);
  HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, 0);

	// Always calibrate ADC first
	HAL_ADCEx_Calibration_Start(&hadc1);

	// Start the conversion sequence
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 0xFFFF);
	//uint32_t vrefint_raw = HAL_ADC_GetValue(&hadc1);

	/*
	 * This macro calculates the vdda voltage (as a uint32_t representing the voltage in milliVolts)
	 * using the vref internal raw adc value, and the internal calibration value in ROM
	 */
//	uint32_t vdda_voltage = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_raw, ADC_RESOLUTION_12B);
//	__NOP();


	display_init(&display, &htim3, TIM_CHANNEL_1,  &htim14, TIM_CHANNEL_1);




//	buzzer_init(&buzzer, &htim1, TIM_CHANNEL_2, TIM_CHANNEL_3, &htim16);
//	buzzer.melody = darude_drop;
//	buzzer.melody_length = sizeof(darude_drop) / sizeof(darude_drop[0]);
//	buzzer_play_melody(&buzzer, 2);
	//Working pretty well, but there is a time delay between loops of a melody, Need to figure out why
	//Maybe record audio of correct loop and compare to bad loop to see difference


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  if (btn1_val == 0){
//		  btn1_val = 1;
//	  }
//	  else {
//		  btn1_val = 0;
//	  }
//	  //HAL_Delay(1000);
//
//
////	  uint8_t btn1_val = HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin);
//	  uint8_t btn2_val = HAL_GPIO_ReadPin(BTN2_GPIO_Port, BTN2_Pin);
//	  uint8_t btn3_val = HAL_GPIO_ReadPin(BTN3_GPIO_Port, BTN3_Pin);
//
//	  //btn1_val = 0;
//	  if (btn1_val == 0) {
//
//		  for (uint8_t i = 0; i < 64; i++){
//			  writeLED(i, all_on[i]);
//			  //HAL_Delay(1);
//		  }
//		  writeLED(0, 0);	//clear the last drawn pixel
//	  }
//
//	  else {
//		  for (uint8_t i = 0; i < 64; i++){
//			  writeLED(i, cross_test[i]);
//			  //HAL_Delay(1);
//		  }
//		  writeLED(0, 0);	//clear the last drawn pixel
//	  }
//
////	  if (btn2_val == 0) {
////
////		  for (uint8_t i = 0; i < 64; i++){
////			  writeLED(i, cross_test[i]);
////			  //HAL_Delay(1);
////		  }
////		  writeLED(0, 0);	//clear the last drawn pixel
////	  }
////
////	  if (btn3_val == 0) {
////
////		  for (uint8_t i = 0; i < 64; i++){
////			  writeLED(i, smiley[i]);
////			  //HAL_Delay(1);
////		  }
////		  writeLED(0, 0);	//clear the last drawn pixel
////	  }





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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.SubSeconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
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
  htim3.Init.Prescaler = 16 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 41 - 1;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 1600 - 1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 416 - 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 16000 - 1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, C7_Pin|C6_Pin|R3_Pin|R4_Pin
                          |R7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, C5_Pin|C4_Pin|C3_Pin|C2_Pin
                          |C1_Pin|C0_Pin|R0_Pin|R1_Pin
                          |R2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, R5_Pin|R6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : C7_Pin C6_Pin R3_Pin R4_Pin
                           R7_Pin */
  GPIO_InitStruct.Pin = C7_Pin|C6_Pin|R3_Pin|R4_Pin
                          |R7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : C5_Pin C4_Pin C3_Pin C2_Pin
                           C1_Pin C0_Pin R0_Pin R1_Pin
                           R2_Pin */
  GPIO_InitStruct.Pin = C5_Pin|C4_Pin|C3_Pin|C2_Pin
                          |C1_Pin|C0_Pin|R0_Pin|R1_Pin
                          |R2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : R5_Pin R6_Pin */
  GPIO_InitStruct.Pin = R5_Pin|R6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN1_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN2_Pin BTN3_Pin */
  GPIO_InitStruct.Pin = BTN2_Pin|BTN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim == buzzer.interrupt_timer) {
		buzzer_interrupt(&buzzer);
	}

	else if (htim == display.frame_timer){
		frame_interrupt(&display);
	}

//	else if (htim == display.multiplex_timer){
//		multiplex_interrupt(&display);
//	}
}


void writeLED(uint8_t LED, uint8_t state){

	uint8_t row = LED/8;	//Find row, discard decimal part
	uint8_t col = LED%8;	//Find column

	__NOP();

	//clear the last drawn pixel
	HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, 1);
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);
	HAL_GPIO_WritePin(R5_GPIO_Port, R5_Pin, 1);
	HAL_GPIO_WritePin(R6_GPIO_Port, R6_Pin, 1);
	HAL_GPIO_WritePin(R7_GPIO_Port, R7_Pin, 1);
	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, 0);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, 0);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, 0);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, 0);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, 0);
	HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, 0);
	HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, 0);
	HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, 0);

	if (state == 1){

		switch (row){

		case 0:
			HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, 0);
			break;
		case 1:
			HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 0);
			break;
		case 2:
			HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 0);
			break;
		case 3:
			HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 0);
			break;
		case 4:
			HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 0);
			break;
		case 5:
			HAL_GPIO_WritePin(R5_GPIO_Port, R5_Pin, 0);
			break;
		case 6:
			HAL_GPIO_WritePin(R6_GPIO_Port, R6_Pin, 0);
			break;
		case 7:
			HAL_GPIO_WritePin(R7_GPIO_Port, R7_Pin, 0);
			break;

		default:

		}

		switch (col){

		case 0:
			HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, 1);
			break;
		case 1:
			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, 1);
			break;
		case 2:
			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, 1);
			break;
		case 3:
			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, 1);
			break;
		case 4:
			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, 1);
			break;
		case 5:
			HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, 1);
			break;
		case 6:
			HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, 1);
			break;
		case 7:
			HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, 1);
			break;

		default:

		}
	}
	__NOP();
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
