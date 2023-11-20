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
#include <limits.h>
#include <float.h>


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MEASURE_CYCLE 99;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

int measure_count = 0;

int interval_time = 0;
int analog_value;
int SAMPLE_SIZE = 10;
int sample_index = 0;
float moving_avg_sample[10] = { 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000, 5000 };

float moving_avg_sample_sum;
float moving_avg_result_value;

int current_time = 0;
int pre_time = 0;
int measure_pre_time = 0;

float bit_voltage;
float voltage;

float avg_10;
float avg_10_data[10];
float avg_100;
float avg_100_data[10];
float avg_1000;
float avg_1000_data[10];

int array_10_index;
int array_100_index;
int array_1000_index;
float full_sum_avg10;
float full_sum_avg100;

float* arr_address;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_MEMORYMAP_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float Average_Calculate10() {

  float min = FLT_MAX;
  float max = FLT_MIN;
  int min_index = 0;
  int max_index = 0;

  float sum = 0.0;

  for(int i = 0; i < 10; i++) {
    if(min > avg_10_data[i]) {
      min = avg_10_data[i];
      min_index = i;
    }

    if(max < avg_10_data[i]) {
      max = avg_10_data[i];
      max_index = i;
    }

  }
  for(int j = 0; j < 10; j++) {
    full_sum_avg10 = full_sum_avg10 + avg_10_data[j];
    if(max_index == j || min_index == j){
      continue;
    } else {
      sum = sum + avg_10_data[j];
    }

  }
  return  sum / 8;
}

float Average_Calculate100() {

  float min = FLT_MAX;
  float max = FLT_MIN;
  int min_index = 0;
  int max_index = 0;

  float sum = 0.0;

  for(int i = 0; i < 10; i++) {
    if(min > avg_100_data[i]) {
      min = avg_100_data[i];
      min_index = i;
    }

    if(max < avg_100_data[i]) {
      max = avg_100_data[i];
      max_index = i;
    }

  }
  for(int i = 0; i < 10; i++) {
    full_sum_avg100 = full_sum_avg100 + avg_100_data[i];
    if(max_index == i || min_index == i){
      continue;
    } else {
      sum = sum + avg_100_data[i];
    }

  }
  return  sum / 8;
}

float Average_Calculate1000() {

  float min = FLT_MAX;
  float max = FLT_MIN;
  int min_index = 0;
  int max_index = 0;

  float sum = 0.0;

  for(int i = 0; i < 10; i++) {
    if(min > avg_1000_data[i]) {
      min = avg_1000_data[i];
      min_index = i;
    }

    if(max < avg_1000_data[i]) {
      max = avg_1000_data[i];
      max_index = i;
    }

  }
  for(int i = 0; i < 10; i++) {
    if(max_index == i || min_index == i){
      continue;
    } else {
      sum = sum + avg_1000_data[i];
    }
  }
  return  sum / 8;
}

void Value_Allocate(float value) {
  int array_length = sizeof(avg_10_data) / sizeof(float);
  array_10_index = measure_count % array_length;

  avg_10_data[array_10_index] = value;

  if(measure_count > 0 && array_10_index == 9) {
    array_100_index = measure_count / 10 % array_length;

    avg_10 = Average_Calculate10();
    avg_100_data[array_100_index] = full_sum_avg10 / 10;
    full_sum_avg10 = 0.0;
  }

  if(measure_count > 10 && array_100_index == 9) {
    array_1000_index = measure_count / 100 % array_length;

    avg_100 = Average_Calculate100();
    avg_1000_data[array_1000_index] = full_sum_avg100 / 10;
    full_sum_avg100 = 0.0;
  }

  if(measure_count > 10 && array_1000_index == 9) {
    avg_1000 = Average_Calculate1000();
  }
}



/*
void Average_10_Calculate(float value) {
  int array_length = sizeof(avg_10_data) / sizeof(float);
  array_10_index = measure_count % array_length;
  avg_10_data[array_10_index] = value;

  if(measure_count > 0 && measure_count % array_length == 9) {

    float data_sum = 0.0;
    for(int i = 0; i < array_length; i++) {
      data_sum = data_sum + avg_10_data[i];
      avg_10_data[i] = 0;
    }



    avg_10 = data_sum / array_length;
  }
}

void Average_100_Calculate(float value) {
  int array_length = sizeof(avg_100_data) / sizeof(float);
  array_100_index = measure_count % array_length;
  avg_100_data[array_100_index] = value;


  if(measure_count > 10 && measure_count % array_length == 9) {
    float data_sum = 0.0;
    for(int i = 0; i < array_length; i++) {
      data_sum = data_sum + avg_100_data[i];
      avg_100_data[i] = 0;
    }

    avg_100 = data_sum / array_length;
  }
}

void Average_1000_Calculate(float value) {
  int array_length = sizeof(avg_1000_data) / sizeof(float);
  array_1000_index = measure_count % array_length;
  avg_1000_data[array_1000_index] = value;

  if(measure_count > 10 && measure_count % array_length == 9) {
    float data_sum = 0.0;
    for(int i = 0; i < array_length; i++) {
      data_sum = data_sum + avg_1000_data[i];
      avg_1000_data[i] = 0;
    }

    avg_1000 = data_sum / array_length;
  }
}
*/
void Moving_Sample_Sum_Calculate() {
  moving_avg_sample_sum = 0;

  for(int i = 0; i < SAMPLE_SIZE; i++){
    moving_avg_sample_sum += moving_avg_sample[i];
    if(i < SAMPLE_SIZE -1 && moving_avg_sample[i + 1] == 5000) {
      break;
    }
  }
}

void Handle_Moving_Sample_New_Data(float value) {

  if(moving_avg_sample[SAMPLE_SIZE - 1] != 5000){

    for(int i = 1; i < SAMPLE_SIZE; i++) {
      moving_avg_sample[i - 1] = moving_avg_sample[i];
    }
  }
  moving_avg_sample[sample_index] = value;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

    analog_value = HAL_ADC_GetValue(hadc);
    bit_voltage = 3.3 / 4096;
    voltage = bit_voltage * analog_value;
    Handle_Moving_Sample_New_Data(voltage);
    Moving_Sample_Sum_Calculate();
    moving_avg_result_value = moving_avg_sample_sum / (sample_index + 1);


    if(current_time - measure_pre_time > 999) {

      // Average_10_Calculate(moving_avg_result_value);
      // Average_100_Calculate(moving_avg_result_value);
      // Average_1000_Calculate(moving_avg_result_value);
      Value_Allocate(moving_avg_result_value);
      measure_count++;
      measure_pre_time = current_time;
    }


    if(sample_index < SAMPLE_SIZE - 1) {
      sample_index++;
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_MEMORYMAP_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start_IT(&hadc1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    current_time = HAL_GetTick();

    if(current_time - pre_time > 99) {
      HAL_ADC_Start_IT(&hadc1);
      interval_time = current_time - pre_time;
      pre_time = current_time;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief MEMORYMAP Initialization Function
  * @param None
  * @retval None
  */
static void MX_MEMORYMAP_Init(void)
{

  /* USER CODE BEGIN MEMORYMAP_Init 0 */

  /* USER CODE END MEMORYMAP_Init 0 */

  /* USER CODE BEGIN MEMORYMAP_Init 1 */

  /* USER CODE END MEMORYMAP_Init 1 */
  /* USER CODE BEGIN MEMORYMAP_Init 2 */

  /* USER CODE END MEMORYMAP_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
