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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;
DMA_HandleTypeDef hdma_tim2_ch1;
DMA_HandleTypeDef hdma_tim2_ch2;
DMA_HandleTypeDef hdma_tim8_ch1;
DMA_HandleTypeDef hdma_tim8_ch2;

/* USER CODE BEGIN PV */
/**************** PWM INPUT **************/

/* define the capturing TIMER's CLOCK and the Prescalar you are using */
#define TIMCLOCK 170000000
#define PSCALARAMP 16
#define PSCALARMEAN 29
#define PSCALARAZIMUTH 29
/* Define the number of samples to be taken by the DMA
   For lower Frequencies, keep the less number for samples
*/
#define MIN(a, b) ((a) < (b) ? (a) : (b))

int riseMEANCaptured = 0, riseAMPCaptured = 0, riseAZIMUTHCaptured = 0;
int fallMEANCaptured = 0, fallAMPCaptured = 0, fallAZIMUTHCaptured = 0;
float frequencyMEAN = 0, frequencyAMP = 0, frequencyAZIMUTH = 0;
float widthMEAN = 0, widthAMP = 0, widthAZIMUTH = 0;
uint32_t riseDataAMP[PWMNUMVAL], fallDataAMP[PWMNUMVAL];
uint32_t riseDatatemp[PWMNUMVAL], fallDatatemp[PWMNUMVAL];
uint16_t riseDataMEAN[PWMNUMVAL], riseDataAZIMUTH[PWMNUMVAL];
uint16_t fallDataMEAN[PWMNUMVAL], fallDataAZIMUTH[PWMNUMVAL];
int isMeasuredAMP = 0, isMeasuredMEAN = 0, isMeasuredAZIMUTH = 0;

void TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim, const int pscalar, int *riseCaptured, int *fallCaptured, int *isMeasured,
                            uint32_t *riseData, uint32_t *fallData, float *frequency, float *width);

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      riseAMPCaptured = 1;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      fallAMPCaptured = 1;
    }
    TIM_IC_CaptureCallback(htim, PSCALARAMP, &riseAMPCaptured, &fallAMPCaptured, &isMeasuredAMP,
                           riseDataAMP, fallDataAMP, &frequencyAMP, &widthAMP);
  }
  if (htim->Instance == TIM8)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
      riseMEANCaptured = 1;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
      fallMEANCaptured = 1;
    }
    for (int i=0;i<PWMNUMVAL;i++)
    {
      riseDatatemp[i]=riseDataMEAN[i];
      fallDatatemp[i]=fallDataMEAN[i];
    }  
    TIM_IC_CaptureCallback(htim, PSCALARMEAN, &riseMEANCaptured, &fallMEANCaptured, &isMeasuredMEAN,
                           riseDatatemp, fallDatatemp, &frequencyMEAN, &widthMEAN);
  }
  if (htim->Instance == TIM3)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
    {
      riseAZIMUTHCaptured = 1;
    }
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
    {
      fallAZIMUTHCaptured = 1;
    }
    for (int i=0;i<PWMNUMVAL;i++)
    {
      riseDatatemp[i]=riseDataAZIMUTH[i];
      fallDatatemp[i]=fallDataAZIMUTH[i];
    }  
    TIM_IC_CaptureCallback(htim, PSCALARAZIMUTH, &riseAZIMUTHCaptured, &fallAZIMUTHCaptured, &isMeasuredAZIMUTH,
                           riseDatatemp, fallDatatemp, &frequencyAZIMUTH, &widthAZIMUTH);
  }
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* The PWM Output from Timer 1 */

  /* TIM2 Channel 1 is set to rising edge, so it will store the data in 'riseData' */
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, riseDataAMP, PWMNUMVAL);
  /* TIM2 Channel 2 is set to falling edge, so it will store the data in 'fallData' */
  HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, fallDataAMP, PWMNUMVAL);

  /* TIM8 Channel 1 is set to rising edge, so it will store the data in 'riseData' */
  HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, riseDataMEAN, PWMNUMVAL);
  /* TIM8 Channel 2 is set to falling edge, so it will store the data in 'fallData' */
  HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_2, fallDataMEAN, PWMNUMVAL);

  /* TIM8 Channel 1 is set to rising edge, so it will store the data in 'riseData' */
  HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_3, riseDataAZIMUTH, PWMNUMVAL);
  /* TIM8 Channel 2 is set to falling edge, so it will store the data in 'fallData' */
  HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_4, fallDataAZIMUTH, PWMNUMVAL);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Call the measurement whenever needed */
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_1, riseDataAMP, PWMNUMVAL);
    HAL_TIM_IC_Start_DMA(&htim2, TIM_CHANNEL_2, fallDataAMP, PWMNUMVAL);
    HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_1, riseDataMEAN, PWMNUMVAL);
    HAL_TIM_IC_Start_DMA(&htim8, TIM_CHANNEL_2, fallDataMEAN, PWMNUMVAL);
    HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_3, riseDataAZIMUTH, PWMNUMVAL);
    HAL_TIM_IC_Start_DMA(&htim3, TIM_CHANNEL_4, fallDataAZIMUTH, PWMNUMVAL);
    HAL_Delay(1000);
    if (isMeasuredAMP)
    {
      TIM2->CNT = 0;
      isMeasuredAMP = 0;
    }
    if (isMeasuredMEAN)
    {
      TIM8->CNT = 0;
      isMeasuredMEAN = 0;
    }
    if (isMeasuredAZIMUTH)
    {
      TIM3->CNT = 0;
      isMeasuredAZIMUTH = 0;
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 17-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4.294967295E9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 30-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOF);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim, const int pscalar, int *riseCaptured, int *fallCaptured, int *isMeasured,
                            uint32_t *riseData, uint32_t *fallData, float *frequency, float *width)
{
  /* Rest of the calculations will be done,
   * once both the DMAs have finished capturing enough data */
  if (!((*riseCaptured) && (*fallCaptured)))
    return;

  // calculate the reference clock
  float refClock = TIMCLOCK / (pscalar + 1);

  int indxr = 0;
  int indxf = 0;

  int countr = 0;
  int countrf = 0;

  float riseavg = 0;
  float rfavg = 0;

  /* In case of high Frequencies, the DMA sometimes captures 0's in the beginning.
   * increment the index until some useful data shows up
   */
  while (riseData[indxr] == 0)
    indxr++;

  /* Again at very high frequencies, sometimes the values don't change
   * So we will wait for the update among the values
   */
  while ((MIN((riseData[indxr + 1] - riseData[indxr]), (riseData[indxr + 2] - riseData[indxr + 1]))) == 0)
    indxr++;

  /* riseavg is the difference in the 2 consecutive rise Time */

  /* Assign a start value to riseavg */
  riseavg += MIN((riseData[indxr + 1] - riseData[indxr]), (riseData[indxr + 2] - riseData[indxr + 1]));
  indxr++;
  countr++;

  /* start adding the values to the riseavg */
  while (indxr < (PWMNUMVAL))
  {
    riseavg += MIN((riseData[indxr + 1] - riseData[indxr]), riseavg / countr);
    countr++;
    indxr++;
  }

  /* Find the average riseavg, the average time between 2 RISE */
  riseavg = riseavg / countr;

  indxr = 0;

  /* The calculation for the Falling pulse on second channel */

  /* If the fall time is lower than rise time,
   * Then there must be some error and we will increment
   * both, until the error is gone
   */
  if (fallData[indxf] < riseData[indxr])
  {
    indxf += 2;
    indxr += 2;
    while (fallData[indxf] < riseData[indxr])
      indxf++;
  }

  else if (fallData[indxf] > riseData[indxr])
  {
    indxf += 2;
    indxr += 2;
    while (fallData[indxf] > riseData[indxr + 1])
      indxr++;
  }

  /* The method used for the calculation below is as follows:
   * If Fall time < Rise Time, increment Fall counter
   * If Fall time - Rise Time is in between 0 and (difference between 2 Rise times), then its a success
   * If fall time > Rise time, but is also > (difference between 2 Rise times), then increment Rise Counter
   */
  while ((indxf < (PWMNUMVAL)) && (indxr < (PWMNUMVAL)))
  {
    /* If the Fall time is lower than rise time, increment the fall indx */
    while ((int16_t)(fallData[indxf] - riseData[indxr]) < 0)
    {
      indxf++;
    }

    /* If the Difference in fall time and rise time is >0 and less than rise average,
     * Then we will register it as a success and increment the countrf (the number of successes)
     */
    if (((int16_t)(fallData[indxf] - riseData[indxr]) >= 0) && (((int16_t)(fallData[indxf] - riseData[indxr]) <= riseavg)))
    {
      rfavg += MIN((fallData[indxf] - riseData[indxr]), (fallData[indxf + 1] - riseData[indxr + 1]));
      indxf++;
      indxr++;
      countrf++;
    }

    else
    {
      indxr++;
    }
  }

  /* Calculate the Average time between 2 Rise */
  rfavg = rfavg / countrf;

  /* Calculate Frequency
   * Freq = Clock/(time taken between 2 Rise)
   */
  *frequency = (refClock / (float)(riseavg+1));

  /* Width of the pulse
   *  = (Time between Rise and fall) / clock
   */
  *width = (rfavg) / (float)(riseavg+1); // width in ns

  *riseCaptured = 0;
  *fallCaptured = 0;

  *isMeasured = 1;
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
