/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/**************** PWM INPUT **************/
/* define the capturing TIMER's CLOCK and the Prescalar you are using */
#define TIM3_ARR_MAX (uint32_t)0xFFFF

#define TIMCLOCK 170000000
#define PSCALAR 0
#define min(a, b) ({ __typeof__ (a) _a = (a); __typeof__ (b) _b = (b); _a < _b ? _a : _b; })

/* Define the number of samples to be taken by the DMA
   For lower Frequencies, keep the less number for samples
*/
int riseMEANCaptured = 0, riseAMPCaptured = 0, riseAZIMUTHCaptured = 0;
int fallMEANCaptured = 0, fallAMPCaptured = 0, fallAZIMUTHCaptured = 0;
float frequencyMEAN = 0, frequencyAMP = 0, frequencyAZIMUTH = 0;
float widthMEAN = 0, widthAMP = 0, widthAZIMUTH = 0;
uint32_t riseDataMEAN[PWMNUMVAL], riseDataAMP[PWMNUMVAL], riseDataAZIMUTH[PWMNUMVAL];
uint32_t fallDataMEAN[PWMNUMVAL], fallDataAMP[PWMNUMVAL], fallDataAZIMUTH[PWMNUMVAL];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void TIM_IC_CaptureCallback(TIM_TypeDef *tim, const int riseCaptured, const int fallCaptured,
                            uint32_t riseData[], uint32_t fallData[], float *frequency, float *width);
void TimerCaptureCompare_Callback(TIM_TypeDef *tim);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_tim3_ch1;
extern DMA_HandleTypeDef hdma_tim3_ch2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
  if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC1(DMA1);
  }
  else if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TE1(DMA1);
  }
  else if (LL_DMA_IsActiveFlag_HT1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_HT1(DMA1);
  }
  else if (LL_DMA_IsActiveFlag_GI1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_GI1(DMA1);
  }
  /* USER CODE END DMA1_Channel1_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel2 global interrupt.
  */
void DMA1_Channel2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
  if (LL_DMA_IsActiveFlag_TC1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC1(DMA1);
  }
  else if (LL_DMA_IsActiveFlag_TE1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TE1(DMA1);
  }
  else if (LL_DMA_IsActiveFlag_HT1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_HT1(DMA1);
  }
  else if (LL_DMA_IsActiveFlag_GI1(DMA1) == 1)
  {
    LL_DMA_ClearFlag_GI1(DMA1);
  }
  /* USER CODE END DMA1_Channel2_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */

  /* USER CODE END DMA1_Channel2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim3_ch2);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel5 global interrupt.
  */
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */

  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if (LL_TIM_IsActiveFlag_CC2(TIM2) == 1)
  {
    LL_TIM_ClearFlag_CC2(TIM2);
    riseAMPCaptured = 1;
  }

  // If the Interrupt is triggered by 2nd Channel
  if (LL_TIM_IsActiveFlag_CC1(TIM2) == 1)
  {
    LL_TIM_ClearFlag_CC1(TIM2);
    fallAMPCaptured = 1;
  }
 // TimerCaptureCompare_Callback(TIM2);

  TIM_IC_CaptureCallback(TIM2, riseAMPCaptured, fallAMPCaptured,
                         riseDataAMP, fallDataAMP, &frequencyAMP, &widthAMP);
  riseAMPCaptured = 0;
  fallAMPCaptured = 0;

  /* USER CODE END TIM2_IRQn 0 */
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  if (LL_TIM_IsActiveFlag_CC1(TIM3) == 1)
  {
    LL_TIM_ClearFlag_CC1(TIM3);
    riseAZIMUTHCaptured = 1;
  }

  // If the Interrupt is triggered by 2nd Channel
  if (LL_TIM_IsActiveFlag_CC2(TIM3_BASE) == 1)
  {
    LL_TIM_ClearFlag_CC2(TIM3);
    fallAZIMUTHCaptured = 1;
  }

  TIM_IC_CaptureCallback(TIM3, riseAZIMUTHCaptured, fallAZIMUTHCaptured,
                         riseDataAZIMUTH, fallDataAZIMUTH, &frequencyAZIMUTH, &widthAZIMUTH);
  riseAZIMUTHCaptured = 0;
  fallAZIMUTHCaptured = 0;

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM8 capture compare interrupt.
  */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
  if (LL_TIM_IsActiveFlag_CC1(TIM8) == 1)
  {
    LL_TIM_ClearFlag_CC1(TIM8);
    riseMEANCaptured = 1;
  }

  // If the Interrupt is triggered by 2nd Channel
  if (LL_TIM_IsActiveFlag_CC2(TIM8) == 1)
  {
    LL_TIM_ClearFlag_CC2(TIM8);
    fallMEANCaptured = 1;
  }

  TIM_IC_CaptureCallback(TIM8, riseMEANCaptured, fallMEANCaptured,
                         riseDataMEAN, fallDataMEAN, &frequencyMEAN, &widthMEAN);
  riseMEANCaptured = 0;
  fallMEANCaptured = 0;

  /* USER CODE END TIM8_CC_IRQn 0 */
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
  * @brief This function handles DMAMUX overrun interrupt.
  */
void DMAMUX_OVR_IRQHandler(void)
{
  /* USER CODE BEGIN DMAMUX_OVR_IRQn 0 */

  /* USER CODE END DMAMUX_OVR_IRQn 0 */

  /* USER CODE BEGIN DMAMUX_OVR_IRQn 1 */

  /* USER CODE END DMAMUX_OVR_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void TIM_IC_CaptureCallback(TIM_TypeDef *tim, const int riseCaptured, const int fallCaptured,
                            uint32_t riseData[], uint32_t fallData[], float *frequency, float *width)
{
  /* Rest of the calculations will be done,
   * once both the DMAs have finished capturing enough data */
  if ((riseCaptured) && (fallCaptured))
  {

    // calculate the reference clock
    float refClock = TIMCLOCK / (PSCALAR + 1);

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
    while ((min((riseData[indxr + 1] - riseData[indxr]), (riseData[indxr + 2] - riseData[indxr + 1]))) == 0)
      indxr++;

    /* riseavg is the difference in the 2 consecutive rise Time */

    /* Assign a start value to riseavg */
    riseavg += min((riseData[indxr + 1] - riseData[indxr]), (riseData[indxr + 2] - riseData[indxr + 1]));
    indxr++;
    countr++;

    /* start adding the values to the riseavg */
    while (indxr < (PWMNUMVAL))
    {
      riseavg += min((riseData[indxr + 1] - riseData[indxr]), riseavg / countr);
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
        rfavg += min((fallData[indxf] - riseData[indxr]), (fallData[indxf + 1] - riseData[indxr + 1]));
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
    *frequency = (refClock / (float)riseavg);
    ;

    /* Width of the pulse
     *  = (Time between Rise and fall) / clock
     */
    *width = ((rfavg) / ((float)(refClock / 1000000))) * 1000; // width in ns
  }
}

void TimerCaptureCompare_Callback(TIM_TypeDef *tim)
{
  /* Capture index */
  static uint16_t uhCaptureIndex = 0;

  /* Captured Values */
  static uint32_t uwICValue1 = 0;
  static uint32_t uwICValue2 = 0;
  static uint32_t uwDiffCapture = 0;

  uint32_t TIM3CLK;
  uint32_t PSC;
  uint32_t IC1PSC;
  uint32_t IC1Polarity;

  if (uhCaptureIndex == 0)
  {
    /* Get the 1st Input Capture value */
    uwICValue1 = LL_TIM_IC_GetCaptureCH1(tim);
    uhCaptureIndex = 1;
  }
  else if (uhCaptureIndex == 1)
  {
    /* Get the 2nd Input Capture value */
    uwICValue2 = LL_TIM_IC_GetCaptureCH1(tim);

    /* Capture computation */
    if (uwICValue2 > uwICValue1)
    {
      uwDiffCapture = (uwICValue2 - uwICValue1);
    }
    else if (uwICValue2 < uwICValue1)
    {
      uwDiffCapture = ((TIM3_ARR_MAX - uwICValue1) + uwICValue2) + 1;
    }
    else
    {
      /* If capture values are equal, we have reached the limit of frequency  */
      /* measures.                                                            */
      //LED_Blinking(LED_BLINK_ERROR);
    }

    /* The signal frequency is calculated as follows:                         */
    /* Frequency = (TIM3*IC1PSC) / (Capture*(PSC+1)*IC1Polarity)           */
    /* where:                                                                 */
    /*  Capture is the difference between two consecutive captures            */
    /*  TIM3CLK is the timer counter clock frequency                           */
    /*  PSC is the timer prescaler value                                      */
    /*  IC1PSC is the input capture prescaler value                           */
    /*  IC1Polarity value depends on the capture sensitivity:                 */
    /*    1 if the input is sensitive to rising or falling edges              */
    /*    2 if the input is sensitive to both rising and falling edges        */

    /* Retrieve actual TIM3 counter clock frequency */
    TIM3CLK = SystemCoreClock;

    /* Retrieve actual TIM3 prescaler value */
    PSC = LL_TIM_GetPrescaler(tim);

    /* Retrieve actual IC1 prescaler ratio */
    IC1PSC = __LL_TIM_GET_ICPSC_RATIO(LL_TIM_IC_GetPrescaler(tim, LL_TIM_CHANNEL_CH1));

    /* Retrieve actual IC1 polarity setting */
    if (LL_TIM_IC_GetPolarity(tim, LL_TIM_CHANNEL_CH1) == LL_TIM_IC_POLARITY_BOTHEDGE)
      IC1Polarity = 2;
    else
      IC1Polarity = 1;

    /* Calculate input signal frequency */
    float uwMeasuredFrequency = (float)(TIM3CLK * IC1PSC) / (float)(uwDiffCapture * (PSC + 1) * IC1Polarity);

    /* reset capture index */
    uhCaptureIndex = 0;
  }
}
/* USER CODE END 1 */
