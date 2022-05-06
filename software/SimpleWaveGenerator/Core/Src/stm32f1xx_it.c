/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f1xx_it.h"
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
extern unsigned short ADC_Value1;
unsigned short smooth1[2] = {0,0};
extern ADC_HandleTypeDef hadc1;

extern unsigned short ADC_Value2;
unsigned short smooth2[2] = {0,0};
extern ADC_HandleTypeDef hadc2;

extern float ADC_Value1_temp;
extern float ADC_Value2_temp;

extern unsigned short frequency_step;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
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
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */
	
	/* ------Calculate the ADC_Value1------ */
	float temp = 0;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,50);
	smooth1[0] = HAL_ADC_GetValue(&hadc1);
	
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,50);
	smooth1[1] = HAL_ADC_GetValue(&hadc1);
	
	ADC_Value1 = (smooth1[0] + smooth2[1])/2;
	if( ADC_Value1<360)//40
	{
		ADC_Value1 = 2350;
		frequency_step = 3;
	}
	else if(ADC_Value1 >= 360&& ADC_Value1 <=370)//43
	{
		ADC_Value1 = 2200;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 370&& ADC_Value1 <=380)//46
	{
		ADC_Value1 = 2050;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 380&& ADC_Value1 <=390)//49
	{
		ADC_Value1 = 1920;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 390&& ADC_Value1 <=400)//52
	{
		ADC_Value1 = 1780;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 400&& ADC_Value1 <=410)//55
	{
		ADC_Value1 = 1680;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 410&& ADC_Value1 <=420)//58
	{
		ADC_Value1 = 1580;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 410&& ADC_Value1 <=420)//61
	{
		ADC_Value1 = 1540;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 420&& ADC_Value1 <=430)//64
	{
		ADC_Value1 = 1490;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 430&& ADC_Value1 <=440)//67
	{
		ADC_Value1 = 1410;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 440&& ADC_Value1 <=450)//70
	{
		ADC_Value1 = 1340;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 450&& ADC_Value1 <=460)//73
	{
		ADC_Value1 = 1295;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 460&& ADC_Value1 <=470)//76
	{
		ADC_Value1 = 1240;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 470&& ADC_Value1 <=480)//79
	{
		ADC_Value1 = 1195;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 480&& ADC_Value1 <=488)//82
	{
		ADC_Value1 = 1150;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 488&& ADC_Value1 <=498)//85
	{
		ADC_Value1 = 1105;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 498&& ADC_Value1 <=508)//88
	{
		ADC_Value1 = 1070;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 508&& ADC_Value1 <=517)//91
	{
		ADC_Value1 = 1030;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 517&& ADC_Value1 <=527)//94
	{
		ADC_Value1 = 1000;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 527&& ADC_Value1 <=545)//97
	{
		ADC_Value1 = 960;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 545&& ADC_Value1 <=1105)//100-270
	{
		// ADC_Value1 = 165;
		// ADC_Value1 = 329;
		// ADC_Value1 = 930;
		ADC_Value1 = 1525 - ADC_Value1*1.09;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 1105)//270-500
	{
		// ADC_Value1 = 165;
		// ADC_Value1 = 329;
		// ADC_Value1 = 930;
		ADC_Value1 = 570 - ADC_Value1*0.221;
		frequency_step = 3;
	}
	else if(ADC_Value1 > 1835)//270-500
	{
		ADC_Value1 = 165;
		// ADC_Value1 = 329;
		// ADC_Value1 = 930;
//		ADC_Value1 = 569 - ADC_Value1*0.221;
//		frequency_step = 3;
	}

//	else if(ADC_Value1 > 555&& ADC_Value1 <=557)//103
//	{
//		ADC_Value1 = 900;
//		frequency_step = 3;
//	}
//	else if(ADC_Value1 > 550 && ADC_Value1 <= 1860)//xxx
//	{
//		ADC_Value1 = ADC_Value1;
//		frequency_step = 3;
//	}
	
//	if (ADC_Value1<=360)
//	{
//		ADC_Value1 = 2350;
//	}
//	else if(ADC_Value1>360 && ADC_Value1<=700)
//	{
//		ADC_Value1 = ADC_Value1 - 360;
//		ADC_Value1 = ADC_Value1 * 5.9;
//		ADC_Value1 = 2350 - ADC_Value1;
//		frequency_step = 3;
//	}
//	else if( ADC_Value1>700 && ADC_Value1<=1100)
//	{
//			ADC_Value1 = ADC_Value1 - 700;
//			ADC_Value1 = ADC_Value1 / 1.5;
//		  ADC_Value1 = 510 - ADC_Value1;
//			frequency_step = 4;
//	}
//	else if(ADC_Value1>1100 && ADC_Value1 <1120)
//	{
//		ADC_Value1 = 350;
//		frequency_step = 6;
//	}
//	else if( ADC_Value1>=1120)
//	{
////		ADC_Value1 = ADC_Value1;
//		ADC_Value1 = ADC_Value1-1110;
//		ADC_Value1 = ADC_Value1/4;
//		ADC_Value1 = 345-ADC_Value1;
//		frequency_step = 6;
//	}
	
//	ADC_Value1 = ADC_Value1 - 742;
//	ADC_Value1 = 3000 - ADC_Value1;
	
	/*--------------------------------------*/

	/* ------Calculate the ADC_Value2------ */
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2,50);
	ADC_Value2 = HAL_ADC_GetValue(&hadc2);
	
	/*limit the range*/
	ADC_Value2 = ADC_Value2 - 729;
	temp = (float)ADC_Value2;
	temp = temp/1.4665;
	ADC_Value2 = (unsigned short)temp;
//	temp = (float)ADC_Value2/1270;
//	ADC_Value2 = temp * 2000;
	/*--------------------------------------*/


  /* USER CODE END TIM1_UP_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
