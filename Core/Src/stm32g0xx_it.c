/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "timer.h"
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
extern uint16_t SetTemp;
extern uint32_t TIM16_Tick;
extern volatile int count;
extern volatile int countStep, countMin, countMax;
extern volatile bool handleMoved;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
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
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
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

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 4 to 15 interrupts.
  */
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
	static uint8_t _count = 0;
	static uint8_t b_flag;
	uint8_t a_value = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_14);
	uint8_t b_value = LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_15);
	LL_EXTI_DisableIT_0_31(LL_EXTI_LINE_14);
	if(a_value == RESET && _count == 0)
	{
		b_flag = 0;
		if(b_value)
			b_flag = 1;
		_count = 1;
	}
	
	if(a_value == SET && _count == 1)
	{
		if(b_value == RESET && b_flag == 1)
		{
			//printf("+");
			count = constrain(count + ((a_value == b_value) ? -countStep : countStep), countMin, countMax);
			handleMoved = true;
		}
		else if(b_value == SET && b_flag == 0)
		{
			//printf("-");
			count = constrain(count + ((a_value == b_value) ? -countStep : countStep), countMin, countMax);
			handleMoved = true;
		}
		_count = 0;
	}
	LL_EXTI_EnableIT_0_31(LL_EXTI_LINE_14);
  /* USER CODE END EXTI4_15_IRQn 0 */
  if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_10) != RESET)
  {
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_10);
    /* USER CODE BEGIN LL_EXTI_LINE_10_FALLING */
		printf("handle moved!\n");
    /* USER CODE END LL_EXTI_LINE_10_FALLING */
  }
  if (LL_EXTI_IsActiveFallingFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearFallingFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14_FALLING */

    /* USER CODE END LL_EXTI_LINE_14_FALLING */
  }
  if (LL_EXTI_IsActiveRisingFlag_0_31(LL_EXTI_LINE_14) != RESET)
  {
    LL_EXTI_ClearRisingFlag_0_31(LL_EXTI_LINE_14);
    /* USER CODE BEGIN LL_EXTI_LINE_14_RISING */
		
    /* USER CODE END LL_EXTI_LINE_14_RISING */
  }
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */
	if(LL_TIM_IsActiveFlag_CC1(TIM16))
	{
		TIM16_Tick++;
		timer_sched();
		LL_TIM_ClearFlag_CC1(TIM16);
	}
  /* USER CODE END TIM16_IRQn 0 */
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
