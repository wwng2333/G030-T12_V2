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
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EventRecorder.h"
#include "oled_driver.h"
#include "stdio.h"
#include "i2c.h"
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

/* USER CODE BEGIN PV */
static u8g2_t u8g2;
uint16_t t12_adc;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void beep();
void MainScreen(u8g2_t *u8g2);
void T12_ADC_Read(void);
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */
	EventRecorderInitialize(EventRecordAll, 1U);
	EventRecorderStart();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	printf("CPU @ %d Hz\n", SystemCoreClock);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	Activate_ADC();
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	
//	LL_TIM_EnableAllOutputs(TIM3);
//	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
//	LL_TIM_EnableCounter(TIM3);

	LL_ADC_EnableIT_EOC(ADC1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		MainScreen(&u8g2);
		//beep();
		T12_ADC_Read();
		LL_mDelay(10);
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
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  }

  /* HSI configuration and activation */
  LL_RCC_HSI_Enable();
  while(LL_RCC_HSI_IsReady() != 1)
  {
  }

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8, LL_RCC_PLLR_DIV_2);
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while(LL_RCC_PLL_IsReady() != 1)
  {
  }

  /* Set AHB prescaler*/
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

  /* Sysclk activation on the main PLL */
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  }

  /* Set APB1 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

  LL_Init1msTick(64000000);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(64000000);
}

/* USER CODE BEGIN 4 */
void TIM17_Delay_100us(void)
{
	LL_TIM_EnableCounter(TIM17);
	while(LL_TIM_IsActiveFlag_UPDATE(TIM17) == RESET)
		;
	LL_TIM_DisableCounter(TIM17);
	LL_TIM_ClearFlag_UPDATE(TIM17);
}

void beep()
{
	for(uint8_t i=0; i<255; i++)
	{
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
		TIM17_Delay_100us();
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
		TIM17_Delay_100us();
	}
}

void MainScreen(u8g2_t *u8g2)
{
  char sprintf_tmp[8] = {0};
  u8g2_FirstPage(u8g2);
  u8g2_SetFontMode(u8g2, 1);
  u8g2_SetFontDirection(u8g2, 0);
  do
  {
    u8g2_SetFont(u8g2, u8g2_font_9x15_te);
    u8g2_DrawStr(u8g2, 0, 10, "SET:");
    u8g2_DrawStr(u8g2, 40, 10, "200");
    u8g2_DrawStr(u8g2, 83, 10, "  OFF");
    u8g2_DrawStr(u8g2, 0, 62, "T12-KU");
    sprintf(sprintf_tmp, "%.1fC", TMP75_ReadTemp());
    u8g2_DrawStr(u8g2, 83, 62, sprintf_tmp);

    u8g2_SetFont(u8g2, u8g2_font_freedoomr25_mn);
    //sprintf(sprintf_tmp, "%d", encoder_value);
    u8g2_DrawStr(u8g2, 37, 45, sprintf_tmp);
  } while (u8g2_NextPage(u8g2));
}

void T12_ADC_Read(void)
{
	LL_ADC_REG_StartConversion(ADC1);
	while(LL_ADC_IsActiveFlag_EOC(ADC1) == RESET)
	{
		;
	}
	t12_adc = __LL_ADC_CALC_DATA_TO_VOLTAGE(3300, LL_ADC_REG_ReadConversionData12(ADC1), LL_ADC_RESOLUTION_12B);
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
