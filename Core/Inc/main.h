/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

#include "stm32g0xx_ll_adc.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_system.h"
#include "stm32g0xx_ll_exti.h"
#include "stm32g0xx_ll_cortex.h"
#include "stm32g0xx_ll_utils.h"
#include "stm32g0xx_ll_pwr.h"
#include "stm32g0xx_ll_dma.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// Firmware version
#define VERSION       "v1.9"

// Type of MOSFET
#define N_MOSFET                // P_MOSFET or N_MOSFET

// Type of OLED Controller
#define SSD1306                 // SSD1306 or SH1106

// Type of rotary encoder
#define ROTARY_TYPE   0         // 0: 2 increments/step; 1: 4 increments/step (default)

// Default temperature control values (recommended soldering temperature: 300-380 C)
#define TEMP_MIN      150       // min selectable temperature
#define TEMP_MAX      400       // max selectable temperature
#define TEMP_DEFAULT  200       // default start setpoint
#define TEMP_SLEEP    150       // temperature in sleep mode
#define TEMP_BOOST     50       // temperature increase in boost mode
#define TEMP_STEP      10       // rotary encoder temp change steps

// Default tip temperature calibration values
#define TEMP1212       216       // temperature at ADC = 200 (5V, 10bit ADC)
#define TEMP1696       308       // temperature at ADC = 280
#define TEMP2181       390       // temperature at ADC = 360
#define TEMPCHP       30        // chip temperature while calibration
#define TIPMAX        8         // max number of tips
#define TIPNAMELENGTH 6         // max length of tip names (including termination)
#define TIPNAME       "T12-KU"   // default tip name

// Default timer values (0 = disabled)
#define TIME2SLEEP     5        // time to enter sleep mode in minutes
#define TIME2OFF      15        // time to shut off heater in minutes
#define TIMEOFBOOST   40        // time to stay in boost mode in seconds

// Control values
#define TIME2SETTLE   950       // time in microseconds to allow OpAmp output to settle
#define SMOOTHIE      0.05      // OpAmp output smooth factor (1=no smoothing; 0.05 default)
#define PID_ENABLE    true     // enable PID control
#define BEEP_ENABLE   true      // enable/disable buzzer
#define BODYFLIP      false     // enable/disable screen flip
#define ECREVERSE     false     // enable/disable rotary encoder reverse
#define MAINSCREEN    1         // type of main screen (0: big numbers; 1: more infos)

#define EEPROM_IDENT  0xE76C   // to identify if EEPROM was written by this program

typedef struct
{
	uint16_t identifier;
	uint16_t DefaultTemp;
	uint16_t SleepTemp;
	uint8_t BoostTemp;
	uint8_t time2sleep;
} SystemParam_A;

typedef struct
{
	uint16_t identifier;
	uint8_t time2off;
	uint8_t timeOfBoost;
	uint8_t SleepTemp;
	union {
		struct {
			unsigned int MainScrType : 1;
			unsigned int beepEnable : 1;
			unsigned int BodyFlip : 1;
			unsigned int ECReverse : 1;
		};
	uint8_t all;
	};
} SystemParam_B;

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int constrain(int x, int min, int max);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PB9_SDA_Pin LL_GPIO_PIN_9
#define PB9_SDA_GPIO_Port GPIOB
#define PB8_SCL_Pin LL_GPIO_PIN_8
#define PB8_SCL_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
