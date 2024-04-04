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
#include "PID.h"
#include "math.h"
#include "Flash.h"
//#include "stm32_button.h"
#include "timer.h"
#include <string.h>
#include "mfbd.h"
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
uint16_t temp_adc;
uint8_t PWM_Output;

PID_TypeDef TPID;

SystemParamStore SystemParam = {0};

// Define the aggressive and conservative PID tuning parameters
double aggKp=11, aggKi=0.5, aggKd=1;
double consKp=66, consKi=0.5, consKd=1;

// Default values that can be changed by the user and stored in the EEPROM
uint16_t  DefaultTemp = TEMP_DEFAULT;
uint16_t  SleepTemp   = TEMP_SLEEP;
uint8_t   BoostTemp   = TEMP_BOOST;
uint8_t   time2sleep  = TIME2SLEEP;
uint8_t   time2off    = TIME2OFF;
uint8_t   timeOfBoost = TIMEOFBOOST;
uint8_t   MainScrType = MAINSCREEN;
bool      PIDenable   = PID_ENABLE;
bool      beepEnable  = BEEP_ENABLE;
bool      BodyFlip    = BODYFLIP;
bool      ECReverse   = ECREVERSE;

// Default values for tips
uint16_t  CalTemp[TIPMAX][4] = {TEMP1212, TEMP1696, TEMP2181, TEMPCHP};
char      TipName[TIPMAX][TIPNAMELENGTH] = {TIPNAME};
uint8_t   CurrentTip   = 0;
uint8_t   NumberOfTips = 1;

// Menu items
const char *SetupItems[]       = { "Setup Menu", "Tip Settings", "Temp Settings",
                                   "Timer Settings", "Control Type", "Main Screen",
                                   "Buzzer", "Screen Flip", "EC Reverse", "Information", "Return" };
const char *TipItems[]         = { "Tip:", "Change Tip", "Calibrate Tip", 
                                   "Rename Tip", "Delete Tip", "Add new Tip", "Return" };
const char *TempItems[]        = { "Temp Settings", "Default Temp", "Sleep Temp", 
                                   "Boost Temp", "Return" };
const char *TimerItems[]       = { "Timer Settings", "Sleep Timer", "Off Timer", 
                                   "Boost Timer", "Return" };
const char *ControlTypeItems[] = { "Control Type", "Direct", "PID" };
const char *MainScreenItems[]  = { "Main Screen", "Big Numbers", "More Infos" };
const char *StoreItems[]       = { "Store Settings ?", "No", "Yes" };
const char *SureItems[]        = { "Are you sure ?", "No", "Yes" };
const char *BuzzerItems[]      = { "Buzzer", "Disable", "Enable" };
const char *FlipItems[]        = { "Screen Flip", "Disable", "Enable" };
const char *ECReverseItems[]   = { "EC Reverse", "Disable", "Enable" };
const char *DefaultTempItems[] = { "Default Temp", "\xB0""C" };
const char *SleepTempItems[]   = { "Sleep Temp", "\xB0""C" };
const char *BoostTempItems[]   = { "Boost Temp", "\xB0""C" };
const char *SleepTimerItems[]  = { "Sleep Timer", "Minutes" };
const char *OffTimerItems[]    = { "Off Timer", "Minutes" };
const char *BoostTimerItems[]  = { "Boost Timer", "Seconds" };
const char *DeleteMessage[]    = { "Warning", "You cannot", "delete your", "last tip!" };
const char *MaxTipMessage[]    = { "Warning", "You reached", "maximum number", "of tips!" };

// Variables for pin change interrupt
volatile uint8_t  a0, b0, c0, d0;
volatile bool     ab0;
volatile int      count, countMin, countMax, countStep;
volatile bool     handleMoved;
 
// Variables for temperature control
uint16_t  SetTemp, ShowTemp, gap, Step;
double    Input, Output, Setpoint, RawTemp, CurrentTemp, ChipTemp;
// Variables for voltage readings
uint16_t  Vcc, Vin;
 
// State variables
bool      inSleepMode = false;
bool      inOffMode   = false;
bool      inBoostMode = false;
bool      inCalibMode = false;
bool      isWorky     = true;
bool      beepIfWorky = true;
bool      TipIsPresent= true;

// Timing variables
uint32_t  sleepmillis;
uint32_t  boostmillis;
uint32_t  buttonmillis;
uint8_t   goneMinutes;
uint8_t   goneSeconds;
uint8_t   SensorCounter = 255;

__IO uint32_t TIM16_Tick = 0;

uint8_t u8g2_update_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void MainScreen(u8g2_t *u8g2);
void RawTemp_Read(void);
void Vin_Read(void);
void Vref_Read(void);
void Temp_ADC_Read(void);
uint16_t denoiseAnalog(uint32_t adc_ch);
void calculateTemp(void);
void ROTARYCheck(void);
void SENSORCheck(void);
void Thermostat(void);
void TimerScreen(void);
void SetupScreen(void);
void TipScreen(void);
void InputNameScreen(void);
void AddTipScreen(void);
void DeleteTipScreen(void);
void ChangeTipScreen(void);
void CalibrationScreen(void);
void MessageScreen(const char *Items[], uint8_t numberOfItems);
void InfoScreen(void);
void TempScreen(void);
uint16_t InputScreen(const char *Items[]);
uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected);
void beep(void);
void setRotary(int rmin, int rmax, int rstep, int rvalue);
uint16_t getRotary(void);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void Read_System_Parmeter(void);
uint32_t get_sys_tick(void);
void t1_1s_cb(void* para);
void t2_300ms_cb(void* para);
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
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	Activate_ADC();
	u8g2_Setup_ssd1306_128x64_noname_f(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	
	PID(&TPID, &Input, &Output, &Setpoint, aggKp, aggKi, aggKd, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&TPID, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&TPID, -1);
	PID_SetOutputLimits(&TPID, 0, 1999);
	
	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH1(TIM3, 0);
	LL_TIM_EnableCounter(TIM3);

	LL_TIM_EnableAllOutputs(TIM14); //Enable TIM for beep
	LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
	LL_TIM_DisableCounter(TIM14); //Disable now, beep later.
	
	LL_TIM_EnableAllOutputs(TIM16); //Enable TIM for tick
	LL_TIM_EnableIT_CC1(TIM16); //Enable TIM16 Interrupt
	LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1N);
	LL_TIM_EnableCounter(TIM16);
	timer_init();
	timer_id t1 = timer_creat(t1_1s_cb, 0, 1000, true, NULL);
	timer_id t2 = timer_creat(t2_300ms_cb, 0, 300, true, NULL);
	timer_sched();
	//Test flash
//	uint16_t len;
//	len = XMEM_ALIGN_SIZE(sizeof(SystemParamStore), 8);
//	if(len > (FLASH_PAGE_SIZE >> 3))
//	{
//		printf("SystemParam size over flash page size...\n");
//	}
//	else
//	{
//		if(ubFlash_Write_DoubleWord(SYSTEM_ARG_STORE_START_ADDR, (uint64_t *)&SystemParam, len) != 0x00U)
//		{
//			printf("Save param failed!\n");
//		}
//		else
//		{
//			
//			printf("Save param OK!\n");
//		}
//	}
	uint32_t test;
	test = *((uint32_t *)SYSTEM_ARG_STORE_START_ADDR);
	printf("test: %u, ", test);
	memset(&SystemParam, 0, sizeof(SystemParam));
	memcpy(&SystemParam, (const void*)SYSTEM_ARG_STORE_START_ADDR, sizeof(SystemParam));
	//Read_System_Parmeter();
	
	// read supply voltages in mV
	Vref_Read();
	Vin_Read();
	
  // read and set current iron temperature
  SetTemp = DefaultTemp;
  RawTemp_Read();
  ChipTemp = TMP75_ReadTemp();
  calculateTemp();
	
	setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
	
	beep(); beep();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		ROTARYCheck();
		//printf("%d->", get_sys_tick());
		SENSORCheck();
		//printf("%d\n", get_sys_tick());
		Thermostat();
		MainScreen(&u8g2);
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
void MainScreen(u8g2_t *u8g2)
{
  char sprintf_tmp[8] = {0};
  u8g2_FirstPage(u8g2);
  u8g2_SetFontMode(u8g2, 1);
  u8g2_SetFontDirection(u8g2, 0);
  do
  {
    u8g2_SetFont(u8g2, u8g2_font_9x15_tr);
    u8g2_DrawStr(u8g2, 0, 10, "SET:");
    sprintf(sprintf_tmp, "%hu", (uint16_t)Setpoint);
    u8g2_DrawStr(u8g2, 40, 10, sprintf_tmp);
		
		if(ShowTemp > 500) strcpy(sprintf_tmp, "ERROR");
		else if(inOffMode) strcpy(sprintf_tmp, "  OFF");
		else if(inSleepMode) strcpy(sprintf_tmp, "SLEEP");
		else if(inBoostMode) strcpy(sprintf_tmp, "BOOST");
		else if(isWorky) strcpy(sprintf_tmp, "WORKY");
		else if(Output < 180) strcpy(sprintf_tmp, " HEAT");
		else strcpy(sprintf_tmp, " HOLD");
		u8g2_DrawStr(u8g2, 83, 10, sprintf_tmp);
		
		if(MainScrType)
		{
			PWM_Output = Output / 1999 * 100;
			sprintf(sprintf_tmp, " %d%%", PWM_Output);
			u8g2_DrawStr(u8g2, 85, 32, sprintf_tmp);
			
			u8g2_DrawStr(u8g2, 0, 62, TipName[CurrentTip]);
			sprintf(sprintf_tmp, "%.1fV", (float)Vin*0.001);
			u8g2_DrawStr(u8g2, 83, 62, sprintf_tmp);

			u8g2_SetFont(u8g2, u8g2_font_freedoomr25_tn);
			
			if (ShowTemp > 500)
			{
				u8g2_DrawStr(u8g2, 37, 45, "000");
			}
			else
			{
				sprintf(sprintf_tmp, "%d", (uint16_t)CurrentTemp);
				u8g2_DrawStr(u8g2, 37, 45, sprintf_tmp);
			}
		}
		else
		{
			u8g2_SetFont(u8g2, u8g2_font_fub42_tn);
			if (ShowTemp > 500)
			{
				u8g2_DrawStr(u8g2, 15, 60, "000");
			}
			else
			{
				sprintf(sprintf_tmp, "%d", (uint16_t)CurrentTemp);
				u8g2_DrawStr(u8g2, 15, 60, sprintf_tmp);
			}
		}
  } while (u8g2_NextPage(u8g2));
}

void ROTARYCheck(void)
{
	SetTemp = getRotary();
		
	// check rotary encoder switch
	uint8_t c = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
	if ( !c && c0 ) 
	{
		beep();
		buttonmillis = get_sys_tick();
		while((!LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) && ((get_sys_tick() - buttonmillis) < 500));
		if ((get_sys_tick() - buttonmillis) >= 500) 
		{
			SetupScreen();
		}
		else
		{
			inBoostMode = !inBoostMode;
			if (inBoostMode) boostmillis = get_sys_tick();
			handleMoved = true;
		}
	}
	c0 = c;
	
	// check timer when in boost mode
  if (inBoostMode && timeOfBoost) 
	{
    goneSeconds = (get_sys_tick() - boostmillis) / 1000;
    if (goneSeconds >= timeOfBoost) {
      inBoostMode = false;              // stop boost mode
      beep();                           // beep if boost mode is over
      beepIfWorky = true;               // beep again when working temperature is reached
    }
  }
}

void Vin_Read(void) //LL_ADC_CHANNEL_11
{
	uint16_t result;
	result = denoiseAnalog(LL_ADC_CHANNEL_11);
	//printf("vin read %d, ", result);
	Vin = (__LL_ADC_CALC_DATA_TO_VOLTAGE(Vcc, result, LL_ADC_RESOLUTION_12B)*11); // ((47 + 4.7) / 4.7) = 11
	//printf("%hu mV\n", Vin);
}

void RawTemp_Read(void)
{
	uint16_t result;
	LL_TIM_OC_SetCompareCH1(TIM3, 0); // shut off heater in order to measure temperature
	LL_mDelay(2); // wait for voltage to settle
	
	result = denoiseAnalog(LL_ADC_CHANNEL_10);
	RawTemp = __LL_ADC_CALC_DATA_TO_VOLTAGE(Vcc, result, LL_ADC_RESOLUTION_12B);
	//printf("t12:%hu mV\n", (uint16_t)RawTemp);
	LL_TIM_OC_SetCompareCH1(TIM3, Output);
}

void Vref_Read(void) //LL_ADC_CHANNEL_VREFINT
{
	uint16_t result;
	result = denoiseAnalog(LL_ADC_CHANNEL_VREFINT);
	//printf("vref read %d, ", result);
	Vcc = __LL_ADC_CALC_VREFANALOG_VOLTAGE(result, LL_ADC_RESOLUTION_12B);
	//printf("%hu mV\n", Vcc);
}

void Temp_ADC_Read(void)
{
	uint16_t result;
	result = denoiseAnalog(LL_ADC_CHANNEL_TEMPSENSOR);
	//printf("temp read %d, ", result);
	
	temp_adc = __LL_ADC_CALC_TEMPERATURE(Vcc, result, LL_ADC_RESOLUTION_12B);
	//printf("%hu C\n", temp_adc);
}

// average several ADC readings in sleep mode to denoise
uint16_t denoiseAnalog(uint32_t adc_ch)
{
  uint16_t result = 0;
	//LL_ADC_Enable(ADC1);
	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, adc_ch);
	LL_ADC_SetChannelSamplingTime(ADC1, adc_ch, LL_ADC_SAMPLINGTIME_160CYCLES_5);
	for(uint8_t i=0; i<32; i++)
	{
		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_IsActiveFlag_EOC(ADC1) == RESET)
		{
			;
		}
		result += LL_ADC_REG_ReadConversionData12(ADC1);
		LL_ADC_ClearFlag_EOC(ADC1);
	}
	//LL_ADC_Disable(ADC1);
  return (result >> 5);                 // devide by 32 and return value
}

// reads temperature, vibration switch and supply voltages
void SENSORCheck(void)
{
	LL_TIM_OC_SetCompareCH1(TIM3, 0); // shut off heater in order to measure temperature
	LL_mDelay(1); // wait for voltage to settle
	
	double temp = denoiseAnalog(LL_ADC_CHANNEL_10);  // read ADC value for temperature
	uint8_t d = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_10); // check handle vibration switch
	if (d != d0) // set flag if handle was moved
	{
		handleMoved = true; 
		d0 = d;
	} 
	if (! SensorCounter--) // get Vin every now and then
	{
		Vin_Read();
	}

	LL_TIM_OC_SetCompareCH1(TIM3, Output);			// turn on again heater
	
  RawTemp += (temp - RawTemp) * SMOOTHIE;     // stabilize ADC temperature reading
  calculateTemp();                            // calculate real temperature value
	
  // stabilize displayed temperature when around setpoint
  if ((ShowTemp != Setpoint) || (fabs(ShowTemp - CurrentTemp) > 5))
	{
		ShowTemp = CurrentTemp;
	}
  if (fabs(ShowTemp - Setpoint) <= 1)
	{
		ShowTemp = Setpoint;
	}
	
	  // set state variable if temperature is in working range; beep if working temperature was just reached
  gap = fabs(SetTemp - CurrentTemp);
  if (gap < 5) 
	{
    if (!isWorky && beepIfWorky) beep();
    isWorky = true;
    beepIfWorky = false;
  }
  else 
	{
		isWorky = false;
	}
	
  // checks if tip is present or currently inserted
  if (ShowTemp > 500) // tip removed ?
	{
		TipIsPresent = false;
	}		
  if (!TipIsPresent && (ShowTemp < 500)) 			// new tip inserted ?
	{
    LL_TIM_OC_SetCompareCH1(TIM3, 0);     // shut off heater
    beep();                                   // beep for info
    TipIsPresent = true;                      // tip is present now
    ChangeTipScreen();                        // show tip selection screen
    //updateEEPROM();                           // update setting in EEPROM
    handleMoved = true;                       // reset all timers
    RawTemp = denoiseAnalog(LL_ADC_CHANNEL_10);     // restart temp smooth algorithm
    c0 = 0;                                 // switch must be released
    setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);  // reset rotary encoder
  }
}

// calculates real temperature value according to ADC reading and calibration values
void calculateTemp(void) 
{
  if      (RawTemp < 1212) CurrentTemp = map(RawTemp, 0, 1212, 21, CalTemp[CurrentTip][0]);
  else if (RawTemp < 1696) CurrentTemp = map(RawTemp, 1212, 1696, CalTemp[CurrentTip][0], CalTemp[CurrentTip][1]);
  else CurrentTemp = map(RawTemp, 1696, 2181, CalTemp[CurrentTip][1], CalTemp[CurrentTip][2]);
}

// controls the heater
void Thermostat(void)
{
  // define Setpoint acoording to current working mode
  if      (inOffMode)   Setpoint = 0;
  else if (inSleepMode) Setpoint = SleepTemp;
  else if (inBoostMode) Setpoint = SetTemp + BoostTemp;
  else                  Setpoint = SetTemp; 

  // control the heater (PID or direct)
  gap = fabs(Setpoint - CurrentTemp);
  if (PIDenable) {
    Input = CurrentTemp;
    if (gap < 30)
		{
			PID_SetTunings(&TPID, consKp, consKi, consKd);
		}
    else 
		{
			PID_SetTunings(&TPID, aggKp, aggKi, aggKd);
		}
    PID_Compute(&TPID);
  } 
	else 
	{
    // turn on heater if current temperature is below setpoint
    if ((CurrentTemp + 0.5) < Setpoint)
		{
			Output = 0;
		}
		else
		{
			Output = 1999;
		}
  }
	LL_TIM_OC_SetCompareCH1(TIM3, Output);
}

void SetupScreen(void)
{
	LL_TIM_OC_SetCompareCH1(TIM3, 0); // shut off heater
	beep();
	uint16_t SaveSetTemp = SetTemp;
	uint8_t selection = 0;
	bool repeat = true;
	
  while (repeat) 
	{
    selection = MenuScreen(SetupItems, 11, selection);
    switch (selection) 
		{
      case 0:   TipScreen(); repeat = false; break;
      case 1:   TempScreen(); break;
      case 2:   TimerScreen(); break;
      case 3:   PIDenable = MenuScreen(ControlTypeItems, 3, PIDenable); break;
      case 4:   MainScrType = MenuScreen(MainScreenItems, 3, MainScrType); break;
      case 5:   beepEnable = MenuScreen(BuzzerItems, 3, beepEnable); break;
//      case 6:   BodyFlip = MenuScreen(FlipItems, sizeof(FlipItems), BodyFlip); SetFlip(); break;
      case 7:   ECReverse = MenuScreen(ECReverseItems, 3, ECReverse); break;
      case 8:   InfoScreen(); break;
      default:  repeat = false; break;
    }
  }
	//updateEEPROM();
	handleMoved = true;
	SetTemp = SaveSetTemp;
	setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, SetTemp);
}

// tip settings screen
void TipScreen(void)
{
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) 
	{
    selection = MenuScreen(TipItems, 7, selection);
    switch (selection) 
		{
      case 0:
				ChangeTipScreen();
				break;
      case 1:
				CalibrationScreen();
				break;
      case 2:
				InputNameScreen();
				break;
      case 3:
				DeleteTipScreen();
				break;
      case 4:
				AddTipScreen();
				break;
      default:
				repeat = false;
				break;
    }
  }
}

// change tip screen
void ChangeTipScreen(void)
{
  uint8_t selected = CurrentTip;
  uint8_t lastselected = selected;
  int8_t  arrow = 0;
  if (selected)
  {
    arrow = 1;
  }
  setRotary(0, NumberOfTips - 1, 1, selected);
  bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;

  do 
  {
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2_FirstPage(&u8g2);
    do
    {
        u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
        u8g2_DrawStr(&u8g2, 0, 10, "Select Tip");
        u8g2_DrawStr(&u8g2, 0, 10+16 * (arrow + 1), ">");
        for (uint8_t i=0; i<3; i++)
        {
          uint8_t drawnumber = selected + i - arrow;
          if (drawnumber < NumberOfTips)
          {
            u8g2_DrawStr(&u8g2, 12, 10+16 * (i + 1), TipName[selected + i - arrow]);
          }
        }
      } while(u8g2_NextPage(&u8g2));
    if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0))
    {
      LL_mDelay(10); 
      lastbutton = false;
    }
  } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);

  beep();
  CurrentTip = selected;
}

// temperature calibration screen
void CalibrationScreen(void) {
  uint16_t CalTempNew[4]; 
	char sprintf_tmp[16] = {0};
	
  for (uint8_t CalStep = 0; CalStep < 3; CalStep++) 
	{
    SetTemp = CalTemp[CurrentTip][CalStep];
    setRotary(100, 500, 1, SetTemp);
    beepIfWorky = true;
    bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;

    do 
		{
      SENSORCheck();      // reads temperature and vibration switch of the iron
      Thermostat();       // heater control
      
      u8g2_FirstPage(&u8g2);
      do 
			{
        u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
        u8g2_DrawStr(&u8g2, 0, 10, "Calibration");
				sprintf(sprintf_tmp, "Step: %d of 3", CalStep + 1);
				u8g2_DrawStr(&u8g2, 0, 26, sprintf_tmp); 
        if (isWorky)
				{
					u8g2_DrawStr(&u8g2, 0, 42, "Set measured");
					sprintf(sprintf_tmp, "temp: %d", getRotary());
					u8g2_DrawStr(&u8g2, 0, 58, sprintf_tmp); 
        }
				else 
				{
					sprintf(sprintf_tmp, "ADC: %d", (uint16_t)RawTemp);
					u8g2_DrawStr(&u8g2, 0, 42, sprintf_tmp); 
					u8g2_DrawStr(&u8g2, 0, 58, "Please wait...");
        }
      } while(u8g2_NextPage(&u8g2));
			if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
			{
				LL_mDelay(10);
				lastbutton = false;
			}
    } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);

		CalTempNew[CalStep] = getRotary();
		beep(); 
		LL_mDelay(10);
  }

	LL_TIM_OC_SetCompareCH1(TIM3, 0); // shut off heater
  LL_mDelay(2); // wait for voltage to settle
  CalTempNew[3] = TMP75_ReadTemp(); // read temperature from TMP75
	
  if ((CalTempNew[0] + 10 < CalTempNew[1]) && (CalTempNew[1] + 10 < CalTempNew[2])) 
	{
    if (MenuScreen(StoreItems, 3, 0))
		{
      for (uint8_t i = 0; i < 4; i++) 
			{
				CalTemp[CurrentTip][i] = CalTempNew[i];
			}
    }
  }
}

// input tip name screen
void InputNameScreen(void) 
{
  uint8_t value, i;
  char sprintf_tmp[6] = {0};
  
  for (uint8_t digit = 0; digit < (TIPNAMELENGTH - 1); digit++) 
  {
    bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;
    setRotary(31, 96, 1, 65);
    do
    {
      value = getRotary();
      if (value == 31)
      {
        value = 95;
        setRotary(31, 96, 1, 95);
      }
      else if (value == 96) 
      {
        value = 32; 
        setRotary(31, 96, 1, 32);
      }
      u8g2_FirstPage(&u8g2);
      do 
      {
        u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
        u8g2_DrawStr(&u8g2, 0, 10, "Enter Tip Name");
        u8g2_DrawStr(&u8g2, 9 * digit, 58, "^");
        for (uint8_t i = 0; i < digit; i++) 
        {
          sprintf_tmp[i] = TipName[CurrentTip][i];
        }
        sprintf_tmp[digit] = value;
        u8g2_DrawStr(&u8g2, 0, 42, sprintf_tmp);
      } while(u8g2_NextPage(&u8g2));
      if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
      {
        LL_mDelay(10);
        lastbutton = false;
      }
    } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);
    TipName[CurrentTip][digit] = value;
    beep();
    LL_mDelay(10);
  }
  TipName[CurrentTip][TIPNAMELENGTH - 1] = 0;
  return;
}

// add new tip screen
void AddTipScreen() 
{
  if (NumberOfTips < TIPMAX) 
	{
    CurrentTip = NumberOfTips++; 
		InputNameScreen();
    CalTemp[CurrentTip][0] = TEMP1212;
		CalTemp[CurrentTip][1] = TEMP1696;
    CalTemp[CurrentTip][2] = TEMP2181;
		CalTemp[CurrentTip][3] = TEMPCHP;
  } 
	else
	{
		MessageScreen(MaxTipMessage, 4);
	}
}	

// delete tip screen
void DeleteTipScreen(void)
{
  if (NumberOfTips == 1)
	{
		MessageScreen(DeleteMessage, 4);
	}
  else if (MenuScreen(SureItems, 3, 0)) 
	{
    if (CurrentTip == (NumberOfTips - 1)) 
		{
			CurrentTip--;
		}
    else 
		{
      for (uint8_t i = CurrentTip; i < (NumberOfTips - 1); i++) 
			{
        for (uint8_t j = 0; j < TIPNAMELENGTH; j++)
				{
					TipName[i][j] = TipName[i+1][j];
				}
        for (uint8_t j = 0; j < 4; j++)
				{
					CalTemp[i][j] = CalTemp[i+1][j];
				}
      }
    }
    NumberOfTips--;
  }
}

void MessageScreen(const char *Items[], uint8_t numberOfItems)
{
  bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;
  u8g2_FirstPage(&u8g2);
  do 
	{
    u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
    for (uint8_t i = 0; i < numberOfItems; i++) 
		{
			u8g2_DrawStr(&u8g2, 0, 10+i * 16, Items[i]);
		}
  } while(u8g2_NextPage(&u8g2));
  do 
	{
    if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
		{
			LL_mDelay(10);
			lastbutton = false;
		}
  } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);
  beep();
}

// information display screen
void InfoScreen(void)
{
  bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;
	char sprintf_tmp[16] = {0};
	
  do 
	{
    Vref_Read();                     // read input voltage
    float fVcc = (float)Vcc / 1000;     // convert mV in V
    Vin_Read();                     // read supply voltage
    float fVin = (float)Vin / 1000;     // convert mv in V
		Temp_ADC_Read();
    u8g2_FirstPage(&u8g2);
      do
			{
        u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
				sprintf(sprintf_tmp, "Firmware: %s", VERSION);
        u8g2_DrawStr(&u8g2, 0, 10, sprintf_tmp);
				sprintf(sprintf_tmp, "Tmp: %dC", temp_adc);
        u8g2_DrawStr(&u8g2, 0, 26, sprintf_tmp); 
				sprintf(sprintf_tmp, "Vin: %.1fV", fVin);
        u8g2_DrawStr(&u8g2, 0, 42, sprintf_tmp);
				sprintf(sprintf_tmp, "Vcc: %.1fV", fVcc);
        u8g2_DrawStr(&u8g2, 0, 58, sprintf_tmp);
				LL_mDelay(50);
      } while(u8g2_NextPage(&u8g2));
		if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
		{
			LL_mDelay(10);
			lastbutton = false;
		}
  } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);

  beep();
}

// timer settings screen
void TimerScreen(void)
{
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat)
	{
    selection = MenuScreen(TimerItems, 5, selection);
    switch (selection) 
		{
      case 0:
				setRotary(0, 30, 1, time2sleep);
				time2sleep = InputScreen(SleepTimerItems);
				break;
      case 1:
				setRotary(0, 60, 5, time2off);
				time2off = InputScreen(OffTimerItems); 
				break;
      case 2:
				setRotary(0, 180, 10, timeOfBoost);
				timeOfBoost = InputScreen(BoostTimerItems);
				break;
      default:  
				repeat = false; 
				break;
    }
  }
}

uint8_t MenuScreen(const char *Items[], uint8_t numberOfItems, uint8_t selected) 
{
  bool isTipScreen;
	isTipScreen = (strcmp(Items[0], "Tip:") == 0) ? 1 : 0;
  uint8_t lastselected = selected;
  int8_t arrow = 0;
  if (selected)
	{
		arrow = 1;
	}
  //numberOfItems >>= 1;
  setRotary(0, numberOfItems - 2, 1, selected);
  bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;

  do 
	{
		selected = 0;
		arrow = 0;
    selected = getRotary();
    arrow = constrain(arrow + selected - lastselected, 0, 2);
    lastselected = selected;
    u8g2_FirstPage(&u8g2);
		do
		{
			u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
			u8g2_DrawStr(&u8g2, 0, 10, Items[0]);
			if (isTipScreen) 
			{
				u8g2_DrawStr(&u8g2, 54, 0, TipName[CurrentTip]);
			}
			u8g2_DrawStr(&u8g2, 0, 10+16 * (arrow + 1), ">");
			for (uint8_t i=0; i<3; i++) 
			{
				uint8_t drawnumber = selected + i + 1 - arrow;
				if (drawnumber < numberOfItems)
				{
					u8g2_DrawStr(&u8g2, 12, 10+16 * (i + 1), Items[selected + i + 1 - arrow]);
				}
			}
		} while(u8g2_NextPage(&u8g2));
		if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
		{
			LL_mDelay(10);
			lastbutton = false;
		}
  } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);

  beep();
  return selected;
}

// temperature settings screen
void TempScreen(void)
{
  uint8_t selection = 0;
  bool repeat = true;  
  while (repeat) 
	{
    selection = MenuScreen(TempItems, 5, selection);
    switch (selection) 
		{
      case 0:
				setRotary(TEMP_MIN, TEMP_MAX, TEMP_STEP, DefaultTemp);
				DefaultTemp = InputScreen(DefaultTempItems);
				break;
      case 1:
				setRotary(20, 200, TEMP_STEP, SleepTemp);
				SleepTemp = InputScreen(SleepTempItems);
				break;
      case 2:
				setRotary(10, 100, TEMP_STEP, BoostTemp);
				BoostTemp = InputScreen(BoostTempItems); 
				break;
      default:
				repeat = false; 
				break;
    }
  }
}

// input value screen
uint16_t InputScreen(const char *Items[]) 
{
  uint16_t value = 0;
  bool lastbutton = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) ? 0 : 1;
	char sprintf_tmp[16] = {0};
	
  do
	{
    value = getRotary();
    u8g2_FirstPage(&u8g2);
      do
			{
        u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
        u8g2_DrawStr(&u8g2, 0, 10, Items[0]);
        u8g2_DrawStr(&u8g2, 0, 42, ">"); 
        if (value == 0)
        {
          u8g2_DrawStr(&u8g2, 10, 42, "Deactivated");
        }
        else
        {
          sprintf(sprintf_tmp, "%d %s", value, Items[1]);
          u8g2_DrawStr(&u8g2, 10, 42, sprintf_tmp);
        }            
      } while(u8g2_NextPage(&u8g2));
		if (lastbutton && LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0)) 
		{
			LL_mDelay(10);
			lastbutton = false;
		}
  } while (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0) || lastbutton);

  beep();
  return value;
}

void beep(void)
{
	if(beepEnable)
	{
		LL_TIM_EnableCounter(TIM14);
		LL_mDelay(32);
		LL_TIM_DisableCounter(TIM14);
	}
}

// sets start values for rotary encoder
void setRotary(int rmin, int rmax, int rstep, int rvalue)
{
  countMin  = rmin << ROTARY_TYPE;
  countMax  = rmax << ROTARY_TYPE;
  countStep = ECReverse ? -rstep : rstep;
  count     = rvalue << ROTARY_TYPE;  
}

// reads current rotary encoder value
uint16_t getRotary(void)
{
	//printf("ec11:%d\n", count);
	return (count >> ROTARY_TYPE);
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Read_System_Parmeter(void)
{
	uint32_t test;
	test = *((uint32_t *)SYSTEM_ARG_STORE_START_ADDR);
//	uint16_t len;
//	//uint8_t crc;
//	
//	len = XMEM_ALIGN_SIZE(sizeof(SystemParamStore), 8);
//	printf("SystemParamStore size: %d %d\n", sizeof(SystemParamStore), len);
//	
//	if(len > (FLASH_PAGE_SIZE >> 3))
//	{
//		len = (FLASH_PAGE_SIZE >> 3);
//		printf("SystemParam size over flash page size...\n");
//	}
//	
//	vFlash_Read_DoubleWord(SYSTEM_ARG_STORE_START_ADDR, (uint64_t *)&SystemParam, len);
}

uint32_t get_sys_tick(void)
{
	return TIM16_Tick;
}

void t1_1s_cb(void* para)
{
	//printf("t1_cb at %d\n", get_sys_tick());
}

void t2_300ms_cb(void* para)
{
	u8g2_update_flag = 0;
	//printf("t2_cb at %d\n", get_sys_tick());
}

int constrain(int x, int min, int max) {
	if (x < min) 
	{
		return min;
	} 
	else if (x > max) 
	{
		return max;
	} 
	else 
	{
		return x;
	}
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
