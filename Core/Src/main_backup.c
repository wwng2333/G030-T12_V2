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
//#include "EventRecorder.h"
#include "oled_driver.h"
#include "stdio.h"
#include "i2c.h"
#include "PID.h"
#include "math.h"
#include <string.h>
#include "SEGGER_RTT.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


// 1. 系统配置结构体 (保存到 EEPROM/Flash)
typedef struct {
    uint16_t defaultTemp;
    uint16_t sleepTemp;
    uint16_t boostTemp;
    
    uint16_t time2sleep;
    uint16_t time2off;
    uint16_t timeOfBoost;
    
    // 采用位域压缩的标志位，仅占用 1 Byte
    union {
        uint8_t allFlags;
        struct {
            uint8_t buzzerEnabled   : 1;
            uint8_t screenFlipped   : 1;
            uint8_t encoderReversed : 1;
            uint8_t mainScreenType  : 1;
            uint8_t controlType     : 1; // 0: Direct, 1: PID
            uint8_t reserved        : 3;
        };
    } flags;
    
    uint8_t currentTip;
    uint8_t numberOfTips;
} SystemConfig_t;

// 3. 硬件外设状态结构体
typedef struct {
    uint16_t vcc_mV;         // 供电电压
    uint16_t vin_mV;         // 输入电压
    uint16_t mcuTemp_C;      // 芯片内部温度
} HardwareState_t;

// 4. 系统主状态枚举
typedef enum {
    SYS_STATE_BOOT,
    SYS_STATE_RUNNING
} SystemState_e;

// leagcy ===

typedef enum {
    CTRL_HEATING,         // 加热周期 (0 - 95ms)
    CTRL_WAIT_SETTLE,     // 断电后等待电压稳定 (95 - 98ms)
    CTRL_MEASURE_AND_PID  // 采样并执行PID控制 (98 - 100ms)
} TempControlState_e;

typedef enum {
    BTN_RELEASED,
    BTN_PRESSED,
    BTN_LONG_PRESSED
} ButtonState;

typedef enum {
    DISPLAY_UPDATE,
    DISPLAY_WAIT
} DisplayState;

typedef struct {
    int16_t last_counter;
    int32_t total_count;
    int16_t delta;
} Encoder_Status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THERMOSTAT_PERIOD_MS 200
#define DISPLAY_PERIOD_MS 50
#define SENSOR_CHECK_PERIOD_MS 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

Encoder_Status enc = {0, 0, 0};

// 赋予掉电配置初始出厂值，防止 EEPROM 未读取时数据为 0
SystemConfig_t sysConfig = {
    .defaultTemp = 300,        // 默认目标温度 300°C
    .sleepTemp = 200,          // 休眠温度 200°C
    .boostTemp = 50,           // 强化模式提升 50°C
    .time2sleep = 5,           // 5 分钟后休眠
    .time2off = 15,            // 15 分钟后关机
    .timeOfBoost = 60,         // 强化模式持续 60 秒
    //.flags.allFlags = 0,       // 先清零所有标志
    .flags.mainScreenType = 1, // 默认大字丰富界面
    .flags.buzzerEnabled = 1,  // 默认开启蜂鸣器
    .flags.controlType = 1,    // 默认开启 PID 控制
    .currentTip = 0,
    .numberOfTips = 1
};
RuntimeState_t  runState;
SystemState_e   g_SysState = SYS_STATE_BOOT;

static u8g2_t u8g2;

IntPID_TypeDef TPID;

// Define the aggressive and conservative PID tuning parameters
double aggKp=5120, aggKi=5120, aggKd=0;
double consKp=5120, consKi=5120, consKd=0; 

// Default values for tips
uint16_t  CalTemp[TIPMAX][4] = {TEMP1212, TEMP1696, TEMP2181, TEMPCHP};
char      TipName[TIPMAX][TIPNAMELENGTH] = {TIPNAME};
uint8_t   CurrentTip   = 0;
uint8_t   NumberOfTips = 1;

// Menu items
// Variables for pin change interrupt
volatile uint8_t  d0;
// Variables for voltage readings
uint16_t  Vcc, Vin;

__IO uint32_t TIM16_Tick = 0;

TempControlState_e controlState = CTRL_HEATING;

static ButtonState g_buttonState = BTN_RELEASED;
static uint32_t g_buttonPressTimestamp = 0;

static DisplayState displayState = DISPLAY_UPDATE;
static uint32_t displayTimestamp = 0;

bool ROTARYCheck_IsMenuRequested(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Process_Encoder(void);
void DisplayManager(void);
void MainScreen(void);
void Vref_Read(void);
uint16_t denoiseAnalog(uint32_t adc_ch);
void calculateTemp(void);
void ROTARYCheck(void);
void SLEEPCheck(void);
void TemperatureControl_Task(void);
uint16_t InputScreen(const char *Items[]);
void beep(void);
uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
void Read_System_Parmeter(void);
uint32_t get_sys_tick(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void System_Boot_Handler(void) {
    // 读取 EEPROM (先注释掉，等后面重构EEPROM读写)
    // getEEPROM(); 
    
    // 初始化运行状态
    runState.setTemp = sysConfig.defaultTemp;
    runState.setpoint = sysConfig.defaultTemp;  // 初始化 setpoint，避免显示为 0
    runState.pidOutput = 0;  // 初始化 pidOutput
    runState.inSleepMode = false;
    runState.inOffMode = false;
    runState.inBoostMode = false;
    runState.displayNeedsUpdate = true;
        
    Vref_Read();
    
    // 初始化温度读取
    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_mDelay(2);
    runState.rawTemp = denoiseAnalog(LL_ADC_CHANNEL_10);
    calculateTemp();

    beep(); beep();
    
    g_SysState = SYS_STATE_RUNNING;
}

void System_Running_Handler(void) {
    // 1. 检查编码器与按键 (如果长按，返回 true，准备进入菜单)
    if (ROTARYCheck_IsMenuRequested()) {
    //    g_SysState = SYS_STATE_MENU;
        return; // 立即交出控制权
    }
    
		// 检查定时器通道是否开启，并且是否到了关闭时间
    if ((TIM14->CCER & TIM_CCER_CC1E) && (get_sys_tick() >= runState.beepTurnOffTick))
    {
        LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    }
		
    // 2. 业务逻辑检测
    SLEEPCheck();
    TemperatureControl_Task();
    DisplayManager();
    
    // 3. 屏幕刷新
    if (runState.displayNeedsUpdate) {
        MainScreen();
        runState.displayNeedsUpdate = false;
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
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, 3);

  /* USER CODE BEGIN Init */
	//EventRecorderInitialize(EventRecordAll, 1U);
	//EventRecorderStart();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	//printf("CPU @ %d Hz\n", SystemCoreClock);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM17_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */
	Activate_ADC();
	u8g2_Setup_ssd1306_128x64_noname_1(&u8g2, U8G2_R0, u8x8_byte_4wire_hw_spi, u8x8_stm32_gpio_and_delay);
	u8g2_InitDisplay(&u8g2);
	u8g2_SetPowerSave(&u8g2, 0);
	
	IntPID_Init(&TPID, aggKp, aggKi, aggKd, 0, 1999);

	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
	LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
	LL_TIM_EnableCounter(TIM1);

	LL_TIM_EnableAllOutputs(TIM3);
	LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
	LL_TIM_OC_SetCompareCH1(TIM3, 0);
	LL_TIM_EnableCounter(TIM3);

	LL_TIM_EnableAllOutputs(TIM14); //Enable TIM for beep
	LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
	LL_TIM_EnableCounter(TIM14); //Disable now, beep later.
	
	LL_TIM_EnableAllOutputs(TIM16); //Enable TIM for tick
	LL_TIM_EnableIT_CC1(TIM16); //Enable TIM16 Interrupt
	LL_TIM_CC_EnableChannel(TIM16, LL_TIM_CHANNEL_CH1N);
	LL_TIM_EnableCounter(TIM16);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		Process_Encoder();
				switch (g_SysState) {
            case SYS_STATE_BOOT:
                System_Boot_Handler();
                break;
                
            case SYS_STATE_RUNNING:
                System_Running_Handler();
                break;
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

void Process_Encoder(void) 
{
    // 用于吸收多余的脉冲（解决拧一格动两下的问题）
    static int16_t step_accumulator = 0; 
    
    uint16_t current_counter = LL_TIM_GetCounter(TIM1);
    
    // 1. 计算差值 (利用无符号溢出特性)
    int16_t delta = (int16_t)(current_counter - enc.last_counter);
    enc.last_counter = current_counter;

    // 2. 检查差值，无动作则直接退出，节省 CPU
    if (delta == 0) {
        return;
    }

    // 3. 应用反向设置标志 (读取 sysConfig 中的位域)
    if (sysConfig.flags.encoderReversed) {
        delta = -delta;
    }

    // 4. 将差值放入累加器池子
    step_accumulator += delta;

    // 5. 步数结算（阈值为 2，如果发现拧一格依然跳了2下，可以把 2 改成 4）
    if (step_accumulator >= 2) {
        runState.setTemp += 5;       // 正转加 5 度
        step_accumulator -= 2;       // 消耗脉冲
        runState.handleMoved = true; // 唤醒系统，退出休眠
        runState.displayNeedsUpdate = true; // 触发屏幕刷新
    } 
    else if (step_accumulator <= -2) {
        runState.setTemp -= 5;       // 反转减 5 度
        step_accumulator += 2;       // 消耗脉冲
        runState.handleMoved = true; // 唤醒系统，退出休眠
        runState.displayNeedsUpdate = true; // 触发屏幕刷新
    }

    // 6. 温度边界限制 (确保你在 main.h 或之前定义了 TEMP_MIN 和 TEMP_MAX，比如 150 和 450)
    // 这里借用你代码底部已经写好的 constrain 函数
    runState.setTemp = constrain(runState.setTemp, TEMP_MIN, TEMP_MAX);
}

void DisplayManager(void)
{
    switch(displayState)
    {
        case DISPLAY_UPDATE:
            runState.displayNeedsUpdate = true; // 只设置标志位
            displayTimestamp = get_sys_tick();
            displayState = DISPLAY_WAIT;
            break;
            
        case DISPLAY_WAIT:
            if (get_sys_tick() - displayTimestamp >= DISPLAY_PERIOD_MS)
            {
                displayState = DISPLAY_UPDATE;
            }
            break;
    }
}

void MainScreen(void)
{
  char sprintf_tmp[8] = {0};
  static float cached_tmp75_temp = 0.0f;  // 缓存 TMP75 温度读数
  static uint32_t last_tmp75_read = 0;    // 上次读取时间

  // 每 500ms 更新一次 TMP75 温度读数，避免频繁 I2C 读取阻塞主循环
  if (get_sys_tick() - last_tmp75_read >= 500) {
    cached_tmp75_temp = TMP75_ReadTemp();
    last_tmp75_read = get_sys_tick();
  }

  u8g2_FirstPage(&u8g2);
  u8g2_SetFontMode(&u8g2, 1);
  u8g2_SetFontDirection(&u8g2, 0);
  do
  {
    // =========================================================
    // 顶部状态栏 (全局通用)
    // =========================================================
    u8g2_SetFont(&u8g2, u8g2_font_9x15_tr);
    u8g2_DrawStr(&u8g2, 0, 10, "SET:");
    sprintf(sprintf_tmp, "%hu", (uint16_t)runState.setpoint);
    u8g2_DrawStr(&u8g2, 40, 10, sprintf_tmp);
		
    if(runState.currentTemp > 500) strcpy(sprintf_tmp, "ERROR");
    else if(runState.inOffMode) strcpy(sprintf_tmp, "  OFF");
    else if(runState.inSleepMode) strcpy(sprintf_tmp, "SLEEP");
    else if(runState.inBoostMode) strcpy(sprintf_tmp, "BOOST");
    else if(runState.isWorky) strcpy(sprintf_tmp, "WORKY");
    else if(runState.pidOutput < 180) strcpy(sprintf_tmp, " HEAT");
    else strcpy(sprintf_tmp, " HOLD");
    u8g2_DrawStr(&u8g2, 83, 10, sprintf_tmp);

    // 计算 PWM 百分比 (0 - 100)
    uint8_t pwm_percent = (uint8_t)(runState.pidOutput / 1999.0 * 100.0);
    if (pwm_percent > 100) pwm_percent = 100;
		
    // =========================================================
    // 区分不同显示模式
    // =========================================================
    if(sysConfig.flags.mainScreenType)
    {
        // ----- 1. 详细信息模式 (不显示进度条，保持原样) -----
        if(d0) //如果烙铁头振动开关动作了，就在屏幕上显示一个小点
        {
            u8g2_DrawStr(&u8g2, 52, 62, ".");
        }
        
        sprintf(sprintf_tmp, "%d%%", pwm_percent);
        u8g2_DrawStr(&u8g2, 92, 28, sprintf_tmp);

        sprintf(sprintf_tmp, "%.1fC", cached_tmp75_temp);  // 使用缓存的温度值
        u8g2_DrawStr(&u8g2, 81, 45, sprintf_tmp);
        
        u8g2_DrawStr(&u8g2, 0, 62, TipName[sysConfig.currentTip]);
        sprintf(sprintf_tmp, "%.1fV", (float)Vin*0.001);
        u8g2_DrawStr(&u8g2, 83, 62, sprintf_tmp);

        u8g2_SetFont(&u8g2, u8g2_font_freedoomr25_tn);
        if (runState.currentTemp > 500)
        {
            u8g2_DrawStr(&u8g2, 10, 45, "ERR");
        }
        else
        {
            sprintf(sprintf_tmp, "%d", (uint16_t)runState.currentTemp);
            u8g2_DrawStr(&u8g2, 10, 45, sprintf_tmp);
        }
    }
    else
    {
        // ----- 2. 大字模式 (左侧绘制垂直进度条) -----
        
        // 画进度条外边框: 避开顶部的"SET:"文字，从 y=16 处往下画，高度 48
        u8g2_DrawFrame(&u8g2, 0, 16, 5, 48); 
        
        // 计算内部实心条的高度 (边框内部可用最大高度为 46)
        uint8_t bar_height = (pwm_percent * 46) / 100;
        
        // 从底部 (y=63) 往上画实心填充
        if (bar_height > 0) {
            u8g2_DrawBox(&u8g2, 1, 64 - bar_height - 1, 3, bar_height); 
        }

        // 绘制大字体 (整体向右偏移 6 个像素，让出左侧进度条的空间)
        u8g2_SetFont(&u8g2, u8g2_font_7Segments_26x42_mn);
        if (runState.currentTemp > 500)
        {
            u8g2_DrawStr(&u8g2, 21, 60, "ERR"); // 15 + 6 = 21
        }
        else
        {
            sprintf(sprintf_tmp, "%d", (uint16_t)runState.currentTemp);
            if (runState.currentTemp < 100)
            {
                u8g2_DrawStr(&u8g2, 38, 60, sprintf_tmp); // 32 + 6 = 38
            }
            else
            {
                u8g2_DrawStr(&u8g2, 21, 60, sprintf_tmp); // 15 + 6 = 21
            }
        }
    }
  } while (u8g2_NextPage(&u8g2));
}

bool ROTARYCheck_IsMenuRequested(void)
{
    bool requestMenu = false;

    bool is_button_down = !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
    switch (g_buttonState) {
        case BTN_RELEASED:
            if (is_button_down) {
                g_buttonPressTimestamp = get_sys_tick();
                g_buttonState = BTN_PRESSED;
            }
            break;

        case BTN_PRESSED:
            if (!is_button_down) {
                // 短按事件
                beep();
                runState.inBoostMode = !runState.inBoostMode;
                if (runState.inBoostMode) runState.boostMillis = get_sys_tick();
                runState.handleMoved = true;
                runState.displayNeedsUpdate = true;
                g_buttonState = BTN_RELEASED;
            } else if (get_sys_tick() - g_buttonPressTimestamp >= 500) {
                // 长按事件：不再直接调用 SetupScreen，而是返回 true 让状态机去处理
                beep();
                requestMenu = true; 
                g_buttonState = BTN_LONG_PRESSED;
            }
            break;
            
        case BTN_LONG_PRESSED:
            if (!is_button_down) {
                g_buttonState = BTN_RELEASED;
            }
            break;
    }

    // 处理强化模式倒计时
    if (runState.inBoostMode && sysConfig.timeOfBoost) {
        uint8_t goneSeconds = (get_sys_tick() - runState.boostMillis) / 1000;
        if (goneSeconds >= sysConfig.timeOfBoost) {
            runState.inBoostMode = false;
            beep();
            runState.beepIfWorky = true;
            runState.displayNeedsUpdate = true;
        }
    }
    
    return requestMenu;
}

void Vref_Read(void) //LL_ADC_CHANNEL_VREFINT
{
	uint16_t result;
	result = denoiseAnalog(LL_ADC_CHANNEL_VREFINT);
	//printf("vref read %d, ", result);
	Vcc = __LL_ADC_CALC_VREFANALOG_VOLTAGE(result, LL_ADC_RESOLUTION_12B);
	//printf("%hu mV\n", Vcc);
}

// average several ADC readings in sleep mode to denoise
uint16_t denoiseAnalog(uint32_t adc_ch)
{
  uint32_t result = 0;

	LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, adc_ch);
	LL_ADC_SetChannelSamplingTime(ADC1, adc_ch, LL_ADC_SAMPLINGTIME_160CYCLES_5);
	
	SCB->SCR |= SCB_SCR_SEVONPEND_Msk; // 开启 SEVONPEND，允许被挂起的外设标志直接唤醒 WFE (无需配置 NVIC 和中断函数)
	LL_ADC_EnableIT_EOC(ADC1); // 开启 ADC_EOC 级别的中断信号产生
	
	for(uint8_t i=0; i<16; i++)
	{
		LL_ADC_REG_StartConversion(ADC1);
		while(LL_ADC_IsActiveFlag_EOC(ADC1) == RESET)
		{
			__WFE(); // CPU 内核时钟停止，进入 Sleep 模式 
		}
		result += LL_ADC_REG_ReadConversionData12(ADC1);
		LL_ADC_ClearFlag_EOC(ADC1);
		NVIC_ClearPendingIRQ(ADC1_IRQn);
	}
	LL_ADC_DisableIT_EOC(ADC1); // 恢复系统默认状态 (关闭中断信号和 SEVONPEND)
	SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk; 
  return (uint16_t)(result >> 4);                 // divide by 16 and return value
}

// check and activate/deactivate sleep modes
void SLEEPCheck(void) 
{
  if (runState.handleMoved) 
  {
    if (runState.inSleepMode) 
    {
      if ((runState.currentTemp + 20) < runState.setTemp) 
      {
        LL_TIM_OC_SetCompareCH1(TIM3, (uint32_t)runState.pidOutput); 
      }
      beep();                           
      runState.beepIfWorky = true;               
      runState.displayNeedsUpdate = true;
    }
    runState.handleMoved = false;                
    runState.inSleepMode = false;                
    runState.inOffMode   = false;                
    runState.sleepMillis = get_sys_tick();       
    runState.displayNeedsUpdate = true;
  }

  uint32_t goneMinutes = (get_sys_tick() - runState.sleepMillis) / 60000;
  if ((!runState.inSleepMode) && (sysConfig.time2sleep > 0) && (goneMinutes >= sysConfig.time2sleep)) 
  {
    runState.inSleepMode = true;
    beep();
    runState.displayNeedsUpdate = true;
  }
  if((!runState.inOffMode) && (sysConfig.time2off > 0) && (goneMinutes >= sysConfig.time2off)) 
  {
    runState.inOffMode = true; 
    beep();
    runState.displayNeedsUpdate = true;
  }
}

// calculates real temperature value according to ADC reading and calibration values
void calculateTemp(void)
{
  if      (runState.rawTemp < 1212) runState.currentTemp = map(runState.rawTemp, 0, 1212, 21, CalTemp[sysConfig.currentTip][0]);
  else if (runState.rawTemp < 1696) runState.currentTemp = map(runState.rawTemp, 1212, 1696, CalTemp[sysConfig.currentTip][0], CalTemp[sysConfig.currentTip][1]);
  else if (runState.rawTemp < 2181) runState.currentTemp = map(runState.rawTemp, 1696, 2181, CalTemp[sysConfig.currentTip][1], CalTemp[sysConfig.currentTip][2]);
  else runState.currentTemp = CalTemp[sysConfig.currentTip][2] + (runState.rawTemp - 2181) * 0.2; // 超出范围时使用外推

  //SEGGER_RTT_printf(0, "%d\r\n", (uint16_t)runState.currentTemp);
}

void TemperatureControl_Task(void)
{
    static uint32_t lastCycleTick = 0;
		static uint32_t offTick = 0;
    uint32_t currentTick = get_sys_tick();
    
    // 防止开机或从菜单退出后，时间差过大导致时序错乱
    if (currentTick - lastCycleTick > 200) {
        lastCycleTick = currentTick; 
        controlState = CTRL_HEATING;
        LL_TIM_OC_SetCompareCH1(TIM3, (uint32_t)runState.pidOutput); 
    }
    
    switch (controlState) {
        case CTRL_HEATING:
            // 加热阶段持续 95ms
            if (currentTick - lastCycleTick >= 95) {
                LL_TIM_OC_SetCompareCH1(TIM3, 0); // 必须切断加热才能准确测温
								offTick = currentTick;
                controlState = CTRL_WAIT_SETTLE;
            }
            break;
            
        case CTRL_WAIT_SETTLE:
            // 等待 3ms，让热电偶/运放的电压彻底稳定 (95 + 3 = 98ms)
            if (currentTick - offTick >= 3) {
                controlState = CTRL_MEASURE_AND_PID;
            }
            break;
            
        case CTRL_MEASURE_AND_PID:
            // 1. 低功耗采样与平滑滤波
            runState.rawTemp += (denoiseAnalog(LL_ADC_CHANNEL_10) - runState.rawTemp) * SMOOTHIE;
            calculateTemp(); // 计算结果存入 runState.currentTemp

            // 2. 确定当前工作模式下的目标温度
            if      (runState.inOffMode)   runState.setpoint = 0;
            else if (runState.inSleepMode) runState.setpoint = sysConfig.sleepTemp;
            else if (runState.inBoostMode) runState.setpoint = runState.setTemp + sysConfig.boostTemp;
            else                           runState.setpoint = runState.setTemp; 

            // 3. 执行 PID 计算
            runState.pidInput = runState.currentTemp;
            runState.pidGap = fabs(runState.setpoint - runState.currentTemp);

            // 动态 PID 参数分配：温差大时满血加热，接近时保守逼近
						if (runState.pidGap < 10) {
                IntPID_SetTunings(&TPID, consKp, consKi, consKd);
            } else {
                IntPID_SetTunings(&TPID, aggKp, aggKi, aggKd);
            }
						runState.pidOutput = IntPID_Compute(&TPID, (int32_t)runState.setpoint, (int32_t)runState.currentTemp);

            // 4. 工作状态判定与蜂鸣器逻辑
            if (runState.pidGap < 5) {
                if (!runState.isWorky && runState.beepIfWorky) beep();
                runState.isWorky = true;
                runState.beepIfWorky = false;
            } else {
                runState.isWorky = false;
            }

            // 5. 立即重新开启加热，装载新的 PWM 输出值
            LL_TIM_OC_SetCompareCH1(TIM3, (uint32_t)runState.pidOutput);
            
            // 6. 要求屏幕刷新数据
            runState.displayNeedsUpdate = true;

            // 7. 开启下一个完整的 100ms 周期
            lastCycleTick = currentTick; 
            controlState = CTRL_HEATING;
            break;
    }
}

void beep(void)
{
	if(sysConfig.flags.buzzerEnabled)
	{
		LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
		runState.beepTurnOffTick = get_sys_tick() + 32;
	}
}

uint16_t map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
	// 防止除零错误
	if (in_max == in_min) {
		return out_min;
	}

	// 使用 uint32_t 进行中间计算，防止溢出
	uint32_t temp = (uint32_t)(x - in_min) * (uint32_t)(out_max - out_min);
	return (uint16_t)(temp / (uint32_t)(in_max - in_min) + out_min);
}

uint32_t get_sys_tick(void)
{
	return TIM16_Tick;
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