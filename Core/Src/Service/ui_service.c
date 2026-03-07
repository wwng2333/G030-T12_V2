/**
 ******************************************************************************
 * @file    ui_service.c
 * @brief   UI服务实现(第1部分)
 ******************************************************************************
 */

#include "ui_service.h"
#include "config_service.h"
#include "temp_control.h"
#include "power_mgmt.h"
#include "adc_hal.h"
#include "encoder_hal.h"
#include "button_hal.h"
#include "buzzer_hal.h"
#include "timer_hal.h"
#include "oled_driver.h"
#include "i2c.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

/* 私有类型 ----------------------------------------------------------*/
typedef struct {
    int16_t last_counter;
    int32_t total_count;
    int16_t delta;
} EncoderStatus_t;

typedef enum {
    DISPLAY_UPDATE,
    DISPLAY_WAIT
} DisplayState_e;

/* 私有变量 ----------------------------------------------------------*/
static EncoderStatus_t s_encoderStatus = {0, 0, 0};
static DisplayState_e s_displayState = DISPLAY_UPDATE;
static uint32_t s_displayTimestamp = 0;
static bool s_displayNeedsUpdate = false;
static UIEventCallback_t s_eventCallback = NULL;

static u8g2_t s_u8g2;

// 外部变量
extern uint16_t Vin;
extern volatile uint8_t d0;

/* 私有函数声明 ----------------------------------------------------*/
static void UIService_ProcessEncoderInternal(void);
static void UIService_ProcessButtonInternal(void);
static void UIService_DisplayManagerTask(void);

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化UI服务
 */
void UIService_Init(void)
{
    // 初始化OLED显示
    u8g2_Setup_ssd1306_128x64_noname_f(&s_u8g2, U8G2_R0,
                                       u8x8_byte_4wire_hw_spi,
                                       u8x8_stm32_gpio_and_delay);
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);

    // 初始化编码器状态
    s_encoderStatus.last_counter = Encoder_HAL_GetCounter();
    s_encoderStatus.total_count = 0;
    s_encoderStatus.delta = 0;

    // 初始化显示状态
    s_displayState = DISPLAY_UPDATE;
    s_displayTimestamp = Timer_HAL_GetTick();
    s_displayNeedsUpdate = true;
}

/**
 * @brief  UI服务任务(需要周期性调用)
 */
void UIService_Task(void)
{
    // 1. 处理编码器输入
    UIService_ProcessEncoderInternal();

    // 2. 处理按键输入
    UIService_ProcessButtonInternal();

    // 3. 显示管理
    UIService_DisplayManagerTask();

    // 4. 刷新显示
    if (s_displayNeedsUpdate) {
        UIService_DisplayMainScreen();
        s_displayNeedsUpdate = false;
    }
}

/**
 * @brief  注册UI事件回调函数
 * @param  callback: 回调函数指针
 */
void UIService_RegisterCallback(UIEventCallback_t callback)
{
    s_eventCallback = callback;
}

/**
 * @brief  强制刷新显示
 */
void UIService_ForceDisplayUpdate(void)
{
    s_displayNeedsUpdate = true;
}

/**
 * @brief  显示是否需要更新
 * @retval true=需要更新, false=不需要更新
 */
bool UIService_IsDisplayUpdateNeeded(void)
{
    return s_displayNeedsUpdate;
}

/**
 * @brief  清除显示更新标志
 */
void UIService_ClearDisplayUpdateFlag(void)
{
    s_displayNeedsUpdate = false;
}

/* 私有函数 ----------------------------------------------------------*/

/**
 * @brief  处理编码器输入(内部函数)
 */
static void UIService_ProcessEncoderInternal(void)
{
    // 用于吸收多余的脉冲(解决拧一格动两下的问题)
    static int16_t step_accumulator = 0;

    uint16_t current_counter = Encoder_HAL_GetCounter();

    // 1. 计算差值(利用无符号溢出特性)
    int16_t delta = (int16_t)(current_counter - s_encoderStatus.last_counter);
    s_encoderStatus.last_counter = current_counter;

    // 2. 检查差值,无动作则直接退出,节省CPU
    if (delta == 0) {
        return;
    }

    // 调试输出 - 可以通过SEGGER RTT查看
    // SEGGER_RTT_printf(0, "Encoder: counter=%d, delta=%d, acc=%d\n",
    //                   current_counter, delta, step_accumulator);

    // 3. 应用反向设置标志
    if (Encoder_HAL_IsReversed()) {
        delta = -delta;
    }

    // 4. 将差值放入累加器池子
    step_accumulator += delta;

    // 5. 步数结算(阈值为2)
    if (step_accumulator >= 2) {
        // 正转加5度
        uint16_t current_temp = TempControl_GetTargetTemp();
        TempControl_SetTargetTemp(current_temp + 5);
        step_accumulator -= 2;

        // 调试输出
        // SEGGER_RTT_printf(0, "Temp UP: %d -> %d\n", current_temp, current_temp + 5);

        // 通知手柄移动
        PowerMgmt_NotifyHandleMoved();
        s_displayNeedsUpdate = true;

        // 触发事件回调
        if (s_eventCallback) {
            s_eventCallback(UI_EVENT_TEMP_CHANGED, NULL);
        }
    } else if (step_accumulator <= -2) {
        // 反转减5度
        uint16_t current_temp = TempControl_GetTargetTemp();
        TempControl_SetTargetTemp(current_temp - 5);
        step_accumulator += 2;

        // 调试输出
        // SEGGER_RTT_printf(0, "Temp DOWN: %d -> %d\n", current_temp, current_temp - 5);

        // 通知手柄移动
        PowerMgmt_NotifyHandleMoved();
        s_displayNeedsUpdate = true;

        // 触发事件回调
        if (s_eventCallback) {
            s_eventCallback(UI_EVENT_TEMP_CHANGED, NULL);
        }
    }
}

/**
 * @brief  处理按键输入(内部函数)
 */
static void UIService_ProcessButtonInternal(void)
{
    ButtonEvent_e event = Button_HAL_Scan();
		SystemConfig_t* config = ConfigService_GetConfig();

    switch (event) {
        case BUTTON_EVENT_SHORT_PRESS:
            // 短按:切换强化模式
            Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
            PowerMgmt_SetBoostMode(!PowerMgmt_IsInBoostMode());
            PowerMgmt_NotifyHandleMoved();
            s_displayNeedsUpdate = true;

            // 触发事件回调
            if (s_eventCallback) {
                s_eventCallback(UI_EVENT_SHORT_PRESS, NULL);
            }
            break;

        case BUTTON_EVENT_LONG_PRESS:
            // 长按:切换显示方式
            Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
						config->flags.mainScreenType = !config->flags.mainScreenType;

            // 触发事件回调
            if (s_eventCallback) {
                s_eventCallback(UI_EVENT_LONG_PRESS, NULL);
            }
            break;

        case BUTTON_EVENT_NONE:
        default:
            break;
    }
}

/**
 * @brief  显示管理任务(内部函数)
 */
static void UIService_DisplayManagerTask(void)
{
    switch (s_displayState) {
        case DISPLAY_UPDATE:
            s_displayNeedsUpdate = true;
            s_displayTimestamp = Timer_HAL_GetTick();
            s_displayState = DISPLAY_WAIT;
            break;

        case DISPLAY_WAIT:
            if (Timer_HAL_GetTick() - s_displayTimestamp >= UI_DISPLAY_UPDATE_PERIOD_MS) {
                s_displayState = DISPLAY_UPDATE;
            }
            break;
    }
}

/**
 * @brief  处理编码器输入(公共接口)
 */
void UIService_ProcessEncoder(void)
{
    UIService_ProcessEncoderInternal();
}

/**
 * @brief  处理按键输入(公共接口)
 */
void UIService_ProcessButton(void)
{
    UIService_ProcessButtonInternal();
}

/**
 * @brief  显示主界面
 */
void UIService_DisplayMainScreen(void)
{
    char sprintf_tmp[16] = {0};
    static float cached_tmp75_temp = 0.0f;
    static uint32_t last_tmp75_read = 0;

    // 获取温度控制状态
    TempControlState_t* tempState = TempControl_GetState();
    SystemConfig_t* config = ConfigService_GetConfig();

    // 每500ms更新一次TMP75温度/Vin电压读数
    if (Timer_HAL_GetTick() - last_tmp75_read >= UI_TMP75_READ_PERIOD_MS) {
        cached_tmp75_temp = TMP75_ReadTemp();
				Vin = ADC_HAL_ReadVin();
        last_tmp75_read = Timer_HAL_GetTick();
    }

    u8g2_FirstPage(&s_u8g2);
    u8g2_SetFontMode(&s_u8g2, 1);
    u8g2_SetFontDirection(&s_u8g2, 0);
    do {
        // 顶部状态栏
        u8g2_SetFont(&s_u8g2, u8g2_font_9x15_tr);
        u8g2_DrawStr(&s_u8g2, 0, 10, "SET:");
        sprintf(sprintf_tmp, "%hu", (uint16_t)tempState->setpoint);
        u8g2_DrawStr(&s_u8g2, 40, 10, sprintf_tmp);

        // 状态文字
        if (tempState->currentTemp > 500) {
            strcpy(sprintf_tmp, "ERROR");
        } else if (PowerMgmt_IsInOffMode()) {
            strcpy(sprintf_tmp, "  OFF");
        } else if (PowerMgmt_IsInSleepMode()) {
            strcpy(sprintf_tmp, "SLEEP");
        } else if (PowerMgmt_IsInBoostMode()) {
            strcpy(sprintf_tmp, "BOOST");
        } else if (tempState->isWorky) {
            strcpy(sprintf_tmp, "WORKY");
        } else if (tempState->pidOutput < 180) {
            strcpy(sprintf_tmp, " HEAT");
        } else {
            strcpy(sprintf_tmp, " HOLD");
        }
        u8g2_DrawStr(&s_u8g2, 83, 10, sprintf_tmp);

        // 计算PWM百分比
        uint8_t pwm_percent = (uint8_t)(tempState->pidOutput / 1999.0 * 100.0);
        if (pwm_percent > 100) pwm_percent = 100;

        // 区分不同显示模式
        if (config->flags.mainScreenType) {
            // 详细信息模式
            if (d0) {
                u8g2_DrawStr(&s_u8g2, 52, 62, ".");
            }

            sprintf(sprintf_tmp, "%d%%", pwm_percent);
						if(pwm_percent == 100)
						{
							u8g2_DrawStr(&s_u8g2, 92, 28, sprintf_tmp);
						} else {
							u8g2_DrawStr(&s_u8g2, 102, 28, sprintf_tmp);
						}

            sprintf(sprintf_tmp, "%.1fC", cached_tmp75_temp);
            u8g2_DrawStr(&s_u8g2, 83, 45, sprintf_tmp);

            u8g2_DrawStr(&s_u8g2, 0, 62, "T12");
            sprintf(sprintf_tmp, "%.1fV", (float)Vin * 0.001);
            u8g2_DrawStr(&s_u8g2, 83, 62, sprintf_tmp);

            u8g2_SetFont(&s_u8g2, u8g2_font_freedoomr25_tn);
            if (tempState->currentTemp > 500) {
                u8g2_DrawStr(&s_u8g2, 10, 45, "ERR");
            } else {
                sprintf(sprintf_tmp, "%d", (uint16_t)tempState->currentTemp);
                u8g2_DrawStr(&s_u8g2, 10, 45, sprintf_tmp);
            }
        } else {
            // 大字模式(带进度条)
            u8g2_DrawFrame(&s_u8g2, 0, 16, 5, 48);
            uint8_t bar_height = (pwm_percent * 46) / 100;
            if (bar_height > 0) {
                u8g2_DrawBox(&s_u8g2, 1, 64 - bar_height - 1, 3, bar_height);
            }

            u8g2_SetFont(&s_u8g2, u8g2_font_7Segments_26x42_mn);
            if (tempState->currentTemp > 500) {
                u8g2_DrawStr(&s_u8g2, 21, 60, "ERR");
            } else {
                sprintf(sprintf_tmp, "%d", (uint16_t)tempState->currentTemp);
                if (tempState->currentTemp < 100) {
                    u8g2_DrawStr(&s_u8g2, 38, 60, sprintf_tmp);
                } else {
                    u8g2_DrawStr(&s_u8g2, 21, 60, sprintf_tmp);
                }
            }
        }
    } while (u8g2_NextPage(&s_u8g2));
}
