/**
 ******************************************************************************
 * @file    buzzer_hal.c
 * @brief   蜂鸣器硬件抽象层实现
 ******************************************************************************
 */

#include "buzzer_hal.h"
#include "timer_hal.h"
#include "tim.h"
#include "stm32g0xx_ll_tim.h"

/* 私有变量 ----------------------------------------------------------*/
static bool s_buzzerEnabled = true;
static uint32_t s_beepTurnOffTick = 0;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化蜂鸣器硬件抽象层
 */
void Buzzer_HAL_Init(void)
{
    // TIM14已经在MX_TIM14_Init()中初始化
    LL_TIM_EnableAllOutputs(TIM14);
    LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableCounter(TIM14);

    s_buzzerEnabled = true;
    s_beepTurnOffTick = 0;
}

/**
 * @brief  蜂鸣器鸣叫
 * @param  duration_ms: 鸣叫持续时间(ms)
 */
void Buzzer_HAL_Beep(uint16_t duration_ms)
{
    if (!s_buzzerEnabled) {
        return;
    }

    LL_TIM_CC_EnableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    s_beepTurnOffTick = Timer_HAL_GetTick() + duration_ms;
}

/**
 * @brief  设置蜂鸣器使能状态
 * @param  enabled: true=使能, false=禁用
 */
void Buzzer_HAL_SetEnabled(bool enabled)
{
    s_buzzerEnabled = enabled;

    if (!enabled) {
        // 立即关闭蜂鸣器
        LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    }
}

/**
 * @brief  获取蜂鸣器使能状态
 * @retval true=使能, false=禁用
 */
bool Buzzer_HAL_IsEnabled(void)
{
    return s_buzzerEnabled;
}

/**
 * @brief  蜂鸣器任务(需要周期性调用,用于自动关闭)
 */
void Buzzer_HAL_Task(void)
{
    // 检查定时器通道是否开启,并且是否到了关闭时间
    if ((TIM14->CCER & TIM_CCER_CC1E) &&
        (Timer_HAL_GetTick() >= s_beepTurnOffTick)) {
        LL_TIM_CC_DisableChannel(TIM14, LL_TIM_CHANNEL_CH1);
    }
}
