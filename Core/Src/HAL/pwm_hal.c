/**
 ******************************************************************************
 * @file    pwm_hal.c
 * @brief   PWM硬件抽象层实现
 ******************************************************************************
 */

#include "pwm_hal.h"
#include "tim.h"
#include "stm32g0xx_ll_tim.h"

/* 私有变量 ----------------------------------------------------------*/
static uint16_t s_currentDuty = 0;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化PWM硬件抽象层
 */
void PWM_HAL_Init(void)
{
    // TIM3已经在MX_TIM3_Init()中初始化
    LL_TIM_EnableAllOutputs(TIM3);
    LL_TIM_CC_EnableChannel(TIM3, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_SetCompareCH1(TIM3, 0);
    LL_TIM_EnableCounter(TIM3);

    s_currentDuty = 0;
}

/**
 * @brief  设置加热器PWM占空比
 * @param  duty: 占空比值 (0 - PWM_HAL_MAX_DUTY)
 */
void PWM_HAL_SetHeaterDuty(uint16_t duty)
{
    if (duty > PWM_HAL_MAX_DUTY) {
        duty = PWM_HAL_MAX_DUTY;
    }

    LL_TIM_OC_SetCompareCH1(TIM3, duty);
    s_currentDuty = duty;
}

/**
 * @brief  获取当前加热器PWM占空比
 * @retval 当前占空比值
 */
uint16_t PWM_HAL_GetHeaterDuty(void)
{
    return s_currentDuty;
}

/**
 * @brief  关闭加热器
 */
void PWM_HAL_HeaterOff(void)
{
    PWM_HAL_SetHeaterDuty(0);
}
