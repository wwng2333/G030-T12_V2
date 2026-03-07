/**
 ******************************************************************************
 * @file    timer_hal.c
 * @brief   定时器硬件抽象层实现
 ******************************************************************************
 */

#include "timer_hal.h"
#include "tim.h"
#include "stm32g0xx_ll_tim.h"

/* 私有变量 ----------------------------------------------------------*/
extern __IO uint32_t TIM16_Tick;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化定时器硬件抽象层
 */
void Timer_HAL_Init(void)
{
    // TIM16已经在MX_TIM16_Init()中初始化
    // 这里可以添加额外的初始化代码
}

/**
 * @brief  获取系统时钟(ms)
 * @retval 系统运行时间(ms)
 */
uint32_t Timer_HAL_GetTick(void)
{
    return TIM16_Tick;
}

/**
 * @brief  延时(ms)
 * @param  ms: 延时时间(ms)
 */
void Timer_HAL_Delay(uint32_t ms)
{
    LL_mDelay(ms);
}
