/**
 ******************************************************************************
 * @file    timer_hal.h
 * @brief   定时器硬件抽象层头文件
 * @note    封装系统时钟相关的硬件操作
 ******************************************************************************
 */

#ifndef __TIMER_HAL_H
#define __TIMER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化定时器硬件抽象层
 */
void Timer_HAL_Init(void);

/**
 * @brief  获取系统时钟(ms)
 * @retval 系统运行时间(ms)
 */
uint32_t Timer_HAL_GetTick(void);

/**
 * @brief  延时(ms)
 * @param  ms: 延时时间(ms)
 */
void Timer_HAL_Delay(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* __TIMER_HAL_H */
