/**
 ******************************************************************************
 * @file    buzzer_hal.h
 * @brief   蜂鸣器硬件抽象层头文件
 * @note    封装蜂鸣器控制相关的硬件操作
 ******************************************************************************
 */

#ifndef __BUZZER_HAL_H
#define __BUZZER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的常量 --------------------------------------------------------*/
#define BUZZER_DEFAULT_DURATION_MS    32

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化蜂鸣器硬件抽象层
 */
void Buzzer_HAL_Init(void);

/**
 * @brief  蜂鸣器鸣叫
 * @param  duration_ms: 鸣叫持续时间(ms)
 */
void Buzzer_HAL_Beep(uint16_t duration_ms);

/**
 * @brief  设置蜂鸣器使能状态
 * @param  enabled: true=使能, false=禁用
 */
void Buzzer_HAL_SetEnabled(bool enabled);

/**
 * @brief  获取蜂鸣器使能状态
 * @retval true=使能, false=禁用
 */
bool Buzzer_HAL_IsEnabled(void);

/**
 * @brief  蜂鸣器任务(需要周期性调用,用于自动关闭)
 */
void Buzzer_HAL_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_HAL_H */
