/**
 ******************************************************************************
 * @file    pwm_hal.h
 * @brief   PWM硬件抽象层头文件
 * @note    封装PWM控制相关的硬件操作
 ******************************************************************************
 */

#ifndef __PWM_HAL_H
#define __PWM_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的常量 --------------------------------------------------------*/
#define PWM_HAL_MAX_DUTY    1999    // PWM最大占空比值

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化PWM硬件抽象层
 */
void PWM_HAL_Init(void);

/**
 * @brief  设置加热器PWM占空比
 * @param  duty: 占空比值 (0 - PWM_HAL_MAX_DUTY)
 */
void PWM_HAL_SetHeaterDuty(uint16_t duty);

/**
 * @brief  获取当前加热器PWM占空比
 * @retval 当前占空比值
 */
uint16_t PWM_HAL_GetHeaterDuty(void);

/**
 * @brief  关闭加热器
 */
void PWM_HAL_HeaterOff(void);

#ifdef __cplusplus
}
#endif

#endif /* __PWM_HAL_H */
