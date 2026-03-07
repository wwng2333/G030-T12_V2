/**
 ******************************************************************************
 * @file    encoder_hal.h
 * @brief   编码器硬件抽象层头文件
 * @note    封装旋转编码器相关的硬件操作
 ******************************************************************************
 */

#ifndef __ENCODER_HAL_H
#define __ENCODER_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化编码器硬件抽象层
 */
void Encoder_HAL_Init(void);

/**
 * @brief  读取编码器计数值
 * @retval 当前计数值
 */
uint16_t Encoder_HAL_GetCounter(void);

/**
 * @brief  设置编码器反向标志
 * @param  reversed: true=反向, false=正向
 */
void Encoder_HAL_SetReversed(bool reversed);

/**
 * @brief  获取编码器反向标志
 * @retval true=反向, false=正向
 */
bool Encoder_HAL_IsReversed(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_HAL_H */
