/**
 ******************************************************************************
 * @file    lm75_hal.h
 * @brief   LM75温度传感器硬件抽象层头文件
 * @note    封装基于软件I2C的LM75/TMP75温度读取操作
 ******************************************************************************
 */

#ifndef __LM75_HAL_H
#define __LM75_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化LM75传感器
 */
void LM75_HAL_Init(void);

/**
 * @brief  读取LM75温度
 * @retval 温度值(°C)，负温度支持
 */
float LM75_HAL_ReadTemp(void);

#ifdef __cplusplus
}
#endif

#endif /* __LM75_HAL_H */
