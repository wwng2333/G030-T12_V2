/**
 ******************************************************************************
 * @file    adc_hal.h
 * @brief   ADC硬件抽象层头文件
 * @note    封装ADC采样相关的硬件操作
 ******************************************************************************
 */

#ifndef __ADC_HAL_H
#define __ADC_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化ADC硬件抽象层
 */
void ADC_HAL_Init(void);

/**
 * @brief  读取温度传感器ADC值(带降噪)
 * @retval 12位ADC值
 */
uint16_t ADC_HAL_ReadTemperature(void);

/**
 * @brief  读取参考电压
 * @retval 参考电压(mV)
 */
uint16_t ADC_HAL_ReadVref(void);

/**
 * @brief  读取输入电压
 * @retval 输入电压(mV)
 */
uint16_t ADC_HAL_ReadVin(void);

/**
 * @brief  读取MCU内部温度
 * @retval MCU温度(°C)
 */
uint16_t ADC_HAL_ReadMcuTemp(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_HAL_H */
