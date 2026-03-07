/**
 ******************************************************************************
 * @file    config_service.h
 * @brief   配置管理服务头文件
 * @note    封装系统配置的读写和管理
 ******************************************************************************
 */

#ifndef __CONFIG_SERVICE_H
#define __CONFIG_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "main.h"

/* 导出的类型 --------------------------------------------------------*/
// 使用main.h中定义的SystemConfig_t

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化配置服务
 */
void ConfigService_Init(void);

/**
 * @brief  获取系统配置
 * @retval 系统配置指针
 */
SystemConfig_t* ConfigService_GetConfig(void);

/**
 * @brief  保存配置到EEPROM
 * @retval true=成功, false=失败
 */
bool ConfigService_SaveToEEPROM(void);

/**
 * @brief  从EEPROM加载配置
 * @retval true=成功, false=失败
 */
bool ConfigService_LoadFromEEPROM(void);

/**
 * @brief  恢复出厂设置
 */
void ConfigService_RestoreDefaults(void);

/**
 * @brief  获取默认温度
 * @retval 默认温度(°C)
 */
uint16_t ConfigService_GetDefaultTemp(void);

/**
 * @brief  获取休眠温度
 * @retval 休眠温度(°C)
 */
uint16_t ConfigService_GetSleepTemp(void);

/**
 * @brief  获取强化温度增量
 * @retval 强化温度增量(°C)
 */
uint16_t ConfigService_GetBoostTemp(void);

/**
 * @brief  获取休眠时间
 * @retval 休眠时间(分钟)
 */
uint16_t ConfigService_GetTime2Sleep(void);

/**
 * @brief  获取关机时间
 * @retval 关机时间(分钟)
 */
uint16_t ConfigService_GetTime2Off(void);

/**
 * @brief  获取强化模式持续时间
 * @retval 强化模式持续时间(秒)
 */
uint16_t ConfigService_GetTimeOfBoost(void);

/**
 * @brief  蜂鸣器是否使能
 * @retval true=使能, false=禁用
 */
bool ConfigService_IsBuzzerEnabled(void);

/**
 * @brief  屏幕是否翻转
 * @retval true=翻转, false=正常
 */
bool ConfigService_IsScreenFlipped(void);

/**
 * @brief  编码器是否反向
 * @retval true=反向, false=正向
 */
bool ConfigService_IsEncoderReversed(void);

/**
 * @brief  获取主界面类型
 * @retval 0=大字模式, 1=详细信息模式
 */
uint8_t ConfigService_GetMainScreenType(void);

/**
 * @brief  获取控制类型
 * @retval 0=直接控制, 1=PID控制
 */
uint8_t ConfigService_GetControlType(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONFIG_SERVICE_H */
