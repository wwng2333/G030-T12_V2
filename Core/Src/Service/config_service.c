/**
 ******************************************************************************
 * @file    config_service.c
 * @brief   配置管理服务实现
 ******************************************************************************
 */

#include "config_service.h"
//#include "Flash.h"
#include <string.h>

/* 私有变量 ----------------------------------------------------------*/
static SystemConfig_t s_sysConfig = {
    .defaultTemp = 300,        // 默认目标温度 300°C
    .sleepTemp = 200,          // 休眠温度 200°C
    .boostTemp = 50,           // 强化模式提升 50°C
    .time2sleep = 5,           // 5 分钟后休眠
    .time2off = 15,            // 15 分钟后关机
    .timeOfBoost = 60,         // 强化模式持续 60 秒
    .flags.mainScreenType = 1, // 默认大字丰富界面
    .flags.buzzerEnabled = 1,  // 默认开启蜂鸣器
    .flags.controlType = 1,    // 默认开启 PID 控制
    .currentTip = 0,
    .numberOfTips = 1
};

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化配置服务
 */
void ConfigService_Init(void)
{
    // 尝试从EEPROM加载配置
    // if (!ConfigService_LoadFromEEPROM()) {
    //     // 加载失败,使用默认配置
    //     ConfigService_RestoreDefaults();
    // }
}

/**
 * @brief  获取系统配置
 * @retval 系统配置指针
 */
SystemConfig_t* ConfigService_GetConfig(void)
{
    return &s_sysConfig;
}

/**
 * @brief  保存配置到EEPROM
 * @retval true=成功, false=失败
 */
bool ConfigService_SaveToEEPROM(void)
{
    // TODO: 实现EEPROM写入
    return false;
}

/**
 * @brief  从EEPROM加载配置
 * @retval true=成功, false=失败
 */
bool ConfigService_LoadFromEEPROM(void)
{
    // TODO: 实现EEPROM读取
    return false;
}

/**
 * @brief  恢复出厂设置
 */
void ConfigService_RestoreDefaults(void)
{
    s_sysConfig.defaultTemp = 300;
    s_sysConfig.sleepTemp = 200;
    s_sysConfig.boostTemp = 50;
    s_sysConfig.time2sleep = 5;
    s_sysConfig.time2off = 15;
    s_sysConfig.timeOfBoost = 60;
    s_sysConfig.flags.mainScreenType = 1;
    s_sysConfig.flags.buzzerEnabled = 1;
    s_sysConfig.flags.controlType = 1;
    s_sysConfig.currentTip = 0;
    s_sysConfig.numberOfTips = 1;
}

/**
 * @brief  获取默认温度
 * @retval 默认温度(°C)
 */
uint16_t ConfigService_GetDefaultTemp(void)
{
    return s_sysConfig.defaultTemp;
}

/**
 * @brief  获取休眠温度
 * @retval 休眠温度(°C)
 */
uint16_t ConfigService_GetSleepTemp(void)
{
    return s_sysConfig.sleepTemp;
}

/**
 * @brief  获取强化温度增量
 * @retval 强化温度增量(°C)
 */
uint16_t ConfigService_GetBoostTemp(void)
{
    return s_sysConfig.boostTemp;
}

/**
 * @brief  获取休眠时间
 * @retval 休眠时间(分钟)
 */
uint16_t ConfigService_GetTime2Sleep(void)
{
    return s_sysConfig.time2sleep;
}

/**
 * @brief  获取关机时间
 * @retval 关机时间(分钟)
 */
uint16_t ConfigService_GetTime2Off(void)
{
    return s_sysConfig.time2off;
}

/**
 * @brief  获取强化模式持续时间
 * @retval 强化模式持续时间(秒)
 */
uint16_t ConfigService_GetTimeOfBoost(void)
{
    return s_sysConfig.timeOfBoost;
}

/**
 * @brief  蜂鸣器是否使能
 * @retval true=使能, false=禁用
 */
bool ConfigService_IsBuzzerEnabled(void)
{
    return s_sysConfig.flags.buzzerEnabled;
}

/**
 * @brief  屏幕是否翻转
 * @retval true=翻转, false=正常
 */
bool ConfigService_IsScreenFlipped(void)
{
    return s_sysConfig.flags.screenFlipped;
}

/**
 * @brief  编码器是否反向
 * @retval true=反向, false=正向
 */
bool ConfigService_IsEncoderReversed(void)
{
    return s_sysConfig.flags.encoderReversed;
}

/**
 * @brief  获取主界面类型
 * @retval 0=大字模式, 1=详细信息模式
 */
uint8_t ConfigService_GetMainScreenType(void)
{
    return s_sysConfig.flags.mainScreenType;
}

/**
 * @brief  获取控制类型
 * @retval 0=直接控制, 1=PID控制
 */
uint8_t ConfigService_GetControlType(void)
{
    return s_sysConfig.flags.controlType;
}
