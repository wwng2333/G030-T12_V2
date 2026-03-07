/**
 ******************************************************************************
 * @file    app_state_machine.h
 * @brief   应用状态机头文件
 * @note    封装应用层的状态机逻辑
 ******************************************************************************
 */

#ifndef __APP_STATE_MACHINE_H
#define __APP_STATE_MACHINE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的类型 --------------------------------------------------------*/
typedef enum {
    APP_STATE_BOOT,      // 启动初始化
    APP_STATE_RUNNING,   // 正常运行
    APP_STATE_MENU,      // 菜单设置
    APP_STATE_ERROR      // 错误状态
} AppState_e;

typedef enum {
    APP_ERROR_NONE = 0,
    APP_ERROR_TEMP_SENSOR_FAULT,    // 温度传感器故障
    APP_ERROR_TEMP_TOO_HIGH,        // 温度过高
    APP_ERROR_VOLTAGE_TOO_LOW,      // 电压过低
    APP_ERROR_EEPROM_FAULT          // EEPROM故障
} AppError_e;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化应用状态机
 */
void AppStateMachine_Init(void);

/**
 * @brief  运行应用状态机(需要周期性调用)
 */
void AppStateMachine_Run(void);

/**
 * @brief  获取当前应用状态
 * @retval 当前应用状态
 */
AppState_e AppStateMachine_GetState(void);

/**
 * @brief  设置应用状态
 * @param  state: 目标状态
 */
void AppStateMachine_SetState(AppState_e state);

/**
 * @brief  获取错误代码
 * @retval 错误代码
 */
AppError_e AppStateMachine_GetError(void);

/**
 * @brief  设置错误代码
 * @param  error: 错误代码
 */
void AppStateMachine_SetError(AppError_e error);

#ifdef __cplusplus
}
#endif

#endif /* __APP_STATE_MACHINE_H */
