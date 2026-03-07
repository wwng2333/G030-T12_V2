/**
 ******************************************************************************
 * @file    power_mgmt.h
 * @brief   电源管理服务头文件
 * @note    封装休眠、关机、强化模式等电源管理逻辑
 ******************************************************************************
 */

#ifndef __POWER_MGMT_H
#define __POWER_MGMT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的类型 --------------------------------------------------------*/
typedef enum {
    POWER_MODE_NORMAL,   // 正常工作模式
    POWER_MODE_SLEEP,    // 休眠模式
    POWER_MODE_OFF,      // 关机模式
    POWER_MODE_BOOST     // 强化模式
} PowerMode_e;

typedef struct {
    bool     inSleepMode;
    bool     inOffMode;
    bool     inBoostMode;
    bool     handleMoved;    // 手柄是否移动

    uint32_t sleepMillis;    // 休眠计时起点
    uint32_t boostMillis;    // 强化计时起点
} PowerMgmtState_t;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化电源管理服务
 */
void PowerMgmt_Init(void);

/**
 * @brief  电源管理任务(需要周期性调用)
 */
void PowerMgmt_Task(void);

/**
 * @brief  获取电源管理状态
 * @retval 电源管理状态指针
 */
PowerMgmtState_t* PowerMgmt_GetState(void);

/**
 * @brief  获取当前电源模式
 * @retval 电源模式
 */
PowerMode_e PowerMgmt_GetMode(void);

/**
 * @brief  设置休眠模式
 * @param  enable: true=进入休眠, false=退出休眠
 */
void PowerMgmt_SetSleepMode(bool enable);

/**
 * @brief  设置关机模式
 * @param  enable: true=进入关机, false=退出关机
 */
void PowerMgmt_SetOffMode(bool enable);

/**
 * @brief  设置强化模式
 * @param  enable: true=进入强化, false=退出强化
 */
void PowerMgmt_SetBoostMode(bool enable);

/**
 * @brief  通知手柄移动(用于唤醒)
 */
void PowerMgmt_NotifyHandleMoved(void);

/**
 * @brief  是否处于休眠模式
 * @retval true=休眠, false=非休眠
 */
bool PowerMgmt_IsInSleepMode(void);

/**
 * @brief  是否处于关机模式
 * @retval true=关机, false=非关机
 */
bool PowerMgmt_IsInOffMode(void);

/**
 * @brief  是否处于强化模式
 * @retval true=强化, false=非强化
 */
bool PowerMgmt_IsInBoostMode(void);

/**
 * @brief  手柄最近是否移动
 * @retval true=正在移动, false=不在移动
 */
bool PowerMgmt_HasHandleMoved(void);

#ifdef __cplusplus
}
#endif

#endif /* __POWER_MGMT_H */
