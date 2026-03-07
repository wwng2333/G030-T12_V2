/**
 ******************************************************************************
 * @file    power_mgmt.c
 * @brief   电源管理服务实现
 ******************************************************************************
 */

#include "power_mgmt.h"
#include "config_service.h"
#include "timer_hal.h"
#include "buzzer_hal.h"
#include "button_hal.h"

/* 私有变量 ----------------------------------------------------------*/
static PowerMgmtState_t s_powerState = {
    .inSleepMode = false,
    .inOffMode = false,
    .inBoostMode = false,
    .handleMoved = false,
    .sleepMillis = 0,
    .boostMillis = 0
};

static uint32_t s_lastMoveDisplayTick = 0;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化电源管理服务
 */
void PowerMgmt_Init(void)
{
    s_powerState.inSleepMode = false;
    s_powerState.inOffMode = false;
    s_powerState.inBoostMode = false;
    s_powerState.handleMoved = false;
    s_powerState.sleepMillis = Timer_HAL_GetTick();
    s_powerState.boostMillis = 0;
}

/**
 * @brief  电源管理任务(需要周期性调用)
 */
void PowerMgmt_Task(void)
{
    SystemConfig_t* config = ConfigService_GetConfig();
    uint32_t current_tick = Timer_HAL_GetTick();

		// 0. 轮询底层硬件：检查手柄是否移动
    if (Vibration_HAL_Scan()) {
        s_powerState.handleMoved = true;
				s_lastMoveDisplayTick = Timer_HAL_GetTick(); // 记录跳变时间供 UI 使用
    }
	
    // 1. 处理手柄移动事件(唤醒)
    if (s_powerState.handleMoved) {
        if (s_powerState.inSleepMode) {
            Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
        }
        s_powerState.handleMoved = false;
        s_powerState.inSleepMode = false;
        s_powerState.inOffMode = false;
        s_powerState.sleepMillis = current_tick;
    }

    // 2. 检查休眠超时
    uint32_t goneMinutes = (current_tick - s_powerState.sleepMillis) / 60000;
    if (!s_powerState.inSleepMode &&
        config->time2sleep > 0 &&
        goneMinutes >= config->time2sleep) {
        s_powerState.inSleepMode = true;
        Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
    }

    // 3. 检查关机超时
    if (!s_powerState.inOffMode &&
        config->time2off > 0 &&
        goneMinutes >= config->time2off) {
        s_powerState.inOffMode = true;
        Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
    }

    // 4. 处理强化模式倒计时
    if (s_powerState.inBoostMode && config->timeOfBoost > 0) {
        uint32_t goneSeconds = (current_tick - s_powerState.boostMillis) / 1000;
        if (goneSeconds >= config->timeOfBoost) {
            s_powerState.inBoostMode = false;
            Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
        }
    }
}

/**
 * @brief  获取电源管理状态
 * @retval 电源管理状态指针
 */
PowerMgmtState_t* PowerMgmt_GetState(void)
{
    return &s_powerState;
}

/**
 * @brief  获取当前电源模式
 * @retval 电源模式
 */
PowerMode_e PowerMgmt_GetMode(void)
{
    if (s_powerState.inOffMode) {
        return POWER_MODE_OFF;
    } else if (s_powerState.inSleepMode) {
        return POWER_MODE_SLEEP;
    } else if (s_powerState.inBoostMode) {
        return POWER_MODE_BOOST;
    } else {
        return POWER_MODE_NORMAL;
    }
}

/**
 * @brief  设置休眠模式
 * @param  enable: true=进入休眠, false=退出休眠
 */
void PowerMgmt_SetSleepMode(bool enable)
{
    s_powerState.inSleepMode = enable;
}

/**
 * @brief  设置关机模式
 * @param  enable: true=进入关机, false=退出关机
 */
void PowerMgmt_SetOffMode(bool enable)
{
    s_powerState.inOffMode = enable;
}

/**
 * @brief  设置强化模式
 * @param  enable: true=进入强化, false=退出强化
 */
void PowerMgmt_SetBoostMode(bool enable)
{
    s_powerState.inBoostMode = enable;
    if (enable) {
        s_powerState.boostMillis = Timer_HAL_GetTick();
    }
}

/**
 * @brief  通知手柄移动(用于唤醒)
 */
void PowerMgmt_NotifyHandleMoved(void)
{
    s_powerState.handleMoved = true;
}

/**
 * @brief  是否处于休眠模式
 * @retval true=休眠, false=非休眠
 */
bool PowerMgmt_IsInSleepMode(void)
{
    return s_powerState.inSleepMode;
}

/**
 * @brief  是否处于关机模式
 * @retval true=关机, false=非关机
 */
bool PowerMgmt_IsInOffMode(void)
{
    return s_powerState.inOffMode;
}

/**
 * @brief  是否处于强化模式
 * @retval true=强化, false=非强化
 */
bool PowerMgmt_IsInBoostMode(void)
{
    return s_powerState.inBoostMode;
}

/**
 * @brief  手柄最近是否移动
 * @retval true=正在移动, false=不在移动
 */
bool PowerMgmt_HasHandleMoved(void)
{
    // 如果距离上次移动不到 200ms，告诉 UI "正在移动"
    if (Timer_HAL_GetTick() - s_lastMoveDisplayTick < 200) {
        return true;
    }
    return false;
}