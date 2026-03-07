/**
 ******************************************************************************
 * @file    app_state_machine.c
 * @brief   应用状态机实现
 ******************************************************************************
 */

#include "app_state_machine.h"
#include "config_service.h"
#include "temp_control.h"
#include "power_mgmt.h"
#include "ui_service.h"
#include "adc_hal.h"
#include "pwm_hal.h"
#include "encoder_hal.h"
#include "button_hal.h"
#include "buzzer_hal.h"
#include "timer_hal.h"

/* 私有变量 ----------------------------------------------------------*/
static AppState_e s_appState = APP_STATE_BOOT;
static AppError_e s_appError = APP_ERROR_NONE;

/* 私有函数声明 ----------------------------------------------------*/
static void AppStateMachine_BootHandler(void);
static void AppStateMachine_RunningHandler(void);
static void AppStateMachine_MenuHandler(void);
static void AppStateMachine_ErrorHandler(void);
static void AppStateMachine_UIEventCallback(UIEvent_e event, void* data);

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化应用状态机
 */
void AppStateMachine_Init(void)
{
    s_appState = APP_STATE_BOOT;
    s_appError = APP_ERROR_NONE;

    // 注册UI事件回调
    UIService_RegisterCallback(AppStateMachine_UIEventCallback);
}

/**
 * @brief  运行应用状态机(需要周期性调用)
 */
void AppStateMachine_Run(void)
{
    switch (s_appState) {
        case APP_STATE_BOOT:
            AppStateMachine_BootHandler();
            break;

        case APP_STATE_RUNNING:
            AppStateMachine_RunningHandler();
            break;

        case APP_STATE_MENU:
            AppStateMachine_MenuHandler();
            break;

        case APP_STATE_ERROR:
            AppStateMachine_ErrorHandler();
            break;

        default:
            s_appState = APP_STATE_ERROR;
            s_appError = APP_ERROR_NONE;
            break;
    }
}

/**
 * @brief  获取当前应用状态
 * @retval 当前应用状态
 */
AppState_e AppStateMachine_GetState(void)
{
    return s_appState;
}

/**
 * @brief  设置应用状态
 * @param  state: 目标状态
 */
void AppStateMachine_SetState(AppState_e state)
{
    s_appState = state;
}

/**
 * @brief  获取错误代码
 * @retval 错误代码
 */
AppError_e AppStateMachine_GetError(void)
{
    return s_appError;
}

/**
 * @brief  设置错误代码
 * @param  error: 错误代码
 */
void AppStateMachine_SetError(AppError_e error)
{
    s_appError = error;
    if (error != APP_ERROR_NONE) {
        s_appState = APP_STATE_ERROR;
    }
}

/* 私有函数 ----------------------------------------------------------*/

/**
 * @brief  启动状态处理函数
 */
static void AppStateMachine_BootHandler(void)
{
    // 1. 初始化配置服务
    ConfigService_Init();

    // 2. 初始化硬件抽象层
    Timer_HAL_Init();
    ADC_HAL_Init();
    PWM_HAL_Init();
    Encoder_HAL_Init();
    Button_HAL_Init();
    Vibration_HAL_Init();
    Buzzer_HAL_Init();

    // 3. 初始化业务逻辑层
    TempControl_Init();
    PowerMgmt_Init();
    UIService_Init();

    // 4. 读取参考电压
    ADC_HAL_ReadVref();

    // 5. 蜂鸣器提示启动完成
    Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
    Timer_HAL_Delay(100);
    Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);

    // 6. 切换到运行状态
    s_appState = APP_STATE_RUNNING;
}

/**
 * @brief  运行状态处理函数
 */
static void AppStateMachine_RunningHandler(void)
{
    // 1. 蜂鸣器任务
    Buzzer_HAL_Task();

    // 2. 温度控制任务
    TempControl_Task();

    // 3. 电源管理任务
    PowerMgmt_Task();

    // 4. UI服务任务
    UIService_Task();

    // 5. 错误检测
    float current_temp = TempControl_GetCurrentTemp();
    if (current_temp > 500) {
        // 温度传感器故障或温度过高
        AppStateMachine_SetError(APP_ERROR_TEMP_SENSOR_FAULT);
    }
}

/**
 * @brief  菜单状态处理函数
 */
static void AppStateMachine_MenuHandler(void)
{
    // TODO: 实现菜单功能
    // 暂时直接返回运行状态
    s_appState = APP_STATE_RUNNING;
}

/**
 * @brief  错误状态处理函数
 */
static void AppStateMachine_ErrorHandler(void)
{
    // 关闭加热器
    PWM_HAL_HeaterOff();

    // 蜂鸣器报警
    static uint32_t last_beep = 0;
    if (Timer_HAL_GetTick() - last_beep >= 1000) {
        Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
        last_beep = Timer_HAL_GetTick();
    }

    // TODO: 显示错误信息
    // TODO: 提供错误恢复机制
}

/**
 * @brief  UI事件回调函数
 * @param  event: UI事件
 * @param  data: 事件数据
 */
static void AppStateMachine_UIEventCallback(UIEvent_e event, void* data)
{
    switch (event) {
        case UI_EVENT_LONG_PRESS:
            // 长按进入菜单
            if (s_appState == APP_STATE_RUNNING) {
                s_appState = APP_STATE_MENU;
            }
            break;

        case UI_EVENT_TEMP_CHANGED:
            // 温度改变,标记需要蜂鸣
            TempControl_GetState()->beepIfWorky = true;
            break;

        default:
            break;
    }
}
