/**
 ******************************************************************************
 * @file    ui_service.h
 * @brief   UI服务头文件
 * @note    封装显示管理、编码器处理、按键处理等UI相关逻辑
 ******************************************************************************
 */

#ifndef __UI_SERVICE_H
#define __UI_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的常量 --------------------------------------------------------*/
#define UI_DISPLAY_UPDATE_PERIOD_MS    50
#define UI_TMP75_READ_PERIOD_MS        500

#define BAR_X          0        // 屏幕左侧起始位置
#define BAR_Y          58       // 屏幕底部起始位置 (假设高度64, 58+5=63)
#define BAR_MAX_W      50       // 进度条总宽度
#define BAR_H          5        // 进度条高度

/* 导出的类型 --------------------------------------------------------*/
typedef enum {
    UI_EVENT_NONE = 0,
    UI_EVENT_TEMP_CHANGED,      // 温度设定值改变
    UI_EVENT_SHORT_PRESS,       // 短按按键
    UI_EVENT_LONG_PRESS,        // 长按按键
    UI_EVENT_DISPLAY_UPDATE     // 显示需要更新
} UIEvent_e;

typedef void (*UIEventCallback_t)(UIEvent_e event, void* data);

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化UI服务
 */
void UIService_Init(void);

/**
 * @brief  UI服务任务(需要周期性调用)
 */
void UIService_Task(void);

/**
 * @brief  注册UI事件回调函数
 * @param  callback: 回调函数指针
 */
void UIService_RegisterCallback(UIEventCallback_t callback);

/**
 * @brief  强制刷新显示
 */
void UIService_ForceDisplayUpdate(void);

/**
 * @brief  显示是否需要更新
 * @retval true=需要更新, false=不需要更新
 */
bool UIService_IsDisplayUpdateNeeded(void);

/**
 * @brief  清除显示更新标志
 */
void UIService_ClearDisplayUpdateFlag(void);

/**
 * @brief  处理编码器输入
 */
void UIService_ProcessEncoder(void);

/**
 * @brief  处理按键输入
 */
void UIService_ProcessButton(void);

/**
 * @brief  显示主界面
 */
void UIService_DisplayMainScreen(void);

#ifdef __cplusplus
}
#endif

#endif /* __UI_SERVICE_H */
