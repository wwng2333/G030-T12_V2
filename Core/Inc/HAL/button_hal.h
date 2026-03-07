/**
 ******************************************************************************
 * @file    button_hal.h
 * @brief   按键硬件抽象层头文件
 * @note    封装按键检测相关的硬件操作
 ******************************************************************************
 */

#ifndef __BUTTON_HAL_H
#define __BUTTON_HAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的类型 --------------------------------------------------------*/
typedef enum {
    BUTTON_EVENT_NONE = 0,
    BUTTON_EVENT_SHORT_PRESS,
    BUTTON_EVENT_LONG_PRESS
} ButtonEvent_e;

/* 导出的常量 --------------------------------------------------------*/
#define BUTTON_LONG_PRESS_TIME_MS    500

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化按键硬件抽象层
 */
void Button_HAL_Init(void);

/**
 * @brief  读取按键原始状态
 * @retval true=按下, false=释放
 */
bool Button_HAL_IsPressed(void);

/**
 * @brief  按键扫描(需要周期性调用)
 * @retval 按键事件
 */
ButtonEvent_e Button_HAL_Scan(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUTTON_HAL_H */
