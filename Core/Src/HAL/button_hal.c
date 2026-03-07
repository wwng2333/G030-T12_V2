/**
 ******************************************************************************
 * @file    button_hal.c
 * @brief   按键硬件抽象层实现
 ******************************************************************************
 */

#include "button_hal.h"
#include "timer_hal.h"
#include "gpio.h"
#include "stm32g0xx_ll_gpio.h"

/* 私有类型 ----------------------------------------------------------*/
typedef enum {
    BTN_STATE_RELEASED,
    BTN_STATE_PRESSED,
    BTN_STATE_LONG_PRESSED
} ButtonState_e;

/* 私有变量 ----------------------------------------------------------*/
static ButtonState_e s_buttonState = BTN_STATE_RELEASED;
static uint32_t s_buttonPressTimestamp = 0;
static uint8_t s_lastVibeState = 1;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化按键硬件抽象层
 */
void Button_HAL_Init(void)
{
    // GPIO已经在MX_GPIO_Init()中初始化
    s_buttonState = BTN_STATE_RELEASED;
    s_buttonPressTimestamp = 0;
}

/**
 * @brief  初始化振动传感器(滚珠开关) HAL
 */
void Vibration_HAL_Init(void)
{
    // 初始化时读取一次 PA11 初始状态
    s_lastVibeState = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11);
}

/**
 * @brief  扫描振动传感器状态 (纯硬件逻辑)
 * @retval true=检测到振动(状态跳变), false=无振动
 */
bool Vibration_HAL_Scan(void)
{
    bool has_moved = false;
    uint8_t current_state = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_11);
    
    // 如果电平发生翻转，说明滚珠发生了移动
    if (current_state != s_lastVibeState) {
        has_moved = true;
        s_lastVibeState = current_state;
    }
    
    return has_moved;
}

/**
 * @brief  读取按键原始状态
 * @retval true=按下, false=释放
 */
bool Button_HAL_IsPressed(void)
{
    // PA0按键,低电平有效
    return !LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_0);
}

/**
 * @brief  按键扫描(需要周期性调用)
 * @retval 按键事件
 */
ButtonEvent_e Button_HAL_Scan(void)
{	
    ButtonEvent_e event = BUTTON_EVENT_NONE;
    bool is_pressed = Button_HAL_IsPressed();
    uint32_t current_tick = Timer_HAL_GetTick();

    switch (s_buttonState) {
        case BTN_STATE_RELEASED:
            if (is_pressed) {
                s_buttonPressTimestamp = current_tick;
                s_buttonState = BTN_STATE_PRESSED;
            }
            break;

        case BTN_STATE_PRESSED:
            if (!is_pressed) {
                // 短按事件
                event = BUTTON_EVENT_SHORT_PRESS;
                s_buttonState = BTN_STATE_RELEASED;
            } else if (current_tick - s_buttonPressTimestamp >= BUTTON_LONG_PRESS_TIME_MS) {
                // 长按事件
                event = BUTTON_EVENT_LONG_PRESS;
                s_buttonState = BTN_STATE_LONG_PRESSED;
            }
            break;

        case BTN_STATE_LONG_PRESSED:
            if (!is_pressed) {
                s_buttonState = BTN_STATE_RELEASED;
            }
            break;
    }

    return event;
}
