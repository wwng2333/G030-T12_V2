/**
 ******************************************************************************
 * @file    encoder_hal.c
 * @brief   编码器硬件抽象层实现
 ******************************************************************************
 */

#include "encoder_hal.h"
#include "config_service.h"
#include "tim.h"
#include "stm32g0xx_ll_tim.h"

/* 私有变量 ----------------------------------------------------------*/
static bool s_encoderReversed = false;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化编码器硬件抽象层
 */
void Encoder_HAL_Init(void)
{
    // TIM1已经在MX_TIM1_Init()中初始化为编码器模式
    // 这里只需要启动编码器

    // 启动编码器通道
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

    // 清零计数器
    LL_TIM_SetCounter(TIM1, 0);

    // 启动计数器
    LL_TIM_EnableCounter(TIM1);

    // 从配置中读取编码器反向设置
    SystemConfig_t* config = ConfigService_GetConfig();
    s_encoderReversed = config->flags.encoderReversed;
}

/**
 * @brief  读取编码器计数值
 * @retval 当前计数值
 */
uint16_t Encoder_HAL_GetCounter(void)
{
    return LL_TIM_GetCounter(TIM1);
}

/**
 * @brief  设置编码器反向标志
 * @param  reversed: true=反向, false=正向
 */
void Encoder_HAL_SetReversed(bool reversed)
{
    s_encoderReversed = reversed;
}

/**
 * @brief  获取编码器反向标志
 * @retval true=反向, false=正向
 */
bool Encoder_HAL_IsReversed(void)
{
    return s_encoderReversed;
}
