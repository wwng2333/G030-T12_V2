/**
 ******************************************************************************
 * @file    temp_control.h
 * @brief   温度控制服务头文件
 * @note    封装温度控制相关的业务逻辑
 ******************************************************************************
 */

#ifndef __TEMP_CONTROL_H
#define __TEMP_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 导出的常量 --------------------------------------------------------*/
#define TEMP_CTRL_HEATING_TIME_MS    95   // 加热时间
#define TEMP_CTRL_SETTLE_TIME_MS     3    // 稳定时间
#define TEMP_CTRL_CYCLE_TIME_MS      100  // 控制周期

#define TEMP_SMOOTHIE                0.05f // 温度平滑系数

/* 导出的类型 --------------------------------------------------------*/
typedef struct {
    float    currentTemp;    // 实际温度
    float    rawTemp;        // ADC平滑后的原始值
    uint16_t setTemp;        // 旋钮设定的基础目标温度
    double   setpoint;       // 实际PID目标温度(含休眠/强化)

    double   pidInput;       // PID输入
    double   pidOutput;      // PID输出(PWM值)
    double   pidGap;         // 温度差值

    bool     isWorky;        // 是否达到工作温度
    bool     beepIfWorky;    // 达到温度后是否需要蜂鸣
} TempControlState_t;

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化温度控制服务
 */
void TempControl_Init(void);

/**
 * @brief  温度控制任务(需要周期性调用)
 */
void TempControl_Task(void);

/**
 * @brief  获取温度控制状态
 * @retval 温度控制状态指针
 */
TempControlState_t* TempControl_GetState(void);

/**
 * @brief  设置目标温度
 * @param  temp: 目标温度(°C)
 */
void TempControl_SetTargetTemp(uint16_t temp);

/**
 * @brief  获取目标温度
 * @retval 目标温度(°C)
 */
uint16_t TempControl_GetTargetTemp(void);

/**
 * @brief  获取当前温度
 * @retval 当前温度(°C)
 */
float TempControl_GetCurrentTemp(void);

/**
 * @brief  设置PID目标温度(用于休眠/强化模式)
 * @param  setpoint: PID目标温度(°C)
 */
void TempControl_SetPIDSetpoint(double setpoint);

/**
 * @brief  获取PID输出值
 * @retval PID输出值(0-1999)
 */
uint16_t TempControl_GetPIDOutput(void);

/**
 * @brief  温度是否达到工作状态
 * @retval true=已达到, false=未达到
 */
bool TempControl_IsWorky(void);

#ifdef __cplusplus
}
#endif

#endif /* __TEMP_CONTROL_H */
