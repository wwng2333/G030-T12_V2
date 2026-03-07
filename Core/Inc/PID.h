#ifndef __PID_INT_H
#define __PID_INT_H

#include <stdint.h>

// 放大倍数：2的8次方 = 256。
// 将参数放大 256 倍，最后输出时右移 8 位还原，运算速度极快
#define PID_SCALE_SHIFT 8

typedef struct {
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
    
    int32_t integral;   // 历史误差积分
    int32_t last_input; // 上一次的温度采样值
    
    int32_t out_min;    // 输出下限
    int32_t out_max;    // 输出上限
} IntPID_TypeDef;

// 函数声明
void IntPID_Init(IntPID_TypeDef *pid, int32_t kp, int32_t ki, int32_t kd, int32_t out_min, int32_t out_max);
void IntPID_SetTunings(IntPID_TypeDef *pid, float kp, float ki, float kd);
int32_t IntPID_Compute(IntPID_TypeDef *pid, int32_t setpoint, int32_t input);
void IntPID_Reset(IntPID_TypeDef *pid);

#endif // __PID_INT_H