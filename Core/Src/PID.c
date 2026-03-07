#include "pid.h"

#define PID_SCALE_SHIFT 8

void IntPID_Init(IntPID_TypeDef *pid, int32_t kp, int32_t ki, int32_t kd, int32_t out_min, int32_t out_max) {
    IntPID_SetTunings(pid, kp, ki, kd);
    pid->out_min = out_min;
    pid->out_max = out_max;
    IntPID_Reset(pid);
}

void IntPID_Reset(IntPID_TypeDef *pid) {
    pid->integral = 0;
    pid->last_input = 0;
}

void IntPID_SetTunings(IntPID_TypeDef *pid, float kp, float ki, float kd) {
    // 将直观的小数自动放大 256 倍，存为整数
    // 这样计算任务里依然是纯整数加减乘和移位，速度极快
    pid->Kp = (int32_t)(kp * (1 << PID_SCALE_SHIFT));
    pid->Ki = (int32_t)(ki * (1 << PID_SCALE_SHIFT));
    pid->Kd = (int32_t)(kd * (1 << PID_SCALE_SHIFT));
}

int32_t IntPID_Compute(IntPID_TypeDef *pid, int32_t setpoint, int32_t input) {
    int32_t error = setpoint - input;
    
    // 1. 比例项 (P)
    int32_t p_term = pid->Kp * error;
    
    // 2. 微分项 (D) - 微分先行，在积分累加前计算
    int32_t d_term = pid->Kd * (input - pid->last_input);
    pid->last_input = input;
    
    // 3. 积分项 (I) - 核心优化：条件积分 (抗积分饱和)
    // 提前计算当前的理论输出趋势
    int32_t max_scaled = pid->out_max << PID_SCALE_SHIFT;
    int32_t min_scaled = pid->out_min << PID_SCALE_SHIFT;
    int32_t current_trend = p_term + pid->integral - d_term;
    
    // 只有当输出没有达到满载（0-1999），
    // 或者虽然满载，但当前误差的方向能让输出退出饱和时，才允许累加积分
    if ((current_trend < max_scaled && current_trend > min_scaled) ||
        (current_trend >= max_scaled && error < 0) ||
        (current_trend <= min_scaled && error > 0)) {
        pid->integral += (pid->Ki * error);
    }
    
    // 4. 计算最终输出并还原缩放比例
    int32_t output = (p_term + pid->integral - d_term) >> PID_SCALE_SHIFT;
    
    // 5. 最终输出限幅 (安全兜底)
    if (output > pid->out_max) {
        return pid->out_max;
    } else if (output < pid->out_min) {
        return pid->out_min;
    }
    
    return output;
}