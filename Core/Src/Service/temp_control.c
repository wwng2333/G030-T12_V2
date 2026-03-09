/**
 ******************************************************************************
 * @file    temp_control.c
 * @brief   温度控制服务实现
 ******************************************************************************
 */

#include "temp_control.h"
#include "config_service.h"
#include "power_mgmt.h"
#include "adc_hal.h"
#include "pwm_hal.h"
#include "timer_hal.h"
#include "buzzer_hal.h"
#include "lm75_hal.h"
#include "PID.h"
#include "main.h"
#include <math.h>

/* 私有类型 ----------------------------------------------------------*/
typedef enum {
    CTRL_HEATING,         // 加热周期 (0 - 95ms)
    CTRL_WAIT_SETTLE,     // 断电后等待电压稳定 (95 - 98ms)
    CTRL_MEASURE_AND_PID  // 采样并执行PID控制 (98 - 100ms)
} TempControlPhase_e;

/* 私有变量 ----------------------------------------------------------*/
static TempControlState_t s_tempState = {
    .currentTemp = 0.0f,
    .rawTemp = 0.0f,
    .setTemp = TEMP_DEFAULT,
    .setpoint = TEMP_DEFAULT,
    .pidInput = 0.0,
    .pidOutput = 0.0,
    .pidGap = 0.0,
    .isWorky = false,
    .beepIfWorky = false
};

static TempControlPhase_e s_controlPhase = CTRL_HEATING;
static uint32_t s_lastCycleTick = 0;
static uint32_t s_offTick = 0;

// 冷端温度补偿
#define COLD_JUNCTION_CAL_TEMP   21.0f   // 校准时的环境温度(°C)
#define COLD_JUNCTION_UPDATE_CYC 10      // 每N个控制周期更新一次冷端温度
static float  s_coldJunctionTemp = COLD_JUNCTION_CAL_TEMP;
static uint8_t s_coldJunctionCnt = 0;

static IntPID_TypeDef s_TPID;

// PID参数
static const double s_aggKp = 20;
static const double s_aggKi = 0.5;
static const double s_aggKd = 4;
static const double s_consKp = 8;
static const double s_consKi = 1;
static const double s_consKd = 8;

// 温度校准数据
static uint16_t s_CalTemp[TIPMAX][4] = {
    {TEMP1212, TEMP1696, TEMP2181, TEMPCHP}
};

/* 私有函数声明 ----------------------------------------------------*/
static void TempControl_CalculateTemp(void);
static uint16_t TempControl_MapValue(uint16_t x, uint16_t in_min, uint16_t in_max,
                                      uint16_t out_min, uint16_t out_max);

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化温度控制服务
 */
void TempControl_Init(void)
{
    SystemConfig_t* config = ConfigService_GetConfig();

    // 初始化温度状态
    s_tempState.setTemp = config->defaultTemp;
    s_tempState.setpoint = config->defaultTemp;
    s_tempState.pidOutput = 0;
    s_tempState.isWorky = false;
    s_tempState.beepIfWorky = false;

    // 初始化PID控制器
    IntPID_Init(&s_TPID, s_aggKp, s_aggKi, s_aggKd, 0, PWM_HAL_MAX_DUTY);

    // 初始化温度读取
    LM75_HAL_Init();
    s_coldJunctionTemp = LM75_HAL_ReadTemp();
    PWM_HAL_HeaterOff();
    Timer_HAL_Delay(2);
    s_tempState.rawTemp = ADC_HAL_ReadTemperature();
    TempControl_CalculateTemp();

    s_controlPhase = CTRL_HEATING;
    s_lastCycleTick = Timer_HAL_GetTick();
}

/**
 * @brief  温度控制任务(需要周期性调用)
 */
void TempControl_Task(void)
{
    uint32_t current_tick = Timer_HAL_GetTick();
    SystemConfig_t* config = ConfigService_GetConfig();

    // 防止开机或从菜单退出后,时间差过大导致时序错乱
    if (current_tick - s_lastCycleTick > TEMP_CTRL_CYCLE_TIME_MS * 2) {
        s_lastCycleTick = current_tick;
        s_controlPhase = CTRL_HEATING;
        PWM_HAL_SetHeaterDuty((uint16_t)s_tempState.pidOutput);
    }

    switch (s_controlPhase) {
        case CTRL_HEATING:
            // 加热阶段持续 95ms
            if (current_tick - s_lastCycleTick >= TEMP_CTRL_HEATING_TIME_MS) {
                PWM_HAL_HeaterOff();  // 必须切断加热才能准确测温
                s_offTick = current_tick;
                s_controlPhase = CTRL_WAIT_SETTLE;
            }
            break;

        case CTRL_WAIT_SETTLE:
            // 等待 3ms,让热电偶/运放的电压彻底稳定
            if (current_tick - s_offTick >= TEMP_CTRL_SETTLE_TIME_MS) {
                s_controlPhase = CTRL_MEASURE_AND_PID;
            }
            break;

        case CTRL_MEASURE_AND_PID:
            // 1. 低功耗采样与平滑滤波
            s_tempState.rawTemp += (ADC_HAL_ReadTemperature() - s_tempState.rawTemp) * TEMP_SMOOTHIE;

            // 2. 定期更新冷端温度(每N个周期读取一次LM75,冷端变化缓慢无需每次读取)
            if (++s_coldJunctionCnt >= COLD_JUNCTION_UPDATE_CYC) {
                s_coldJunctionTemp = LM75_HAL_ReadTemp();
                s_coldJunctionCnt = 0;
            }

            TempControl_CalculateTemp();

            // 2. 确定当前工作模式下的目标温度
            if (PowerMgmt_IsInOffMode()) {
                s_tempState.setpoint = 0;
            } else if (PowerMgmt_IsInSleepMode()) {
                s_tempState.setpoint = config->sleepTemp;
            } else if (PowerMgmt_IsInBoostMode()) {
                s_tempState.setpoint = s_tempState.setTemp + config->boostTemp;
            } else {
                s_tempState.setpoint = s_tempState.setTemp;
            }

            // 3. 执行 PID 计算
            s_tempState.pidInput = s_tempState.currentTemp;
            s_tempState.pidGap = fabs(s_tempState.setpoint - s_tempState.currentTemp);

            // 动态 PID 参数分配:温差大时满血加热,接近时保守逼近
            if (s_tempState.pidGap < 10) {
                IntPID_SetTunings(&s_TPID, s_consKp, s_consKi, s_consKd);
            } else {
                IntPID_SetTunings(&s_TPID, s_aggKp, s_aggKi, s_aggKd);
            }
            s_tempState.pidOutput = IntPID_Compute(&s_TPID,
                                                   (int32_t)s_tempState.setpoint,
                                                   (int32_t)s_tempState.currentTemp);

            // 4. 工作状态判定与蜂鸣器逻辑
            if (s_tempState.pidGap < 5) {
                if (!s_tempState.isWorky && s_tempState.beepIfWorky) {
                    Buzzer_HAL_Beep(BUZZER_DEFAULT_DURATION_MS);
                }
                s_tempState.isWorky = true;
                s_tempState.beepIfWorky = false;
            } else {
                s_tempState.isWorky = false;
            }

            // 5. 立即重新开启加热,装载新的 PWM 输出值
            PWM_HAL_SetHeaterDuty((uint16_t)s_tempState.pidOutput);

            // 6. 开启下一个完整的 100ms 周期
            s_lastCycleTick = current_tick;
            s_controlPhase = CTRL_HEATING;
            break;
    }
}

/**
 * @brief  获取温度控制状态
 * @retval 温度控制状态指针
 */
TempControlState_t* TempControl_GetState(void)
{
    return &s_tempState;
}

/**
 * @brief  设置目标温度
 * @param  temp: 目标温度(°C)
 */
void TempControl_SetTargetTemp(uint16_t temp)
{
    // 温度边界限制
    if (temp < TEMP_MIN) {
        temp = TEMP_MIN;
    } else if (temp > TEMP_MAX) {
        temp = TEMP_MAX;
    }

    s_tempState.setTemp = temp;
}

/**
 * @brief  获取目标温度
 * @retval 目标温度(°C)
 */
uint16_t TempControl_GetTargetTemp(void)
{
    return s_tempState.setTemp;
}

/**
 * @brief  获取当前温度
 * @retval 当前温度(°C)
 */
float TempControl_GetCurrentTemp(void)
{
    return s_tempState.currentTemp;
}

/**
 * @brief  设置PID目标温度(用于休眠/强化模式)
 * @param  setpoint: PID目标温度(°C)
 */
void TempControl_SetPIDSetpoint(double setpoint)
{
    s_tempState.setpoint = setpoint;
}

/**
 * @brief  获取PID输出值
 * @retval PID输出值(0-1999)
 */
uint16_t TempControl_GetPIDOutput(void)
{
    return (uint16_t)s_tempState.pidOutput;
}

/**
 * @brief  温度是否达到工作状态
 * @retval true=已达到, false=未达到
 */
bool TempControl_IsWorky(void)
{
    return s_tempState.isWorky;
}

/* 私有函数 ----------------------------------------------------------*/

/**
 * @brief  计算实际温度值
 */
static void TempControl_CalculateTemp(void)
{
    SystemConfig_t* config = ConfigService_GetConfig();
    uint16_t rawTemp = (uint16_t)s_tempState.rawTemp;
    uint8_t tipIndex = config->currentTip;

    if (rawTemp < 1212) {
        s_tempState.currentTemp = TempControl_MapValue(rawTemp, 0, 1212,
                                                       21, s_CalTemp[tipIndex][0]);
    } else if (rawTemp < 1696) {
        s_tempState.currentTemp = TempControl_MapValue(rawTemp, 1212, 1696,
                                                       s_CalTemp[tipIndex][0],
                                                       s_CalTemp[tipIndex][1]);
    } else if (rawTemp < 2181) {
        s_tempState.currentTemp = TempControl_MapValue(rawTemp, 1696, 2181,
                                                       s_CalTemp[tipIndex][1],
                                                       s_CalTemp[tipIndex][2]);
    } else {
        // 超出范围时使用外推
        s_tempState.currentTemp = s_CalTemp[tipIndex][2] + (rawTemp - 2181) * 0.2f;
    }

    // 冷端温度补偿:加上实际冷端温度与校准基准温度的差值
    s_tempState.currentTemp += (s_coldJunctionTemp - COLD_JUNCTION_CAL_TEMP);
}

/**
 * @brief  映射函数
 */
static uint16_t TempControl_MapValue(uint16_t x, uint16_t in_min, uint16_t in_max,
                                      uint16_t out_min, uint16_t out_max)
{
    // 防止除零错误
    if (in_max == in_min) {
        return out_min;
    }

    // 使用 uint32_t 进行中间计算,防止溢出
    uint32_t temp = (uint32_t)(x - in_min) * (uint32_t)(out_max - out_min);
    return (uint16_t)(temp / (uint32_t)(in_max - in_min) + out_min);
}
