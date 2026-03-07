# 新架构快速参考指南

## 如何使用新架构

### 1. 添加新功能

#### 示例：添加一个新的传感器读取功能

**步骤1: 在HAL层添加硬件接口**
```c
// Core/Inc/HAL/sensor_hal.h
#ifndef __SENSOR_HAL_H
#define __SENSOR_HAL_H

uint16_t Sensor_HAL_ReadValue(void);

#endif
```

```c
// Core/Src/HAL/sensor_hal.c
#include "sensor_hal.h"
#include "adc.h"

uint16_t Sensor_HAL_ReadValue(void)
{
    // 实现传感器读取
    return 0;
}
```

**步骤2: 在Service层添加业务逻辑**
```c
// Core/Inc/Service/sensor_service.h
#ifndef __SENSOR_SERVICE_H
#define __SENSOR_SERVICE_H

void SensorService_Init(void);
void SensorService_Task(void);
uint16_t SensorService_GetValue(void);

#endif
```

**步骤3: 在App层调用**
```c
// Core/Src/app_state_machine.c
#include "sensor_service.h"

static void AppStateMachine_BootHandler(void)
{
    // ...
    SensorService_Init();
    // ...
}

static void AppStateMachine_RunningHandler(void)
{
    // ...
    SensorService_Task();
    // ...
}
```

### 2. 修改现有功能

#### 示例：修改温度控制参数

**方法1: 修改配置服务**
```c
// 在config_service.c中修改默认值
static SystemConfig_t s_sysConfig = {
    .defaultTemp = 350,  // 修改默认温度为350°C
    // ...
};
```

**方法2: 修改温度控制逻辑**
```c
// 在temp_control.c中修改PID参数
static const double s_aggKp = 6000;  // 增大P参数
static const double s_aggKi = 6000;  // 增大I参数
```

### 3. 调试技巧

#### 查看温度控制状态
```c
TempControlState_t* state = TempControl_GetState();
printf("Current Temp: %.1f\n", state->currentTemp);
printf("Target Temp: %d\n", state->setTemp);
printf("PID Output: %.0f\n", state->pidOutput);
```

#### 查看电源管理状态
```c
PowerMgmtState_t* state = PowerMgmt_GetState();
printf("Sleep Mode: %d\n", state->inSleepMode);
printf("Boost Mode: %d\n", state->inBoostMode);
```

#### 查看系统配置
```c
SystemConfig_t* config = ConfigService_GetConfig();
printf("Default Temp: %d\n", config->defaultTemp);
printf("Buzzer Enabled: %d\n", config->flags.buzzerEnabled);
```

### 4. 常用API

#### HAL层API

**定时器**
```c
uint32_t tick = Timer_HAL_GetTick();  // 获取系统时钟(ms)
Timer_HAL_Delay(100);                 // 延时100ms
```

**ADC**
```c
uint16_t temp_adc = ADC_HAL_ReadTemperature();  // 读取温度ADC值
uint16_t vref = ADC_HAL_ReadVref();             // 读取参考电压
```

**PWM**
```c
PWM_HAL_SetHeaterDuty(1000);  // 设置加热器PWM占空比
PWM_HAL_HeaterOff();          // 关闭加热器
```

**编码器**
```c
uint16_t counter = Encoder_HAL_GetCounter();  // 读取编码器计数
Encoder_HAL_SetReversed(true);                // 设置编码器反向
```

**按键**
```c
ButtonEvent_e event = Button_HAL_Scan();  // 扫描按键
if (event == BUTTON_EVENT_SHORT_PRESS) {
    // 处理短按
}
```

**蜂鸣器**
```c
Buzzer_HAL_Beep(32);              // 蜂鸣32ms
Buzzer_HAL_SetEnabled(false);     // 禁用蜂鸣器
```

#### Service层API

**配置管理**
```c
SystemConfig_t* config = ConfigService_GetConfig();
uint16_t temp = ConfigService_GetDefaultTemp();
bool buzzer = ConfigService_IsBuzzerEnabled();
```

**温度控制**
```c
TempControl_SetTargetTemp(350);           // 设置目标温度
float temp = TempControl_GetCurrentTemp(); // 获取当前温度
bool ready = TempControl_IsWorky();       // 是否达到工作温度
```

**电源管理**
```c
PowerMgmt_SetBoostMode(true);         // 进入强化模式
PowerMgmt_NotifyHandleMoved();        // 通知手柄移动(唤醒)
bool sleeping = PowerMgmt_IsInSleepMode();  // 是否休眠
```

**UI服务**
```c
UIService_ForceDisplayUpdate();       // 强制刷新显示
UIService_RegisterCallback(callback); // 注册UI事件回调
```

#### App层API

**状态机**
```c
AppState_e state = AppStateMachine_GetState();  // 获取当前状态
AppStateMachine_SetState(APP_STATE_MENU);       // 切换状态
AppStateMachine_SetError(APP_ERROR_TEMP_TOO_HIGH);  // 设置错误
```

### 5. 事件回调机制

#### 注册UI事件回调
```c
void MyUIEventCallback(UIEvent_e event, void* data)
{
    switch (event) {
        case UI_EVENT_TEMP_CHANGED:
            // 温度改变
            break;
        case UI_EVENT_SHORT_PRESS:
            // 短按按键
            break;
        case UI_EVENT_LONG_PRESS:
            // 长按按键
            break;
        default:
            break;
    }
}

// 在初始化时注册
UIService_RegisterCallback(MyUIEventCallback);
```

### 6. 状态机使用

#### 添加新状态
```c
// 在app_state_machine.h中添加
typedef enum {
    APP_STATE_BOOT,
    APP_STATE_RUNNING,
    APP_STATE_MENU,
    APP_STATE_CALIBRATION,  // 新增校准状态
    APP_STATE_ERROR
} AppState_e;
```

```c
// 在app_state_machine.c中添加处理函数
static void AppStateMachine_CalibrationHandler(void)
{
    // 实现校准逻辑
}

// 在Run函数中添加case
void AppStateMachine_Run(void)
{
    switch (s_appState) {
        // ...
        case APP_STATE_CALIBRATION:
            AppStateMachine_CalibrationHandler();
            break;
        // ...
    }
}
```

### 7. 错误处理

#### 检测错误
```c
// 在温度控制任务中
if (current_temp > 500) {
    AppStateMachine_SetError(APP_ERROR_TEMP_TOO_HIGH);
}
```

#### 自定义错误处理
```c
// 在app_state_machine.c中修改
static void AppStateMachine_ErrorHandler(void)
{
    // 关闭加热器
    PWM_HAL_HeaterOff();

    // 根据错误类型处理
    AppError_e error = AppStateMachine_GetError();
    switch (error) {
        case APP_ERROR_TEMP_TOO_HIGH:
            // 温度过高处理
            break;
        case APP_ERROR_VOLTAGE_TOO_LOW:
            // 电压过低处理
            break;
        default:
            break;
    }
}
```

### 8. 性能优化建议

#### 减少函数调用开销
```c
// 不好的做法：频繁调用
for (int i = 0; i < 100; i++) {
    uint32_t tick = Timer_HAL_GetTick();
    // ...
}

// 好的做法：缓存结果
uint32_t tick = Timer_HAL_GetTick();
for (int i = 0; i < 100; i++) {
    // 使用缓存的tick
}
```

#### 避免重复计算
```c
// 不好的做法
if (TempControl_GetCurrentTemp() > 300) {
    float temp = TempControl_GetCurrentTemp();
    // ...
}

// 好的做法
float temp = TempControl_GetCurrentTemp();
if (temp > 300) {
    // ...
}
```

### 9. 常见问题

#### Q: 如何修改温度控制周期？
A: 在temp_control.h中修改常量：
```c
#define TEMP_CTRL_HEATING_TIME_MS    95   // 加热时间
#define TEMP_CTRL_SETTLE_TIME_MS     3    // 稳定时间
#define TEMP_CTRL_CYCLE_TIME_MS      100  // 控制周期
```

#### Q: 如何修改显示刷新频率？
A: 在ui_service.h中修改常量：
```c
#define UI_DISPLAY_UPDATE_PERIOD_MS  50   // 显示刷新周期
```

#### Q: 如何禁用蜂鸣器？
A: 方法1 - 通过配置：
```c
SystemConfig_t* config = ConfigService_GetConfig();
config->flags.buzzerEnabled = 0;
```
方法2 - 通过HAL：
```c
Buzzer_HAL_SetEnabled(false);
```

#### Q: 如何添加新的显示界面？
A: 在ui_service.c中添加新的显示函数：
```c
void UIService_DisplayCalibrationScreen(void)
{
    u8g2_FirstPage(&s_u8g2);
    do {
        // 绘制校准界面
    } while (u8g2_NextPage(&s_u8g2));
}
```

### 10. 代码风格指南

#### 命名规范
- **模块名**: 大驼峰 + 下划线，如 `Timer_HAL_GetTick()`
- **私有函数**: static + 小写s前缀，如 `static void s_internalFunction()`
- **私有变量**: static + 小写s前缀，如 `static uint32_t s_counter`
- **类型定义**: 大驼峰 + _t后缀，如 `TempControlState_t`
- **枚举**: 大驼峰 + _e后缀，如 `AppState_e`

#### 注释规范
```c
/**
 * @brief  函数简要说明
 * @param  param1: 参数1说明
 * @param  param2: 参数2说明
 * @retval 返回值说明
 */
uint32_t MyFunction(uint32_t param1, uint32_t param2)
{
    // 实现
}
```

## 总结

新架构提供了清晰的分层结构和丰富的API接口，使得添加新功能、修改现有功能、调试和优化都变得更加简单。遵循上述指南，你可以快速上手并高效地开发和维护代码。
