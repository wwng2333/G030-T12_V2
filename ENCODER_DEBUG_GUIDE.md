# 编码器调试指南

## 问题描述
编码器不能用，编码器使用TIM1编码器模式读取。

## 可能的原因

### 1. TIM1初始化顺序问题

**检查点**:
- TIM1是否在main.c中正确初始化？
- Encoder_HAL_Init()是否在TIM1初始化之后调用？

**当前代码流程**:
```
main.c:
  MX_TIM1_Init()  ← TIM1初始化

app_state_machine.c (BOOT状态):
  Encoder_HAL_Init()  ← 启动编码器
```

**问题**: `Encoder_HAL_Init()` 在 `AppStateMachine_BootHandler()` 中调用，但此时TIM1可能还没有初始化！

### 2. 初始化顺序修复

需要确保TIM1在编码器HAL初始化之前就已经初始化。

**方案1: 在main.c中已经初始化TIM1**
检查main.c中是否调用了 `MX_TIM1_Init()`

**方案2: 在Encoder_HAL_Init中初始化TIM1**
如果main.c中没有初始化TIM1，需要在Encoder_HAL_Init中调用。

## 调试步骤

### 步骤1: 检查TIM1是否初始化

在 `Encoder_HAL_Init()` 中添加调试代码：

```c
void Encoder_HAL_Init(void)
{
    // 检查TIM1是否已启用
    if (!(RCC->APBENR2 & RCC_APBENR2_TIM1EN)) {
        // TIM1时钟未使能，需要初始化
        MX_TIM1_Init();
    }

    // 启动编码器
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_EnableCounter(TIM1);

    // 从配置中读取编码器反向设置
    SystemConfig_t* config = ConfigService_GetConfig();
    s_encoderReversed = config->flags.encoderReversed;
}
```

### 步骤2: 检查编码器计数值

在 `UIService_ProcessEncoderInternal()` 中添加调试输出：

```c
static void UIService_ProcessEncoderInternal(void)
{
    static int16_t step_accumulator = 0;
    uint16_t current_counter = Encoder_HAL_GetCounter();

    // 调试输出
    static uint32_t last_debug = 0;
    if (Timer_HAL_GetTick() - last_debug >= 1000) {
        // 每秒输出一次编码器计数值
        printf("Encoder: %d\n", current_counter);
        last_debug = Timer_HAL_GetTick();
    }

    // ... 原有代码
}
```

### 步骤3: 检查GPIO配置

确认PA8和PA9配置为TIM1的编码器输入：
- PA8 → TIM1_CH1
- PA9 → TIM1_CH2

### 步骤4: 检查编码器模式

在tim.c中确认TIM1配置为编码器模式：
```c
LL_TIM_SetEncoderMode(TIM1, LL_TIM_ENCODERMODE_X2_TI1);
```

## 快速修复方案

### 修复1: 确保TIM1在main.c中初始化

检查 `main.c` 中是否有：
```c
MX_TIM1_Init();
```

如果没有，需要添加。

### 修复2: 修改Encoder_HAL_Init

更新 `encoder_hal.c`:

```c
#include "encoder_hal.h"
#include "config_service.h"
#include "tim.h"
#include "stm32g0xx_ll_tim.h"

void Encoder_HAL_Init(void)
{
    // 确保TIM1已初始化
    // 如果main.c中没有调用MX_TIM1_Init()，在这里调用
    // MX_TIM1_Init();  // 取消注释如果需要

    // 启动编码器通道
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

    // 启动计数器
    LL_TIM_EnableCounter(TIM1);

    // 从配置中读取编码器反向设置
    SystemConfig_t* config = ConfigService_GetConfig();
    s_encoderReversed = config->flags.encoderReversed;
}
```

### 修复3: 检查编码器初始计数值

在 `UIService_Init()` 中正确初始化编码器状态：

```c
void UIService_Init(void)
{
    // ... 其他初始化代码

    // 初始化编码器状态 - 读取当前计数值
    s_encoderStatus.last_counter = Encoder_HAL_GetCounter();
    s_encoderStatus.total_count = 0;
    s_encoderStatus.delta = 0;

    // ... 其他初始化代码
}
```

## 测试方法

### 测试1: 读取编码器计数值

添加测试代码：
```c
// 在main循环中或某个周期性任务中
uint16_t counter = Encoder_HAL_GetCounter();
printf("Encoder counter: %d\n", counter);
```

旋转编码器，观察计数值是否变化。

### 测试2: 检查delta值

在 `UIService_ProcessEncoderInternal()` 中：
```c
int16_t delta = (int16_t)(current_counter - s_encoderStatus.last_counter);
if (delta != 0) {
    printf("Delta: %d\n", delta);
}
```

### 测试3: 检查温度设定值

在温度改变后：
```c
if (step_accumulator >= 2) {
    uint16_t current_temp = TempControl_GetTargetTemp();
    TempControl_SetTargetTemp(current_temp + 5);
    printf("Temp set to: %d\n", current_temp + 5);
    // ...
}
```

## 常见问题

### Q1: 编码器计数值始终为0
**原因**: TIM1未正确初始化或未启动
**解决**: 确保MX_TIM1_Init()被调用，并且LL_TIM_EnableCounter(TIM1)被执行

### Q2: 编码器计数值变化但温度不变
**原因**: delta计算错误或step_accumulator阈值问题
**解决**: 检查delta计算逻辑，调整阈值

### Q3: 编码器方向反了
**原因**: encoderReversed设置错误
**解决**: 在配置中设置 `config->flags.encoderReversed = 1`

### Q4: 编码器响应太灵敏或太迟钝
**原因**: step_accumulator阈值不合适
**解决**: 调整阈值（当前为2，可以改为4）

## 推荐的修改

### 修改encoder_hal.c

```c
/**
 * @brief  初始化编码器硬件抽象层
 */
void Encoder_HAL_Init(void)
{
    // 检查TIM1是否已初始化，如果没有则初始化
    if (!(LL_APB2_GRP1_IsEnabledClock(LL_APB2_GRP1_PERIPH_TIM1))) {
        MX_TIM1_Init();
    }

    // 启动编码器通道
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH2);

    // 启动计数器
    LL_TIM_EnableCounter(TIM1);

    // 从配置中读取编码器反向设置
    SystemConfig_t* config = ConfigService_GetConfig();
    s_encoderReversed = config->flags.encoderReversed;
}
```

## 总结

编码器不工作的最可能原因是TIM1初始化顺序问题。请按照以下步骤排查：

1. ✅ 确认main.c中调用了MX_TIM1_Init()
2. ✅ 确认Encoder_HAL_Init()在TIM1初始化之后调用
3. ✅ 添加调试输出查看编码器计数值
4. ✅ 检查GPIO配置是否正确
5. ✅ 测试编码器硬件是否正常

如果以上都正常，问题可能在于：
- 编码器硬件连接问题
- GPIO复用功能配置错误
- TIM1时钟未使能
