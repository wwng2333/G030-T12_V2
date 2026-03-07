# 编码器问题修复总结

## 问题描述
编码器不能用，编码器使用TIM1编码器模式读取。

## 已完成的修复

### 1. 更新encoder_hal.c ✅

**修改内容**:
- 添加了 `#include "config_service.h"` 以读取配置
- 在初始化时从配置中读取编码器反向设置
- 添加了 `LL_TIM_SetCounter(TIM1, 0)` 清零计数器
- 添加了注释说明初始化流程

**修改后的代码**:
```c
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
```

## 初始化流程确认

### 正确的初始化顺序

```
main.c:
  1. MX_GPIO_Init()           ← 初始化GPIO (PA8, PA9)
  2. MX_TIM1_Init()           ← 初始化TIM1为编码器模式
  3. AppStateMachine_Init()   ← 初始化应用状态机
       ↓
  app_state_machine.c (BOOT状态):
  4. ConfigService_Init()     ← 初始化配置服务
  5. Encoder_HAL_Init()       ← 启动编码器
       ↓
  6. UIService_Init()         ← 初始化UI服务
       - 读取编码器初始计数值
```

### TIM1配置确认

在 `tim.c` 中，TIM1配置为编码器模式：
```c
LL_TIM_SetEncoderMode(TIM1, LL_TIM_ENCODERMODE_X2_TI1);
```

- **编码器模式**: X2模式（每个边沿计数一次）
- **通道1**: PA8 (TIM1_CH1)
- **通道2**: PA9 (TIM1_CH2)
- **计数范围**: 0-65535 (16位)

## 可能仍需检查的问题

### 1. 硬件连接

确认编码器硬件连接：
- 编码器A相 → PA8
- 编码器B相 → PA9
- 编码器GND → GND
- 编码器VCC → 3.3V (或5V，取决于编码器规格)

### 2. 编码器类型

确认编码器类型是否匹配：
- **增量式编码器**: 支持 ✅
- **绝对式编码器**: 不支持 ❌

### 3. 编码器信号电平

确认信号电平：
- STM32G0的GPIO输入电平: 0-3.3V
- 如果编码器输出5V信号，需要电平转换

### 4. 上拉/下拉电阻

当前配置为无上拉/下拉：
```c
GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
```

如果编码器信号不稳定，可以尝试：
- 使能内部上拉: `LL_GPIO_PULL_UP`
- 或添加外部上拉电阻(4.7kΩ - 10kΩ)

## 调试方法

### 方法1: 使用SEGGER RTT输出

在 `UIService_ProcessEncoderInternal()` 中添加：

```c
static void UIService_ProcessEncoderInternal(void)
{
    static int16_t step_accumulator = 0;
    uint16_t current_counter = Encoder_HAL_GetCounter();

    // 调试输出
    static uint32_t last_debug = 0;
    if (Timer_HAL_GetTick() - last_debug >= 500) {
        SEGGER_RTT_printf(0, "Encoder: %d, Delta: %d\n",
                         current_counter,
                         (int16_t)(current_counter - s_encoderStatus.last_counter));
        last_debug = Timer_HAL_GetTick();
    }

    // ... 原有代码
}
```

### 方法2: 使用LED指示

添加LED闪烁来指示编码器活动：

```c
if (delta != 0) {
    // 编码器有变化，闪烁LED
    LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
```

### 方法3: 直接读取GPIO状态

在main循环中添加测试代码：

```c
// 测试GPIO输入
bool ch1 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_8);
bool ch2 = LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_9);
printf("CH1: %d, CH2: %d\n", ch1, ch2);
```

旋转编码器，观察GPIO状态是否变化。

## 如果编码器仍然不工作

### 检查清单

- [ ] 确认硬件连接正确
- [ ] 确认编码器供电正常
- [ ] 确认GPIO配置正确(PA8, PA9)
- [ ] 确认TIM1时钟已使能
- [ ] 确认编码器模式配置正确
- [ ] 测试GPIO输入是否有信号
- [ ] 检查编码器信号电平是否匹配
- [ ] 尝试添加上拉电阻
- [ ] 使用示波器查看编码器信号波形

### 替代方案

如果硬件编码器模式不工作，可以使用GPIO中断方式：

1. 配置PA8和PA9为GPIO输入+中断
2. 在中断中检测边沿
3. 软件实现编码器解码

## 预期行为

编码器正常工作时：
1. 旋转编码器，TIM1计数值应该变化
2. 正转时计数值增加，反转时计数值减少
3. 每旋转一格，温度设定值改变5°C
4. 显示屏上的温度设定值实时更新

## 总结

已完成的修复：
- ✅ 添加配置服务依赖
- ✅ 从配置中读取编码器反向设置
- ✅ 清零计数器初始值
- ✅ 确认初始化顺序正确

如果编码器仍然不工作，请按照调试方法逐步排查，最可能的原因是硬件连接或信号电平问题。
