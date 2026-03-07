# 项目架构重构说明

## 概述

本项目采用三层架构设计，将原有的单一main.c文件(800+行)重构为模块化的分层架构，提高了代码的可维护性、可测试性和可扩展性。

## 架构设计

### 1. 硬件抽象层 (HAL)
位置: `Core/Inc/HAL/` 和 `Core/Src/HAL/`

封装所有硬件操作，提供统一的接口：

- **timer_hal.c/h** - 定时器管理
  - 系统时钟获取
  - 延时函数

- **adc_hal.c/h** - ADC采样
  - 温度传感器读取(带降噪)
  - 参考电压读取
  - 输入电压读取

- **pwm_hal.c/h** - PWM控制
  - 加热器PWM占空比设置
  - 加热器开关控制

- **encoder_hal.c/h** - 编码器读取
  - 编码器计数值读取
  - 编码器反向设置

- **button_hal.c/h** - 按键检测
  - 按键状态读取
  - 短按/长按事件检测

- **buzzer_hal.c/h** - 蜂鸣器控制
  - 蜂鸣器鸣叫
  - 蜂鸣器使能控制
  - 自动关闭任务

### 2. 业务逻辑层 (Service)
位置: `Core/Inc/Service/` 和 `Core/Src/Service/`

封装核心业务逻辑：

- **config_service.c/h** - 配置管理
  - 系统配置读写
  - EEPROM操作
  - 出厂设置恢复

- **temp_control.c/h** - 温度控制
  - PID温度控制
  - 温度采样与计算
  - 温度校准
  - 控制周期管理(加热95ms → 稳定3ms → 采样计算2ms)

- **power_mgmt.c/h** - 电源管理
  - 休眠模式管理
  - 关机模式管理
  - 强化模式管理
  - 手柄移动检测

- **ui_service.c/h** - UI服务
  - 编码器处理
  - 按键处理
  - 显示管理
  - UI事件回调

### 3. 应用层 (App)
位置: `Core/Inc/` 和 `Core/Src/`

主程序逻辑：

- **app_state_machine.c/h** - 应用状态机
  - 启动状态(BOOT)
  - 运行状态(RUNNING)
  - 菜单状态(MENU)
  - 错误状态(ERROR)
  - 状态转换管理
  - 错误处理

- **main.c** - 主程序入口(已简化)

## 架构优势

### 1. 模块化设计
- 每个模块职责单一，易于理解和维护
- 模块之间通过接口通信，降低耦合度

### 2. 分层清晰
- 硬件抽象层：隔离硬件细节
- 业务逻辑层：封装核心功能
- 应用层：协调各模块工作

### 3. 易于测试
- 每个模块可独立测试
- HAL层可以mock，便于单元测试

### 4. 易于扩展
- 新增功能只需添加新模块
- 修改硬件只需修改HAL层

### 5. 代码复用
- HAL层可用于其他STM32项目
- Service层可移植到不同硬件平台

## 状态机设计

### 应用状态
```
BOOT (启动) → RUNNING (运行) ⇄ MENU (菜单)
                ↓
            ERROR (错误)
```

### 温度控制状态
```
HEATING (加热95ms) → WAIT_SETTLE (稳定3ms) → MEASURE_AND_PID (采样计算2ms)
    ↑                                                    ↓
    └────────────────────────────────────────────────────┘
```

### 电源模式
- NORMAL (正常)
- SLEEP (休眠)
- OFF (关机)
- BOOST (强化)

## 关键常量定义

### 温度控制
```c
#define TEMP_CTRL_HEATING_TIME_MS    95   // 加热时间
#define TEMP_CTRL_SETTLE_TIME_MS     3    // 稳定时间
#define TEMP_CTRL_CYCLE_TIME_MS      100  // 控制周期
#define TEMP_SMOOTHIE                0.05f // 温度平滑系数
```

### UI更新
```c
#define UI_DISPLAY_UPDATE_PERIOD_MS  50   // 显示刷新周期
#define UI_TMP75_READ_PERIOD_MS      500  // TMP75读取周期
```

### 按键
```c
#define BUTTON_LONG_PRESS_TIME_MS    500  // 长按时间阈值
```

### 蜂鸣器
```c
#define BUZZER_DEFAULT_DURATION_MS   32   // 默认鸣叫时长
```

## 事件驱动机制

### UI事件
```c
typedef enum {
    UI_EVENT_NONE = 0,
    UI_EVENT_TEMP_CHANGED,      // 温度设定值改变
    UI_EVENT_SHORT_PRESS,       // 短按按键
    UI_EVENT_LONG_PRESS,        // 长按按键
    UI_EVENT_DISPLAY_UPDATE     // 显示需要更新
} UIEvent_e;
```

通过回调函数实现模块间解耦：
```c
UIService_RegisterCallback(AppStateMachine_UIEventCallback);
```

## 错误处理

### 错误类型
```c
typedef enum {
    APP_ERROR_NONE = 0,
    APP_ERROR_TEMP_SENSOR_FAULT,    // 温度传感器故障
    APP_ERROR_TEMP_TOO_HIGH,        // 温度过高
    APP_ERROR_VOLTAGE_TOO_LOW,      // 电压过低
    APP_ERROR_EEPROM_FAULT          // EEPROM故障
} AppError_e;
```

### 错误处理流程
1. 检测到错误 → 设置错误代码
2. 自动切换到ERROR状态
3. 关闭加热器
4. 蜂鸣器报警
5. 显示错误信息

## 下一步工作

### 1. 集成到编译系统
- 将新文件添加到Makefile或IDE项目中
- 配置头文件搜索路径

### 2. 迁移原有代码
- 逐步将main.c中的代码迁移到新架构
- 保持功能完全一致

### 3. 测试验证
- 单元测试各模块
- 集成测试整体功能
- 硬件测试

### 4. 功能完善
- 实现EEPROM读写
- 完善菜单功能
- 增加错误恢复机制

### 5. 文档完善
- 添加API文档
- 添加使用示例
- 添加调试指南

## 注意事项

1. **编译配置**
   - 需要将新的.c文件添加到编译列表
   - 需要添加头文件搜索路径: `Core/Inc/HAL` 和 `Core/Inc/Service`

2. **外部变量**
   - `TIM16_Tick` - 需要在tim.c中定义
   - `Vin` - 需要在main.c中定义
   - `d0` - 需要在main.c中定义

3. **依赖关系**
   - HAL层不依赖Service层和App层
   - Service层依赖HAL层，不依赖App层
   - App层依赖HAL层和Service层

4. **中断处理**
   - TIM16中断需要更新TIM16_Tick
   - 其他中断保持不变

## 总结

通过这次重构，我们将一个800+行的单一文件拆分成了20+个模块化文件，每个文件职责清晰，代码结构更加合理。这不仅提高了代码的可读性和可维护性，也为后续的功能扩展和优化奠定了良好的基础。
