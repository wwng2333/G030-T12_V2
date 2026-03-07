# 新模块文件结构

## 文件清单

### 硬件抽象层 (HAL)

#### 头文件 (Core/Inc/HAL/)
- adc_hal.h - ADC硬件抽象层
- button_hal.h - 按键硬件抽象层
- buzzer_hal.h - 蜂鸣器硬件抽象层
- encoder_hal.h - 编码器硬件抽象层
- pwm_hal.h - PWM硬件抽象层
- timer_hal.h - 定时器硬件抽象层

#### 源文件 (Core/Src/HAL/)
- adc_hal.c
- button_hal.c
- buzzer_hal.c
- encoder_hal.c
- pwm_hal.c
- timer_hal.c

### 业务逻辑层 (Service)

#### 头文件 (Core/Inc/Service/)
- config_service.h - 配置管理服务
- power_mgmt.h - 电源管理服务
- temp_control.h - 温度控制服务
- ui_service.h - UI服务

#### 源文件 (Core/Src/Service/)
- config_service.c
- power_mgmt.c
- temp_control.c
- ui_service.c

### 应用层 (App)

#### 头文件 (Core/Inc/)
- app_state_machine.h - 应用状态机

#### 源文件 (Core/Src/)
- app_state_machine.c

## 文件树结构

```
G030-T12_V2/
├── Core/
│   ├── Inc/
│   │   ├── HAL/
│   │   │   ├── adc_hal.h
│   │   │   ├── button_hal.h
│   │   │   ├── buzzer_hal.h
│   │   │   ├── encoder_hal.h
│   │   │   ├── pwm_hal.h
│   │   │   └── timer_hal.h
│   │   ├── Service/
│   │   │   ├── config_service.h
│   │   │   ├── power_mgmt.h
│   │   │   ├── temp_control.h
│   │   │   └── ui_service.h
│   │   ├── app_state_machine.h
│   │   └── main.h (原有文件)
│   └── Src/
│       ├── HAL/
│       │   ├── adc_hal.c
│       │   ├── button_hal.c
│       │   ├── buzzer_hal.c
│       │   ├── encoder_hal.c
│       │   ├── pwm_hal.c
│       │   └── timer_hal.c
│       ├── Service/
│       │   ├── config_service.c
│       │   ├── power_mgmt.c
│       │   ├── temp_control.c
│       │   └── ui_service.c
│       ├── app_state_machine.c
│       └── main.c (原有文件,需要重构)
└── ARCHITECTURE.md (架构说明文档)
```

## 统计信息

- **总文件数**: 21个新文件
  - 头文件: 11个
  - 源文件: 10个

- **代码行数估算**:
  - HAL层: ~600行
  - Service层: ~800行
  - App层: ~200行
  - 总计: ~1600行

## 模块依赖关系

```
┌─────────────────────────────────────────┐
│           Application Layer             │
│  ┌───────────────────────────────────┐  │
│  │    app_state_machine.c/h          │  │
│  │    main.c                         │  │
│  └───────────────────────────────────┘  │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│          Service Layer                  │
│  ┌─────────────┐  ┌──────────────────┐ │
│  │ config_     │  │ temp_control.c/h │ │
│  │ service.c/h │  │                  │ │
│  └─────────────┘  └──────────────────┘ │
│  ┌─────────────┐  ┌──────────────────┐ │
│  │ power_      │  │ ui_service.c/h   │ │
│  │ mgmt.c/h    │  │                  │ │
│  └─────────────┘  └──────────────────┘ │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│           HAL Layer                     │
│  ┌──────────┐ ┌──────────┐ ┌─────────┐ │
│  │ adc_hal  │ │ pwm_hal  │ │ timer_  │ │
│  │ .c/h     │ │ .c/h     │ │ hal.c/h │ │
│  └──────────┘ └──────────┘ └─────────┘ │
│  ┌──────────┐ ┌──────────┐ ┌─────────┐ │
│  │ encoder_ │ │ button_  │ │ buzzer_ │ │
│  │ hal.c/h  │ │ hal.c/h  │ │ hal.c/h │ │
│  └──────────┘ └──────────┘ └─────────┘ │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│         Hardware (STM32G0)              │
│  TIM, ADC, GPIO, SPI, etc.              │
└─────────────────────────────────────────┘
```

## 编译配置

### 需要添加的头文件搜索路径
```
-I Core/Inc/HAL
-I Core/Inc/Service
```

### 需要添加到编译的源文件
```
Core/Src/HAL/adc_hal.c
Core/Src/HAL/button_hal.c
Core/Src/HAL/buzzer_hal.c
Core/Src/HAL/encoder_hal.c
Core/Src/HAL/pwm_hal.c
Core/Src/HAL/timer_hal.c
Core/Src/Service/config_service.c
Core/Src/Service/power_mgmt.c
Core/Src/Service/temp_control.c
Core/Src/Service/ui_service.c
Core/Src/app_state_machine.c
```

## 下一步操作

1. **配置编译系统**
   - 在Makefile或IDE项目中添加新文件
   - 添加头文件搜索路径

2. **重构main.c**
   - 简化main()函数
   - 移除已迁移到新模块的代码
   - 保留STM32 CubeMX生成的代码

3. **测试验证**
   - 编译检查
   - 功能测试
   - 性能测试

4. **文档完善**
   - API文档
   - 使用示例
   - 调试指南
