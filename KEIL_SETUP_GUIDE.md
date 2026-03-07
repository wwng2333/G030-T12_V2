# 需要添加到Keil项目的文件清单

## 编译配置步骤

### 1. 添加源文件到项目

在Keil MDK中，按照以下步骤添加文件：

#### HAL层文件（创建新组 "HAL"）
```
../Core/Src/HAL/timer_hal.c
../Core/Src/HAL/adc_hal.c
../Core/Src/HAL/pwm_hal.c
../Core/Src/HAL/encoder_hal.c
../Core/Src/HAL/button_hal.c
../Core/Src/HAL/buzzer_hal.c
```

#### Service层文件（创建新组 "Service"）
```
../Core/Src/Service/config_service.c
../Core/Src/Service/temp_control.c
../Core/Src/Service/power_mgmt.c
../Core/Src/Service/ui_service.c
```

#### App层文件（添加到现有的 "Application/User" 组）
```
../Core/Src/app_state_machine.c
```

### 2. 添加头文件搜索路径

在 Project → Options for Target → C/C++ → Include Paths 中添加：

```
../Core/Inc/HAL
../Core/Inc/Service
```

### 3. 详细操作步骤

#### 步骤1: 添加HAL层文件
1. 在Project窗口中，右键点击项目名称
2. 选择 "Add Group..."
3. 输入组名: `HAL`
4. 右键点击新创建的 "HAL" 组
5. 选择 "Add Existing Files to Group 'HAL'..."
6. 导航到 `Core/Src/HAL/` 目录
7. 选择所有6个.c文件（可以按住Ctrl多选）
8. 点击 "Add"

#### 步骤2: 添加Service层文件
1. 重复上述步骤，创建 "Service" 组
2. 添加 `Core/Src/Service/` 目录下的4个.c文件

#### 步骤3: 添加App层文件
1. 找到现有的 "Application/User" 或 "Application" 组
2. 右键点击该组
3. 选择 "Add Existing Files to Group..."
4. 选择 `Core/Src/app_state_machine.c`
5. 点击 "Add"

#### 步骤4: 添加头文件路径
1. 右键点击项目名称
2. 选择 "Options for Target..."
3. 切换到 "C/C++" 标签页
4. 在 "Include Paths" 区域，点击右侧的 "..." 按钮
5. 点击 "New" 按钮，添加路径：
   - `../Core/Inc/HAL`
6. 再次点击 "New" 按钮，添加路径：
   - `../Core/Inc/Service`
7. 点击 "OK" 保存

### 4. 验证配置

添加完成后，你的项目结构应该类似：

```
Project
├── Application/User
│   ├── main.c
│   ├── app_state_machine.c  ← 新添加
│   └── ...
├── HAL  ← 新组
│   ├── timer_hal.c
│   ├── adc_hal.c
│   ├── pwm_hal.c
│   ├── encoder_hal.c
│   ├── button_hal.c
│   └── buzzer_hal.c
├── Service  ← 新组
│   ├── config_service.c
│   ├── temp_control.c
│   ├── power_mgmt.c
│   └── ui_service.c
└── ...
```

### 5. 编译测试

1. 点击 "Build" 按钮（或按F7）
2. 检查编译输出，应该没有错误
3. 如果有警告，检查是否是正常的警告

### 6. 常见问题

#### 问题1: 找不到头文件
**错误信息**: `fatal error: xxx_hal.h: No such file or directory`

**解决方案**:
- 检查头文件搜索路径是否正确添加
- 确保路径使用相对路径 `../Core/Inc/HAL`

#### 问题2: 重复定义
**错误信息**: `multiple definition of 'xxx'`

**解决方案**:
- 检查是否重复添加了源文件
- 检查main_backup.c是否被添加到项目中（应该排除）

#### 问题3: 链接错误
**错误信息**: `undefined reference to 'xxx'`

**解决方案**:
- 确保所有.c文件都已添加到项目
- 检查函数名是否拼写正确

### 7. 快速检查清单

- [ ] 添加了6个HAL层.c文件
- [ ] 添加了4个Service层.c文件
- [ ] 添加了1个app_state_machine.c文件
- [ ] 添加了2个头文件搜索路径
- [ ] 编译无错误
- [ ] 编译无警告（或只有预期的警告）

### 8. 文件路径参考

如果使用绝对路径，文件位置如下：

```
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\HAL\timer_hal.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\HAL\adc_hal.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\HAL\pwm_hal.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\HAL\encoder_hal.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\HAL\button_hal.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\HAL\buzzer_hal.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\Service\config_service.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\Service\temp_control.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\Service\power_mgmt.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\Service\ui_service.c
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Src\app_state_machine.c
```

头文件路径：
```
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Inc\HAL
C:\Users\Administrator\Desktop\G030-T12_V2\Core\Inc\Service
```
