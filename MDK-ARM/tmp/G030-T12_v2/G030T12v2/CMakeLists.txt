# CMSIS Build CMakeLists generated on 2024-04-06T14:04:58

cmake_minimum_required(VERSION 3.22)

# Target options

set(TARGET G030-T12_v2)
set(CPU Cortex-M0+)
set(PRJ_DIR "h:/G030-T12_V2/MDK-ARM")
set(OUT_DIR "H:/G030-T12_V2/MDK-ARM/out/G030-T12_v2/G030T12v2")
set(INT_DIR "H:/G030-T12_V2/MDK-ARM/tmp/G030-T12_v2/G030T12v2")
set(FPU NO_FPU)
set(BYTE_ORDER Little-endian)
set(OPTIMIZE none)
set(DEBUG on)
set(CC_FLAGS_GLOBAL "-std=c99 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(CXX_FLAGS_GLOBAL "-xc++ -std=c++03 -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -ffunction-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality")
set(LD_FLAGS_GLOBAL "--library_type=microlib --map --load_addr_map_info --xref --callgraph --symbols --info sizes --info totals --info unused --info veneers --strict --summary_stderr --info summarysizes")
set(LD_SCRIPT "h:/G030-T12_V2/MDK-ARM/G030-T12_v2_G030T12v2.sct")

set(DEFINES
  STM32G030xx
  _RTE_
  USE_FULL_LL_DRIVER
  HSE_VALUE=8000000
  HSE_STARTUP_TIMEOUT=100
  LSE_STARTUP_TIMEOUT=5000
  LSE_VALUE=32768
  EXTERNAL_CLOCK_VALUE=12288000
  HSI_VALUE=16000000
  LSI_VALUE=32000
  VDD_VALUE=3300
  PREFETCH_ENABLE=1
  INSTRUCTION_CACHE_ENABLE=1
  DATA_CACHE_ENABLE=1
  __MICROLIB
)

set(INC_PATHS
  "h:/G030-T12_V2/Core/Inc"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Inc"
  "h:/G030-T12_V2/Drivers/CMSIS/Device/ST/STM32G0xx/Include"
  "h:/G030-T12_V2/Drivers/CMSIS/Include"
  "h:/G030-T12_V2/Core/Src/u8g2"
  "h:/G030-T12_V2/MDK-ARM/RTE/Compiler"
  "h:/G030-T12_V2/MDK-ARM/RTE/_G030T12v2"
  "C:/Users/Robin Lu/AppData/Local/arm/packs/ARM/CMSIS/5.9.0/CMSIS/Core/Include"
  "C:/Users/Robin Lu/AppData/Local/arm/packs/Keil/ARM_Compiler/1.7.2/Include"
  "C:/Users/Robin Lu/AppData/Local/arm/packs/Keil/STM32G0xx_DFP/1.5.0/Drivers/CMSIS/Device/ST/STM32G0xx/Include"
)

set(AS_ARM_SRC_FILES
  "h:/G030-T12_V2/MDK-ARM/startup_stm32g030xx.s"
)

set(CC_SRC_FILES
  "C:/Users/Robin Lu/AppData/Local/arm/packs/Keil/ARM_Compiler/1.7.2/Source/EventRecorder.c"
  "C:/Users/Robin Lu/AppData/Local/arm/packs/Keil/ARM_Compiler/1.7.2/Source/retarget_io.c"
  "h:/G030-T12_V2/Core/Src/Flash.c"
  "h:/G030-T12_V2/Core/Src/PID.c"
  "h:/G030-T12_V2/Core/Src/SEGGER_RTT.c"
  "h:/G030-T12_V2/Core/Src/SEGGER_RTT_printf.c"
  "h:/G030-T12_V2/Core/Src/adc.c"
  "h:/G030-T12_V2/Core/Src/gpio.c"
  "h:/G030-T12_V2/Core/Src/i2c.c"
  "h:/G030-T12_V2/Core/Src/main.c"
  "h:/G030-T12_V2/Core/Src/oled_driver.c"
  "h:/G030-T12_V2/Core/Src/spi.c"
  "h:/G030-T12_V2/Core/Src/stm32_button.c"
  "h:/G030-T12_V2/Core/Src/stm32g0xx_it.c"
  "h:/G030-T12_V2/Core/Src/system_stm32g0xx.c"
  "h:/G030-T12_V2/Core/Src/tim.c"
  "h:/G030-T12_V2/Core/Src/timer.c"
  "h:/G030-T12_V2/Core/Src/u8g2/mui.c"
  "h:/G030-T12_V2/Core/Src/u8g2/mui_u8g2.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_arc.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_bitmap.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_box.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_buffer.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_button.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_circle.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_cleardisplay.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_d_memory.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_d_setup.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_font.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_fonts.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_hvline.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_input_value.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_intersection.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_kerning.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_line.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_ll_hvline.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_message.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_polygon.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_selection_list.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8g2_setup.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8log.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8log_u8g2.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8log_u8x8.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_8x8.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_byte.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_cad.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_capture.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_d_ssd1306_128x64_noname.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_debounce.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_display.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_fonts.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_gpio.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_input_value.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_message.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_selection_list.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_setup.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_string.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_u16toa.c"
  "h:/G030-T12_V2/Core/Src/u8g2/u8x8_u8toa.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_adc.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_dma.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_exti.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_gpio.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_pwr.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_rcc.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_spi.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_tim.c"
  "h:/G030-T12_V2/Drivers/STM32G0xx_HAL_Driver/Src/stm32g0xx_ll_utils.c"
)

set(AS_FLAGS_h:/G030-T12_V2/MDK-ARM/startup_stm32g030xx.s "-masm=auto")

# Toolchain config map

set(TOOLCHAIN_VERSION_MIN "6.21.0")
set(TOOLCHAIN_VERSION_MAX "6.21.0")
include ("C:/Users/Robin Lu/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.3.0/etc/AC6.6.18.0.cmake")
include ("C:/Users/Robin Lu/.vcpkg/artifacts/2139c4c6/tools.open.cmsis.pack.cmsis.toolbox/2.3.0/etc/CMSIS-Build-Utils.cmake")

# Setup project

project(${TARGET} LANGUAGES AS_ARM C)

cbuild_get_running_toolchain(TOOLCHAIN_ROOT TOOLCHAIN_VERSION C)

# Global Flags

set(CMAKE_AS_ARM_FLAGS "${AS_ARM_CPU} ${AS_ARM_BYTE_ORDER} ${AS_ARM_DEFINES} ${AS_ARM_OPTIONS_FLAGS} ${AS_ARM_FLAGS}")
cbuild_get_system_includes(CC_SYS_INC_PATHS_LIST CC_SYS_INC_PATHS)
set(CMAKE_C_FLAGS "${CC_CPU} ${CC_BYTE_ORDER} ${CC_DEFINES} ${CC_OPTIONS_FLAGS} ${CC_FLAGS} ${CC_FLAGS_GLOBAL} ${CC_SYS_INC_PATHS}")
set(CMAKE_C_LINK_FLAGS "${LD_CPU} ${_LS}\"${LD_SCRIPT}\" ${LD_FLAGS_GLOBAL} ${LD_OPTIONS_FLAGS} ${LD_FLAGS}")

# Local Flags

foreach(SRC ${AS_ARM_SRC_FILES})
  string(REPLACE " " "?" S ${SRC})
  if(DEFINED AS_FLAGS_${S})
    set(AS_FLAGS_LOCAL "${AS_FLAGS_${S}}")
  else()
    set(AS_FLAGS_LOCAL "${AS_FLAGS_GLOBAL}")
  endif()
  set_source_files_properties(${SRC} PROPERTIES COMPILE_FLAGS "${AS_FLAGS_LOCAL}")
  set_source_files_properties(${SRC} PROPERTIES LANGUAGE AS_ARM)
endforeach()

# Compilation Database

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_custom_target(database COMMAND ${CMAKE_COMMAND} -E copy_if_different "${INT_DIR}/compile_commands.json" "${OUT_DIR}")

# Setup Target

add_executable(${TARGET} ${AS_ARM_SRC_FILES} ${CC_SRC_FILES})
set_target_properties(${TARGET} PROPERTIES PREFIX "" SUFFIX ".axf" OUTPUT_NAME "G030-T12_v2")
set_target_properties(${TARGET} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUT_DIR} LINK_DEPENDS ${LD_SCRIPT})
target_include_directories(${TARGET} PUBLIC ${INC_PATHS})
