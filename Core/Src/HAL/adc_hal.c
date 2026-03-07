/**
 ******************************************************************************
 * @file    adc_hal.c
 * @brief   ADC硬件抽象层实现
 ******************************************************************************
 */

#include "adc_hal.h"
#include "adc.h"
#include "stm32g0xx_ll_adc.h"

/* 私有变量 ----------------------------------------------------------*/
static uint16_t s_vcc_mV = 3300;  // 参考电压(mV)

/* 私有函数声明 ----------------------------------------------------*/
static uint16_t ADC_DenoiseRead(uint32_t adc_channel);

/* 导出的函数 --------------------------------------------------------*/

/**
 * @brief  初始化ADC硬件抽象层
 */
void ADC_HAL_Init(void)
{
    // ADC已经在MX_ADC1_Init()和Activate_ADC()中初始化
    // 读取参考电压
    s_vcc_mV = ADC_HAL_ReadVref();
}

/**
 * @brief  读取温度传感器ADC值(带降噪)
 * @retval 12位ADC值
 */
uint16_t ADC_HAL_ReadTemperature(void)
{
    return ADC_DenoiseRead(LL_ADC_CHANNEL_10);
}

/**
 * @brief  读取参考电压
 * @retval 参考电压(mV)
 */
uint16_t ADC_HAL_ReadVref(void)
{
    uint16_t vrefint_data = ADC_DenoiseRead(LL_ADC_CHANNEL_VREFINT);
    s_vcc_mV = __LL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_data, LL_ADC_RESOLUTION_12B);
    return s_vcc_mV;
}

/**
 * @brief  读取输入电压 (ADC Channel 11)
 * @retval 输入电压(mV)
 */
uint16_t ADC_HAL_ReadVin(void)
{
    // 1. 读取通道 11 的原始 ADC 数值
    // 假设 ADC_DenoiseRead 是你封装好的去抖动/平均值读取函数
    uint16_t ad_raw = ADC_DenoiseRead(LL_ADC_CHANNEL_11);

    // 2. 将原始数值转换为当前 VCC 下的毫伏值 (mV)
    // __LL_ADC_CALC_DATA_TO_VOLTAGE 需要参考电压、原始值、分辨率三个参数
    uint16_t vin_raw_mV = __LL_ADC_CALC_DATA_TO_VOLTAGE(s_vcc_mV, ad_raw, LL_ADC_RESOLUTION_12B);

    // 3. 根据硬件电路计算真实电压（扩大 11 倍）
    // 注意：结果使用 uint32_t 进行中间计算以防溢出
    uint32_t vin_real_mV = (uint32_t)vin_raw_mV * 11;

    return (uint16_t)vin_real_mV;
}

/**
 * @brief  读取MCU内部温度
 * @retval MCU温度(°C)
 */
uint16_t ADC_HAL_ReadMcuTemp(void)
{
    // TODO: 根据实际需求实现
    return 25;
}

/* 私有函数 ----------------------------------------------------------*/

/**
 * @brief  ADC降噪读取(平均16次采样)
 * @param  adc_channel: ADC通道
 * @retval 12位ADC值
 */
static uint16_t ADC_DenoiseRead(uint32_t adc_channel)
{
    uint32_t result = 0;

    LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, adc_channel);
    LL_ADC_SetChannelSamplingTime(ADC1, adc_channel, LL_ADC_SAMPLINGTIME_160CYCLES_5);

    // 开启SEVONPEND,允许被挂起的外设标志直接唤醒WFE
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;
    LL_ADC_EnableIT_EOC(ADC1);

    // 采样16次求平均
    for (uint8_t i = 0; i < 16; i++) {
        LL_ADC_REG_StartConversion(ADC1);
        while (LL_ADC_IsActiveFlag_EOC(ADC1) == RESET) {
            __WFE();  // 进入Sleep模式,等待ADC转换完成
        }
        result += LL_ADC_REG_ReadConversionData12(ADC1);
        LL_ADC_ClearFlag_EOC(ADC1);
        NVIC_ClearPendingIRQ(ADC1_IRQn);
    }

    // 恢复默认状态
    LL_ADC_DisableIT_EOC(ADC1);
    SCB->SCR &= ~SCB_SCR_SEVONPEND_Msk;

    return (uint16_t)(result >> 4);  // 除以16
}
