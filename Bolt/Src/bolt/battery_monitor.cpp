#include "interface/battery_monitor.hpp"

bolt::adc::SyncBatteryMonitor::SyncBatteryMonitor(ADC_TypeDef *adc,
                                                   uint8_t channel,
                                                   float divider_ratio,
                                                   float min_voltage,
                                                   float max_voltage)
    : adc_(adc),
      channel_(channel),
      divider_ratio_(divider_ratio),
      min_voltage_(min_voltage),
      max_voltage_(max_voltage)
{
}

uint32_t bolt::adc::SyncBatteryMonitor::readRaw()
{
    /* Enable ADC1 peripheral clock if not already on */
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* Configure channel in regular sequence rank 1, 239.5 cycle sample time */
    if (channel_ <= 9U)
    {
        adc_->SMPR2 &= ~(0x7UL << (channel_ * 3U));
        adc_->SMPR2 |= (0x7UL << (channel_ * 3U)); /* 239.5 cycles */
    }
    else
    {
        uint8_t ch = channel_ - 10U;
        adc_->SMPR1 &= ~(0x7UL << (ch * 3U));
        adc_->SMPR1 |= (0x7UL << (ch * 3U));
    }

    /* Set sequence length = 1 conversion, channel in SQ1 */
    adc_->SQR1 = 0;                                     /* L[3:0] = 0 → 1 conversion */
    adc_->SQR3 = (channel_ & 0x1FU);                    /* SQ1 = channel */

    /* Enable ADC (first ADON write powers on; requires stabilisation) */
    adc_->CR2 |= ADC_CR2_ADON;

    /* Wait for ADC power-up stabilisation (~1 µs at 72 MHz ≈ 72 cycles) */
    for (volatile uint32_t i = 0U; i < 100U; ++i) { __NOP(); }

    /* Configure software trigger: EXTTRIG=1, EXTSEL=111 (SWSTART) */
    adc_->CR2 |= ADC_CR2_EXTTRIG | (0x7UL << ADC_CR2_EXTSEL_Pos);

    /* Start conversion */
    adc_->CR2 |= ADC_CR2_SWSTART;

    /* Wait for end of conversion (timeout ~1 ms at 72 MHz) */
    uint32_t timeout = 72000U;
    while (!(adc_->SR & ADC_SR_EOC))
    {
        if (--timeout == 0U)
        {
            adc_->CR2 &= ~ADC_CR2_ADON;
            return 0U;
        }
    }

    uint32_t raw = adc_->DR;

    /* Disable ADC */
    adc_->CR2 &= ~ADC_CR2_ADON;

    return raw;
}

float bolt::adc::SyncBatteryMonitor::voltage()
{
    uint32_t raw = readRaw();
    float v_measured = (static_cast<float>(raw) / ADC_STEPS) * V_REF;
    return v_measured * divider_ratio_;
}

float bolt::adc::SyncBatteryMonitor::percentage()
{
    float v = voltage();

    if (v <= min_voltage_)
    {
        return 0.0f;
    }
    if (v >= max_voltage_)
    {
        return 100.0f;
    }

    return ((v - min_voltage_) / (max_voltage_ - min_voltage_)) * 100.0f;
}
