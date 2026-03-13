#ifndef BOLT_BATTERY_MONITOR_HPP
#define BOLT_BATTERY_MONITOR_HPP

#include "interface.hpp"
#include "stm32f1xx_hal.h"

namespace bolt
{
    namespace adc
    {
        /**
         * SyncBatteryMonitor performs a blocking ADC read to measure battery voltage.
         *
         * Uses direct register access (no HAL ADC module required).
         *
         * Voltage conversion:
         *   v_measured = (raw / 4096.0f) * V_REF
         *   v_battery  = v_measured * divider_ratio
         *
         * Percentage is clamped to [0, 100] over the configured [min_voltage, max_voltage] range.
         * Default parameters target a 3S LiPo (9.6 V empty, 12.6 V full).
         */
        class SyncBatteryMonitor : public bolt::BatteryMonitor
        {
        public:
            /**
             * @param adc            Pointer to the ADC peripheral register block (e.g. ADC1).
             * @param channel        ADC channel number (0-17).
             * @param divider_ratio  Reciprocal of the voltage-divider attenuation factor
             *                       (e.g. 3.0f when the divider reduces battery voltage
             *                       to 1/3 before the ADC pin). Use 1.0f for direct connection.
             * @param min_voltage    Battery voltage corresponding to 0% (volts).
             * @param max_voltage    Battery voltage corresponding to 100% (volts).
             */
            SyncBatteryMonitor(ADC_TypeDef *adc,
                               uint8_t channel,
                               float divider_ratio = 4.0303f,
                               float min_voltage   = 9.6f,
                               float max_voltage   = 12.6f);

            ~SyncBatteryMonitor() = default;

            float voltage() override;
            float percentage() override;

        private:
            uint32_t readRaw();

            ADC_TypeDef *adc_;
            uint8_t channel_;
            float divider_ratio_;
            float min_voltage_;
            float max_voltage_;

            static constexpr float ADC_STEPS = 4096.0f;
            static constexpr float V_REF     = 3.3f;
        };
    }
}

#endif /* BOLT_BATTERY_MONITOR_HPP */
