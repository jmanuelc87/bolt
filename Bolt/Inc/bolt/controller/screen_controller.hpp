#ifndef BOLT_SCREEN_CONTROLLER_HPP
#define BOLT_SCREEN_CONTROLLER_HPP

#include <cstdint>
#include <cstdio>

#include "interface.hpp"
#include "interface/timer_interface.hpp"
#include "interface/battery_monitor.hpp"
#include "interface/screen_interface.hpp"
#include "controller/icm20948_sensor_controller.hpp"
#include "controller/button_controller.hpp"

using bolt::timer::ProcessAsyncTimerPort;
using bolt::adc::SyncBatteryMonitor;
using bolt::controller::ICM20948SensorController;
using bolt::controller::ICM20948SensorData;

namespace bolt
{
    namespace controller
    {
        // Pages:
        //   0 — Battery (voltage + percentage)
        //   1 — Accelerometer (X, Y, Z)
        //   2 — Gyroscope (X, Y, Z)
        //   3 — Magnetometer (X, Y, Z) + Temperature
        static constexpr uint8_t SCREEN_PAGE_COUNT = 4;

        class ScreenController
        {
        public:
            ScreenController(ProcessAsyncTimerPort *sampler,
                             SyncBatteryMonitor *battery,
                             bolt::Display *display,
                             ICM20948SensorController *imu,
                             ButtonController *button)
                : sampler_(sampler), battery_(battery), display_(display), imu_(imu)
            {
                display_->init();

                sampler_->timElapsedCompleteCallback = [this]()
                {
                    refresh();
                };

                button->onPressed = [this]()
                {
                    page_ = (page_ % SCREEN_PAGE_COUNT) + 1;
                };
            }

            ~ScreenController() = default;

            void refresh()
            {
                display_->clear();

                // newlib-nano does not support %f in snprintf without -u _printf_float.
                // Convert floats to scaled integers and format with %d instead.
                auto fmtf = [](char *dst, size_t n, float v, int dec)
                {
                    int32_t scale = 1;
                    for (int i = 0; i < dec; i++) scale *= 10;
                    int32_t x = static_cast<int32_t>(v >= 0.0f ? v * scale + 0.5f : v * scale - 0.5f);
                    int32_t a = x < 0 ? -x : x;
                    const char *sign = x < 0 ? "-" : "";
                    if (dec == 0)      snprintf(dst, n, "%s%d",       sign, (int)a);
                    else if (dec == 1) snprintf(dst, n, "%s%d.%01d",  sign, (int)(a / scale), (int)(a % scale));
                    else               snprintf(dst, n, "%s%d.%02d",  sign, (int)(a / scale), (int)(a % scale));
                };

                // 128x32 display: 4 rows of 8px each (y = 0, 8, 16, 24)
                switch (page_)
                {
                case 1:
                {
                    float v   = battery_->voltage();
                    float pct = battery_->percentage();
                    int32_t v_int = static_cast<int32_t>(v);
                    int32_t v_dec = static_cast<int32_t>((v - static_cast<float>(v_int)) * 100.0f);
                    int32_t pct_i = static_cast<int32_t>(pct);
                    snprintf(buf_, sizeof(buf_), "BATTERY");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "%d.%02dV  %d%%", (int)v_int, (int)v_dec, (int)pct_i);
                    display_->print(0, 16, buf_);
                    break;
                }
                case 2:
                {
                    ICM20948SensorData d{};
                    imu_->readAll(d);
                    char sx[10], sy[10], sz[10];
                    fmtf(sx, sizeof(sx), d.accel_x, 2);
                    fmtf(sy, sizeof(sy), d.accel_y, 2);
                    fmtf(sz, sizeof(sz), d.accel_z, 2);
                    snprintf(buf_, sizeof(buf_), "ACCEL (g)");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "X:%s Y:%s", sx, sy);
                    display_->print(0, 8, buf_);
                    snprintf(buf_, sizeof(buf_), "Z:%s", sz);
                    display_->print(0, 16, buf_);
                    break;
                }
                case 3:
                {
                    ICM20948SensorData d{};
                    imu_->readAll(d);
                    char sx[10], sy[10], sz[10];
                    fmtf(sx, sizeof(sx), d.gyro_x, 1);
                    fmtf(sy, sizeof(sy), d.gyro_y, 1);
                    fmtf(sz, sizeof(sz), d.gyro_z, 1);
                    snprintf(buf_, sizeof(buf_), "GYRO (deg/s)");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "X:%s Y:%s", sx, sy);
                    display_->print(0, 8, buf_);
                    snprintf(buf_, sizeof(buf_), "Z:%s", sz);
                    display_->print(0, 16, buf_);
                    break;
                }
                case 4:
                {
                    ICM20948SensorData d{};
                    imu_->readAll(d);
                    char smx[10], smy[10], smz[10], st[10];
                    fmtf(smx, sizeof(smx), d.mag_x,      0);
                    fmtf(smy, sizeof(smy), d.mag_y,      0);
                    fmtf(smz, sizeof(smz), d.mag_z,      0);
                    fmtf(st,  sizeof(st),  d.temperature, 1);
                    snprintf(buf_, sizeof(buf_), "MAG(uT)+TEMP");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "X:%s Y:%s", smx, smy);
                    display_->print(0, 8, buf_);
                    snprintf(buf_, sizeof(buf_), "Z:%s T:%s", smz, st);
                    display_->print(0, 16, buf_);
                    break;
                }
                default:
                    break;
                }

                snprintf(buf_, sizeof(buf_), "pg:%d/%d", page_, (int)SCREEN_PAGE_COUNT);
                display_->print(80, 24, buf_);

                display_->update();
            }

        private:
            ProcessAsyncTimerPort *sampler_;
            SyncBatteryMonitor *battery_;
            bolt::Display *display_;
            ICM20948SensorController *imu_;

            uint8_t page_ = 0;
            char buf_[32]{};
        };
    }
}

#endif /* BOLT_SCREEN_CONTROLLER_HPP */
