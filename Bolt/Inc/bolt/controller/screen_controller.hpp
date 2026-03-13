#ifndef BOLT_SCREEN_CONTROLLER_HPP
#define BOLT_SCREEN_CONTROLLER_HPP

#include <cstdint>
#include <cstdio>

#include "interface.hpp"
#include "interface/timer_interface.hpp"
#include "interface/battery_monitor.hpp"
#include "interface/screen_interface.hpp"
#include "controller/icm20948_controller.hpp"
#include "controller/button_controller.hpp"

using bolt::timer::ProcessAsyncTimerPort;
using bolt::adc::SyncBatteryMonitor;
using bolt::controller::ICM20948Controller;
using bolt::controller::ICM20948Data;

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
                             ICM20948Controller *imu,
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
                    page_ = (page_ + 1) % SCREEN_PAGE_COUNT;
                };
            }

            ~ScreenController() = default;

            void refresh()
            {
                ICM20948Data imuData{};
                imu_->readAll(imuData);

                display_->clear();

                switch (page_)
                {
                case 0:
                {
                    float v = battery_->voltage();
                    float pct = battery_->percentage();
                    snprintf(buf_, sizeof(buf_), "BATTERY");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "%.2f V", v);
                    display_->print(0, 16, buf_);
                    snprintf(buf_, sizeof(buf_), "%3.0f %%", pct);
                    display_->print(0, 32, buf_);
                    break;
                }
                case 1:
                    snprintf(buf_, sizeof(buf_), "ACCEL");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "X: %d", imuData.accel_x);
                    display_->print(0, 16, buf_);
                    snprintf(buf_, sizeof(buf_), "Y: %d", imuData.accel_y);
                    display_->print(0, 32, buf_);
                    snprintf(buf_, sizeof(buf_), "Z: %d", imuData.accel_z);
                    display_->print(0, 48, buf_);
                    break;
                case 2:
                    snprintf(buf_, sizeof(buf_), "GYRO");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "X: %d", imuData.gyro_x);
                    display_->print(0, 16, buf_);
                    snprintf(buf_, sizeof(buf_), "Y: %d", imuData.gyro_y);
                    display_->print(0, 32, buf_);
                    snprintf(buf_, sizeof(buf_), "Z: %d", imuData.gyro_z);
                    display_->print(0, 48, buf_);
                    break;
                case 3:
                    snprintf(buf_, sizeof(buf_), "MAG + TEMP");
                    display_->print(0, 0, buf_);
                    snprintf(buf_, sizeof(buf_), "X:%d Y:%d", imuData.mag_x, imuData.mag_y);
                    display_->print(0, 16, buf_);
                    snprintf(buf_, sizeof(buf_), "Z: %d", imuData.mag_z);
                    display_->print(0, 32, buf_);
                    snprintf(buf_, sizeof(buf_), "T: %d", imuData.temperature);
                    display_->print(0, 48, buf_);
                    break;
                default:
                    break;
                }

                display_->update();
            }

        private:
            ProcessAsyncTimerPort *sampler_;
            SyncBatteryMonitor *battery_;
            bolt::Display *display_;
            ICM20948Controller *imu_;

            uint8_t page_ = 0;
            char buf_[32]{};
        };
    }
}

#endif /* BOLT_SCREEN_CONTROLLER_HPP */
