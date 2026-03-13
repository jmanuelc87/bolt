#ifndef BOLT_SCREEN_INTERFACE_HPP
#define BOLT_SCREEN_INTERFACE_HPP

#include <cstdint>
#include <cstdarg>
#include <cstdio>

#include "interface.hpp"
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

namespace bolt
{
    namespace display
    {
        class SSD1306Display : public bolt::Display
        {
        public:
            SSD1306Display() = default;
            ~SSD1306Display() = default;

            void init() override
            {
                ssd1306_Init();
                ssd1306_Fill(Black);
                ssd1306_UpdateScreen();
            }

            void clear() override
            {
                ssd1306_Fill(Black);
            }

            void update() override
            {
                ssd1306_UpdateScreen();
            }

            void print(uint8_t x, uint8_t y, const char *str) override
            {
                ssd1306_SetCursor(x, y);
                ssd1306_WriteString(const_cast<char *>(str), Font_6x8, White);
            }

            void printf(uint8_t x, uint8_t y, const char *fmt, ...)
            {
                va_list args;
                va_start(args, fmt);
                vsnprintf(buf_, sizeof(buf_), fmt, args);
                va_end(args);
                print(x, y, buf_);
            }

            void setOn(bool on) override
            {
                ssd1306_SetDisplayOn(on ? 1 : 0);
            }

        private:
            char buf_[32]{};
        };
    }
}

#endif /* BOLT_SCREEN_INTERFACE_HPP */
