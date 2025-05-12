#include "controller/led_controller.hpp"

void bolt::controller::LedController::blink(int16_t numBlinks, uint16_t delayMs)
{
    if (numBlinks > 0)
    {
        for (uint16_t i = 0; i < numBlinks; i++)
        {
            this->ledPin_.toggle();
            vTaskDelay(delayMs / portTICK_RATE_MS);
        }
    }
    else if (numBlinks == -1)
    {
        while (1)
        {
            this->ledPin_.toggle();
            vTaskDelay(delayMs / portTICK_RATE_MS);
        }
    }
}