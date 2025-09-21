#include "controller/peripheral_controllers.hpp"

void bolt::controller::BeepController::on()
{
    this->beepPin_.setHigh();
}

void bolt::controller::BeepController::off()
{
    this->beepPin_.setLow();
}