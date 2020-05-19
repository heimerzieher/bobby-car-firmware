#include "controllerstate.h"

ControllerState::ControllerState()
{
    this->leftMotorSettings = defaultMotorSettings();
    this->rightMotorSettings = defaultMotorSettings();
}
