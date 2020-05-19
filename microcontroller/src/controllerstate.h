#ifndef CONTROLLERSTATE_H
#define CONTROLLERSTATE_H

extern "C"
{
    #include "SerialCommunication.h"
}

class ControllerState
{
public:
    ControllerState();
    MotorSettings leftMotorSettings;
    MotorSettings rightMotorSettings;

    MotorInput leftMotorInput;
    MotorInput rightMotorInput;

    bool overrideLeftMotorInput;
    bool overrideRightMotorInput;
};

#endif // CONTROLLERSTATE_H
