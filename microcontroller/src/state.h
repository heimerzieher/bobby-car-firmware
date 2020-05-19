#ifndef STATE_H
#define STATE_H

#include <HardwareSerial.h>

#include "controllerstate.h"

class State
{
public:
    State();
    ControllerState frontControllerState;
    ControllerState rearControllerState;
    //MotorController frontController(Serial1);
    //MotorController rearController(Serial2);

};

/*
void saveState(State& state)
{

}

State loadState(void)
{

}
*/

#endif // STATE_H
