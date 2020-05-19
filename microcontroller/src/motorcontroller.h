#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include <stdint.h>
#include <functional>
#include <Arduino.h>
#include <HardwareSerial.h>

extern "C"
{
    #include "SerialCommunication.h"
}

//#include "state.h"
#include "config.h"

class MotorController
{
public:
    MotorController(HardwareSerial& serialInterface);

    SerialFeedback feedback;
    SerialCommand command;
    bool isOnline;

    void receiveFeedback(void);
    void transmitCommand(void);

    HardwareSerial& serial;

private:
    SerialFeedback _feedback;

    uint8_t idx = 0;                        // Index for new data pointer
    uint16_t bufStartFrame;                 // Buffer Start Frame
    byte *p;                                // Pointer declaration for the new received data
    byte incomingByte;
    byte incomingBytePrev;

    uint32_t serialTimeoutCounter;
};

#endif // MOTORCONTROLLER_H
