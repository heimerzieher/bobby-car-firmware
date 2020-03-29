#ifndef SERIALCOMMINICATION_H
#define SERIALCOMMINICATION_H

#include <stdint.h>
#include "MotorSettings.h"
#include "MotorInput.h"


typedef struct
{
  uint16_t  start;

  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   adc1;
  int16_t   adc2;
  int16_t   pwm1;
  int16_t   pwm2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;

  uint16_t test;

  MotorSettings leftMotorSettings;
  MotorSettings rightMotorSettings;

  MotorInput leftMotorInput;
  MotorInput rightMotorInput;

  uint16_t  checksum;
} SerialFeedback;

typedef struct
{
  uint16_t  start;

  MotorSettings leftMotorSettings;
  MotorSettings rightMotorSettings;

  uint16_t overrideMotorInput;

  MotorInput leftMotorInput;
  MotorInput rightMotorInput;

  uint16_t  checksum;
} SerialCommand;


uint16_t calculateChecksumMotorInput(MotorInput input)
{
    return (input.pwm);
}

uint16_t calculateChecksumMotorSettings(MotorSettings settings)
{
    return ((uint16_t)settings.enable ^ (uint16_t)settings.diagnosticsEnable ^
            (uint16_t)settings.controlMode ^ (uint16_t)settings.controlType ^
            settings.iMotorMax ^ settings.iDcCurrentMax  ^  settings.nMotorMax ^
            (uint16_t)settings.fieldWeakeningEnable ^ settings.fieldWeakeningLow ^
            settings.fieldWeakeningMax ^ settings.fieldWeakeningHigh);
}

uint16_t calculateChecksumFeedback(SerialFeedback feedback)
{
    return (feedback.start ^ feedback.cmd1 ^ feedback.cmd2 ^
            feedback.adc1 ^ feedback.adc2 ^ feedback.pwm1 ^ feedback.pwm2 ^
            feedback.speedR_meas ^ feedback.speedL_meas ^ feedback.batVoltage ^
            feedback.boardTemp ^ feedback.cmdLed ^ feedback.test ^
            calculateChecksumMotorSettings(feedback.leftMotorSettings) ^ calculateChecksumMotorSettings(feedback.rightMotorSettings));
}

uint16_t calculateChecksumCommand(SerialCommand command)
{
    return (command.start ^ calculateChecksumMotorSettings(command.leftMotorSettings) ^
            calculateChecksumMotorSettings(command.rightMotorSettings) ^
            command.overrideMotorInput ^
            calculateChecksumMotorInput(command.leftMotorInput) ^
            calculateChecksumMotorInput(command.rightMotorInput));
}

#endif // SERIALCOMMINICATION_H
