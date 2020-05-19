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

    bool test;

    MotorSettings leftMotorSettings;
    MotorSettings rightMotorSettings;

    MotorInput leftMotorInput;
    MotorInput rightMotorInput;

    uint16_t  checksum;
} testStruct;


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

  MotorStatus leftMotorStatus;
  MotorStatus rightMotorStatus;

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

uint16_t calculateChecksumMotorStatus(MotorStatus status);

uint16_t calculateChecksumMotorSettings(MotorSettings settings);

uint16_t calculateChecksumFeedback(SerialFeedback feedback);

uint16_t calculateChecksumCommand(SerialCommand command);




#endif // SERIALCOMMINICATION_H
