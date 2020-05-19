#include "SerialCommunication.h" 

SerialFeedback feedback;
SerialFeedback newFeedback;

SerialCommand command;

uint16_t calculateChecksumMotorInput(MotorInput input)
{
    return (input.pwm);
}

uint16_t calculateChecksumMotorStatus(MotorStatus status)
{
    return (status.nRpm ^ status.hallA ^ status.hallB ^ status.hallC ^ status.dcCurrent ^ status.errorCode);
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
            calculateChecksumMotorSettings(feedback.leftMotorSettings) ^ calculateChecksumMotorSettings(feedback.rightMotorSettings) /*^
            calculateChecksumMotorStatus(feedback.leftMotorStatus) ^ calculateChecksumMotorStatus(feedback.rightMotorStatus)*/);
}

uint16_t calculateChecksumCommand(SerialCommand command)
{
    return (command.start ^ calculateChecksumMotorSettings(command.leftMotorSettings) ^
            calculateChecksumMotorSettings(command.rightMotorSettings) ^
            command.overrideMotorInput ^
            calculateChecksumMotorInput(command.leftMotorInput) ^
            calculateChecksumMotorInput(command.rightMotorInput));
}
