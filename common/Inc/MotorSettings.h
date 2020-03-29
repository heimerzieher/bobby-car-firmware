#ifndef SETTINGS_H
#define SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    Commutation = 0,
    Sinusoidal = 1,
    FOC = 2
} ControlType;

typedef enum
{
    OpenMode = 0,
    VoltageMode = 1,
    SpeedMode = 2,
    TorqueMode = 3
} ControlMode;

typedef struct
{
    uint16_t enable; // Enable/Disable Motor

    uint16_t diagnosticsEnable; // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

    uint16_t controlType;
    uint16_t controlMode;

    int16_t iMotorMax;                    // [A] Maximum motor current limit
    int16_t iDcCurrentMax;                 // [A] Maximum DC Link current limit (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
    int16_t nMotorMax;                   // [rpm] Maximum motor speed limit

    uint16_t fieldWeakeningEnable;           // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled, 1 = Enabled
    int16_t fieldWeakeningMax;              // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed.
    int16_t fieldWeakeningPhaseAdvanceMax; // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
    int16_t fieldWeakeningHigh;          // [-] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
    int16_t fieldWeakeningLow;           // [-] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.
} MotorSettings;

//TODO make a const variable
MotorSettings defaultMotorSettings(void)
{
    MotorSettings settings;


    settings.enable = (uint16_t)true; // Enable/Disable Motor

    settings.diagnosticsEnable = (uint16_t)true; // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

    settings.controlType = (uint16_t)FOC;
    settings.controlMode = (uint16_t)TorqueMode;

    settings.iMotorMax  = 12;                    // [A] Maximum motor current limit
    settings.iDcCurrentMax = 14;                 // [A] Maximum DC Link current limit (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
    settings.nMotorMax = 1000;                   // [rpm] Maximum motor speed limit

    settings.fieldWeakeningEnable = (uint16_t)true;           // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled, 1 = Enabled
    settings.fieldWeakeningMax = 5;              // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed.
    settings.fieldWeakeningPhaseAdvanceMax = 25; // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
    settings.fieldWeakeningHigh = 1500;          // [-] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
    settings.fieldWeakeningLow = 1000;           // [-] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.

    return settings;
};

/*
typedef struct MotorSettings
{
    bool enable = false; // Enable/Disable Motor

    bool diagnosticsEnable = true; // [-] Motor Diagnostics enable flag: 0 = Disabled, 1 = Enabled (default)

    ControlType controlType = ControlType::FOC;
    ControlMode controlMode = ControlMode::TorqueMode;

    int16_t iMotorMax  = 12;                    // [A] Maximum motor current limit
    int16_t iDcCurrentMax = 14;                 // [A] Maximum DC Link current limit (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
    int16_t nMotorMax = 1000;                   // [rpm] Maximum motor speed limit

    bool fieldWeakeningEnable = true;           // [-] Field Weakening / Phase Advance enable flag: 0 = Disabled, 1 = Enabled
    int16_t fieldWeakeningMax = 5;              // [A] Maximum Field Weakening D axis current (only for FOC). Higher current results in higher maximum speed.
    int16_t fieldWeakeningPhaseAdvanceMax = 25; // [deg] Maximum Phase Advance angle (only for SIN). Higher angle results in higher maximum speed.
    int16_t fieldWeakeningHigh = 1500;          // [-] Input target High threshold for reaching maximum Field Weakening / Phase Advance. Do NOT set this higher than 1500.
    int16_t fieldWeakeningLow = 1000;           // [-] Input target Low threshold for starting Field Weakening / Phase Advance. Do NOT set this higher than 1000.
};*/

#endif // SETTINGS_H
