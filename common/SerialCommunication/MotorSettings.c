#include "MotorSettings.h" 


//TODO make a const variable
MotorSettings defaultMotorSettings(void)
{
    MotorSettings settings;


    settings.enable = true; // Enable/Disable Motor

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
