/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include "timer.h"
#include "rpm_sensing.h"
#include "motor.h"

class PidController {
public:
    // extra bits for the duty cycle
    static constexpr uint8_t kDutyCycleShift = 7;

public:
    PidController();

    // reset PID controller before starting
    void reset();

    // restore default PID settings
    void resetPidValues();
    // set PID values
    void setPidValues(const PidSettings &pid);
    // get PID values
    const PidSettings &getPidValues() const;
    // update single value of the PID settings
    void updatePidValue(PidConfigEnum pid, int16_t steps);
    // display PID values
    void printValues(Print &buffer, bool displayOM = true) const;
    // display PID settings menu
    void displayPidSettingsMenu(PidConfigEnum highlight) const;

    #if HAVE_PID_CONTROLLER_STATS
        Stats _stats;
    #endif
    PidSettings _settings;

// private:
    friend class RpmSense;
    friend class CurrentLimit;

    // this method will turn the motor on when called without checking anything
    void updateTicks(int32_t ticks);
    // get duty cycle of the PID controller
    uint8_t getDutyCycle() const;

private:
    float _integral;
    uint16_t _dutyCycle;
    int32_t _previousError;
    MicrosTimer _lastUpdate;
};

inline PidController::PidController() :
    _integral(0),
    _dutyCycle(0),
    _previousError(0)
    // _setPointRpm(0),
    // _setPointRpmTicks(0)
{
    resetPidValues();
}

inline void PidController::resetPidValues()
{
    _settings = PidSettings();
}

inline void PidController::setPidValues(const PidSettings &pid)
{
    _settings = pid;
}

inline const PidSettings &PidController::getPidValues() const
{
    return _settings;
}

inline uint8_t PidController::getDutyCycle() const
{
    return _dutyCycle >> kDutyCycleShift;
}

extern PidController pid;
