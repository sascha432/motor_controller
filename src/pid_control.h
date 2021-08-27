/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include "timer.h"

#define PID_BV(type)                _BV((uint8_t)PidConfigEnum::type - (uint8_t)PidConfigEnum::KD)
#define IS_PID_BV(num, type)        (num & PID_BV(type))

class PidController {
public:
    static constexpr float DT_MULTIPLIER = 1 / 1000000.0 * 1;
    static constexpr float OUTPUT_MULTIPLIER = 0.042;

    PidController();

    void reset();
    void update();

    void resetPidValues();
    void setPidValues(float Kp, float Ki, float Pd);
    void getPidValues(float &Kp, float &Ki, float &Pd) const;
    void updatePidValue(PidConfigEnum pid, int16_t steps);
    inline void updatePidValue(uint8_t num, int16_t steps) {
        updatePidValue(static_cast<PidConfigEnum>(num + static_cast<uint8_t>(PidConfigEnum::KD)), steps);
    }
    // 0=auto/use data.pid_config
    // or a bitset PID_BV(KD)|PID_BV(KI)|PID_BV(DTMUL)
    void printValues(Print &buffer, uint8_t tye = 0) const;

    float Kp, Ki, Kd, outputMultiplier, dtMultiplier;
    float integral;
    long previous_error;
    volatile uint8_t duty_cycle;
    uint16_t set_point_rpm_pulse_length;
    MicrosTimer last_update;
    // float voltage_multiplier;
};

inline PidController::PidController() :
    integral(0),
    previous_error(0),
    duty_cycle(),
    set_point_rpm_pulse_length(0)
{
    resetPidValues();
}

inline void PidController::resetPidValues()
{
    Kp = 20;
    Ki = 0.0035;
    Kd = 0.055;
    outputMultiplier = OUTPUT_MULTIPLIER;
    dtMultiplier = DT_MULTIPLIER;
}

inline void PidController::setPidValues(float _Kp, float _Ki, float _Kd)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    outputMultiplier = OUTPUT_MULTIPLIER;
    dtMultiplier = DT_MULTIPLIER;
}

inline void PidController::getPidValues(float &_Kp, float &_Ki, float &_Kd) const
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

extern PidController pid;

#if DEBUG
extern bool pid_enable_serial_output;
#endif
