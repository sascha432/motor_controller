/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include "timer.h"

class PidController {
public:
    static constexpr float dtMultiplier = 1 / 1000000.0 * 1;
    static constexpr float outputMultiplier = 0.042;

    PidController();

    void reset();
    void update();

    void resetPidValues();
    void setPidValues(float Kp, float Ki, float Pd);
    void getPidValues(float &Kp, float &Ki, float &Pd) const;
    void updatePidValue(uint8_t num, int8_t steps);
    void printValues(Print &buffer) const;

    float Kp, Ki, Kd;
    float integral;
    long previous_error;
    uint8_t duty_cycle;
    uint16_t set_point_rpm_pulse_length;
    MicrosTimer last_update;
    // float voltage_multiplier;
};

extern PidController pid;

#if DEBUG
extern bool pid_enable_serial_output;
#endif
