/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include "timer.h"

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    long previous_error;
    uint8_t duty_cycle;
    uint16_t set_point_rpm_pulse_length;
    MicrosTimer last_update;
    // float voltage_multiplier;
} PidController_t;

extern PidController_t pid;

#if DEBUG
extern bool pid_enable_serial_output;
#endif

void reset_pid();
void update_pid_controller();
