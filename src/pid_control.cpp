/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "pid_control.h"
#include "rpm_sensing.h"
#include "motor.h"
#include "helpers.h"
#include "current_limit.h"

PidController pid;

static uint8_t calc_digits(float n)
{
    if (n == 0) {
        return 0;
    }
    uint8_t digits = 1;
    for(;;) {
        if (n >= 10) {
            break;
        }
        n *= 10;
        if (++digits >= 5) {
            break;
        }
    }
    return digits;
}

static uint32_t pow10_int(uint8_t n) {
    uint32_t res = 10;
    while(n--) {
        res *= 10;
    }
    return res;
}

void PidController::updatePidValue(PidConfigEnum pid, int16_t steps)
{
    auto &value = _settings[pid];
    auto multiplier = std::clamp(10.0 / pow10_int(value == 0 ? 5 : calc_digits(value)), 0.00001, 1.0);
    value += (KNOB_GET_VALUE(steps, KNOB_PID_SPEED) * multiplier);
    if (value < 0) {
        value = 0;
    }
}

void PidController::printValues(Print &output, bool displayOM) const
{
    output.print(F("PID "));
    output.print(_settings.Kp, calc_digits(_settings.Kp));
    output.print(' ');
    output.print(_settings.Ki, calc_digits(_settings.Ki));
    output.print(' ');
    output.print(_settings.Kd, calc_digits(_settings.Kd));
    if (displayOM) {
        output.print(' ');
        output.print(_settings.OutputMultiplier, calc_digits(_settings.OutputMultiplier));
    }
    output.println();
}

void PidController::displayPidSettingsMenu(PidConfigEnum highlight) const
{
    if (highlight < PidConfigEnum::SAVE) {
        display_print_hl((highlight == PidConfigEnum::KP), F("Kp"));
        display.println(_settings.Kp, calc_digits(_settings.Kp));
    }

    if (highlight < PidConfigEnum::RESTORE) {
        display_print_hl((highlight == PidConfigEnum::KI), F("Ki"));
        display.println(_settings.Ki, calc_digits(_settings.Ki));
    }

    display_print_hl((highlight == PidConfigEnum::KD), F("Kd"));
    display.println(_settings.Kd, calc_digits(_settings.Kd));

    display_print_hl((highlight == PidConfigEnum::OM), F("Mul"));
    display.println(_settings.OutputMultiplier, calc_digits(_settings.OutputMultiplier));

    if (highlight >= PidConfigEnum::SAVE) {
        display_print_hl((highlight == PidConfigEnum::SAVE), F("Save"), '\n');
    }

    if (highlight >= PidConfigEnum::RESTORE) {
        display_print_hl(true, F("Restore"));
    }
}

void PidController::reset()
{
    data.setSetPointRPM(data.getSetPointRPM());
    _integral = 0;
    _dutyCycle = START_DUTY_CYCLE_PID << kDutyCycleShift;
    _previousError = 0;
    _lastUpdate.start();
}

static inline float calc_delta_t(uint16_t time)
{
    return 1.0 / (10000000.0 / time);
}

void PidController::updateTicks(int32_t ticks)
{
    auto _micros = micros();
    float delta_t; // gets calculated on demand
    uint16_t time = _lastUpdate.getTime(_micros);
    _lastUpdate.start(_micros);

    float error = ticks - data.getSetPointRPMTicks(); // values are inverted
    #if HAVE_CURRENT_LIMIT
        // reduce overshoot after the current limit has been triggered
        if (_currentLimitmultiplier < 1) {
            error *= _currentLimitmultiplier;
            _currentLimitmultiplier += kCurrentLimitIncrement;
        }
    #endif
    float output = _settings.Kp * error;
    if (_settings.Ki) {
        delta_t = calc_delta_t(time);
        _integral = _integral + (error * delta_t);
        output += _settings.Ki * _integral;
    }
    if (_settings.Kd) {
        if (!_settings.Ki) {
            delta_t = calc_delta_t(time);
        }
        float derivative = (error - _previousError) / delta_t;
        output += _settings.Kd * derivative;
        _previousError = error;
    }

    output *= pid.getPidValues().OutputMultiplier;

    _dutyCycle = std::clamp<int32_t>(_dutyCycle + output, MIN_DUTY_CYCLE_PID << kDutyCycleShift, MAX_DUTY_CYCLE_PID << kDutyCycleShift);

    // apply current limit
    uint8_t dutyCycle8 = current_limit.getDutyCycle(getDutyCycle());
    if (getMotorPWM_timer() != dutyCycle8) {
        setMotorPWM_timer(dutyCycle8);
    }
}
