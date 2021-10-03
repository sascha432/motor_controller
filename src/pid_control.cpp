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

void PidController::updatePidValue(PidConfigEnum pid, int16_t steps)
{
    auto &value = _settings[pid];
    value += (KNOB_GET_VALUE(steps, KNOB_PID_SPEED) * 0.001);
}

uint8_t PidController::getDutyCycle() const
{
    return _dutyCycle >> kDutyCycleShift;
}

void PidController::printValues(Print &buffer) const
{
    char temp[64];
    PrintBuffer buf(temp, sizeof(temp));

    buf.printTrimmed(_settings.Kp);
    buf.print(' ');
    buf.printTrimmed(_settings.Ki);
    buf.print(' ');
    buf.printTrimmed(_settings.Kd);

    buffer.print(F("PID "));
    buffer.println(buf.getBuffer());
}

void PidController::displayPidSettingsMenu(PidConfigEnum highlight) const
{
    char temp[32];
    PrintBuffer buf(temp, sizeof(temp));

    display_print_hl((highlight == PidConfigEnum::KP), F("Kp"));
    buf.clearPrintTrimmed(_settings.Kp);
    display.println(buf.getBuffer());

    display_print_hl((highlight == PidConfigEnum::KI), F("Ki"));
    buf.clearPrintTrimmed(_settings.Ki);
    display.println(buf.getBuffer());

    display_print_hl((highlight == PidConfigEnum::KD), F("Kd"));
    buf.clearPrintTrimmed(_settings.Kd);
    display.println(buf.getBuffer());

    display_print_hl((highlight == PidConfigEnum::SAVE), F("Save"));
    display_print_hl((highlight == PidConfigEnum::RESTORE), F("Restore"));
}

void PidController::reset()
{
    data.setSetPointRPM(data.getSetPointRPM());
    _integral = 0;
    _dutyCycle = START_DUTY_CYCLE_PID << kDutyCycleShift;
    _previousError = 0;
    _lastUpdate.start();
}

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif

void PidController::updateTicks(int32_t ticks)
{
    auto _micros = micros();
    float delta_t = 100000.0 / (float)_lastUpdate.getTime(_micros);
    // 1.0 / (1000 * 1000.0 * _lastUpdate.getTime(_micros));
    _lastUpdate.start(_micros);

    float error = (int32_t)ticks - data.getSetPointRPMTicks(); // values are inverted
    // error *= std::min<float>(1.0, delta_t);
    // delta_t /= 1000;

    float output = _settings.Kp * error;
    if (_settings.Ki) {
        _integral = _integral + (error * delta_t);
        output += _settings.Ki * _integral;
    }
    if (_settings.Kd) {
        float derivative = (error - _previousError) / delta_t;
        output += _settings.Kd * derivative;
        _previousError = error;
    }

    output /= 32;

    // // // the output is divided by ((1 << kDutyCycleShift) * 32) = 4096 before added to the duty cycle 0-255
    // int32_t tmp = output;
    // tmp >>= 5;

    uint8_t dutyCycleBefore = current_limit.getDutyCycle(getDutyCycle());
    _dutyCycle = std::clamp<int32_t>(_dutyCycle + output, MIN_DUTY_CYCLE_PID << kDutyCycleShift, MAX_DUTY_CYCLE_PID << kDutyCycleShift);
    uint8_t dutyCycle8 = current_limit.getDutyCycle(getDutyCycle());

    #if HAVE_PID_CONTROLLER_STATS
        if (dutyCycleBefore != dutyCycle8) {
            setMotorPWM(dutyCycle8);
            _stats.add(RPM_SENSE_TICKS_TO_RPM(ticks), ui_data.getRpm(), data.getSetPointRPM(), dutyCycle8);
        }
        else {
            if (_stats._state == StatsState::RUNNING && ui_data.getRpm() == data.getSetPointRPM()) {
                _stats.stop();
                motor.stop(MotorStateEnum::OFF);
            }
        }
    #else
        if (dutyCycleBefore != dutyCycle8) {
            setMotorPWM(dutyCycle8);
        }
    #endif
}
