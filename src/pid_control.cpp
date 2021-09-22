/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "pid_control.h"
#include "rpm_sensing.h"
#include "motor.h"
#include "helpers.h"

PidController pid;

void PidController::updatePidValue(PidConfigEnum pid, int16_t steps)
{
    float *output;
    float value;
    switch(pid) {
        case PidConfigEnum::KP:
            output = &Kp;
            value = 0.1;
            break;
        case PidConfigEnum::KI:
            output = &Ki;
            value = 0.00001;
            break;
        case PidConfigEnum::KD:
            output = &Kd;
            value = 0.0001;
            break;
        case PidConfigEnum::OMUL:
            output = &outputMultiplier;
            value = 0.00001;
            break;
        case PidConfigEnum::DTMUL:
            output = &dtMultiplier;
            value = 0.00001;
            break;
        default:
            return;
    }
    #if HAVE_PRINTF_FLT
        Serial.printf_P(PSTR("pid %u: s:%d: %.2f>"), pid, steps, value);
    #else
        Serial.printf_P(PSTR("pid %u: s:%d: "), (int)pid, steps);
        Serial.print(value);
        Serial.print('>');
    #endif
    *output += value * steps;
    Serial.println(*output);
}

void PidController::printValues(Print &buffer, uint8_t type) const
// void PidController::printValues(char *buffer, uint8_t type) const
{
    //PrintBuffer buf(buffer, len);
    if (IS_PID_BV(type, OMUL) || IS_PID_BV(type, DTMUL)) {
        #if HAVE_PRINTF_FLT
            buffer.printf_P(PSTR("PID Multi. %f\n%f\n"), outputMultiplier, dtMultiplier);
        #else
            buffer.print(F("PID Multi. "));
            buffer.println(outputMultiplier, 6);
            buffer.print(' ');
            buffer.println(dtMultiplier, 6);
        #endif
    }
    else {
        #if HAVE_PRINTF_FLT
            buffer.printf_P(PSTR("PID %.1f %.4f %.3f\n"), Kp, Ki, Kd);
        #else
            buffer.print(F("PID "));
            buffer.print(Kp, 1);
            buffer.print(' ');
            buffer.print(Ki, 4);
            buffer.print(' ');
            buffer.println(Kd, 3);
        #endif
    }
}

void PidController::reset()
{
    duty_cycle = VELOCITY_START_DUTY_CYCLE;
    integral = 0;
    previous_error = 0;
    last_update.start();
}

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif
// called inside ISR
void PidController::update()
{
    auto _micros = micros();
    float delta_t = last_update.getTime(_micros) * dtMultiplier;
    #if DEBUG_PID_CONTROLLER
        uint16_t delta_t_short = last_update.getTime(_micros);
    #endif
    last_update.start(_micros);

    auto measured_pulse_length = rpm_sense.getTimerIntegralMicros();
    // ui_data.display_pulse_length_integral = (ui_data.display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + measured_pulse_length) / (DISPLAY_RPM_MULTIPLIER + 1);
    ui_data.updateRpmPulseWidth(measured_pulse_length);

    if (motor.isOn()) {
        int32_t error = measured_pulse_length - set_point_rpm_pulse_length; // values are inverted
        #if DEBUG && 0
            if (abs(error) > 1000000) {
                Serial.printf_P(PSTR("ERR %lu, %lu, %u\n"), error, measured_pulse_length, set_point_rpm_pulse_length);
                motor.stop(MotorStateEnum::ERROR);
            }
        #endif

        integral = integral + error * delta_t;
        float derivative = (error - previous_error) / delta_t;

        int32_t output = (Kp * error + Ki * integral + Kd * derivative) * outputMultiplier;
        #if DEBUG && 0
            if (abs(output) > 10000) {
                Serial.printf_P(PSTR("OVF %ld\n"), output);
            }
        #endif
        previous_error = error;

        duty_cycle = std::clamp<int32_t>(output, MIN_DUTY_CYCLE_PID, MAX_DUTY_CYCLE);
        motor.setSpeed(duty_cycle);

        // ui_data.display_duty_cycle_integral = (ui_data.display_duty_cycle_integral * DISPLAY_DUTY_CYCLE_MULTIPLIER + duty_cycle) / (DISPLAY_DUTY_CYCLE_MULTIPLIER + 1);
        ui_data.updateDutyCyle(duty_cycle);
    }
}
