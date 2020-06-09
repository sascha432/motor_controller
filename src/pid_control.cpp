/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "pid_control.h"
#include "rpm_sensing.h"
#include "helpers.h"

#if DEBUG_PID_CONTROLLER

uint16_t pid_test_set_points[PID_TEST_SET_POINTS] = { 1500, 750, 3500, 500, 600, 700, 2000, 2100, 1500, 1000 };  // each RPM is tested for (pid_test_duration/PID_TEST_SET_POINTS) ms
uint32_t pid_test_timer_start = 0;
uint32_t pid_test_timer_end = 0;
uint32_t pid_test_duration = 0;
uint16_t pid_test_micros_offset = 0;
uint16_t pid_test_counter;
float pid_tune_increment = 0;

#endif

PidController pid;

PidController::PidController() : integral(0), previous_error(0), duty_cycle(), set_point_rpm_pulse_length(0)
{
    resetPidValues();
}

void PidController::resetPidValues()
{
    Kp = 20;
    Ki = 0.0035;
    Kd = 0.055;
}

void PidController::setPidValues(float _Kp, float _Ki, float _Kd)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
}

void PidController::getPidValues(float &_Kp, float &_Ki, float &_Kd) const
{
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;
}

void PidController::updatePidValue(uint8_t num, int8_t steps)
{
    float *output;
    float value;
    switch(num) {
        case 0:
            output = &Kp;
            value = 0.1;
            break;
        case 1:
            output = &Ki;
            value = 0.00001;
            break;
        case 2:
            output = &Kd;
            value = 0.0001;
            break;
        default:
            return;
    }
    *output += value * steps;
}

void PidController::printValues(Print &buffer) const
//void PidController::printValues(char *buffer, uint8_t len) const
{
    //PrintBuffer buf(buffer, len);
    buffer.print(F("PID "));
    buffer.print(Kp, 1);
    buffer.print(' ');
    buffer.print(Ki, 4);
    buffer.print(' ');
    buffer.println(Kd, 3);
}

void PidController::reset()
{
    duty_cycle = VELOCITY_START_DUTY_CYCLE;
    integral = 0;
    previous_error = 0;
    last_update.start();
}

void PidController::update()
{
    auto _micros = micros();
    float delta_t = last_update.getTime(_micros) * dtMultiplier;
#if DEBUG_PID_CONTROLLER
    uint16_t delta_t_short = last_update.getTime(_micros);
#endif
    last_update.start(_micros);

    auto measured_pulse_length = capture_timer_get_micros();
    ui_data.display_pulse_length_integral = (ui_data.display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + measured_pulse_length) / (DISPLAY_RPM_MULTIPLIER + 1);

    if (data.motor_state == MotorStateEnum::ON) {
        int32_t error = measured_pulse_length - set_point_rpm_pulse_length; // values are inverted
#if DEBUG
        if (abs(error) > 1000000) {
            Serial_printf_P(PSTR("ERR %lu, %lu, %u\n"), error, measured_pulse_length, set_point_rpm_pulse_length);
            motor_stop(ERROR);
        }
#endif

        integral = integral + error * delta_t;
        float derivative = (error - previous_error) / delta_t;

        int32_t output = (Kp * error + Ki * integral + Kd * derivative) * outputMultiplier;
//        int32_t output = (Kp * error + Ki * integral + Kd * derivative) * voltage_multiplier;
#if DEBUG
        if (abs(output) > 10000) {
            Serial_printf_P(PSTR("OVF %ld\n"), output);
        }
#endif
        previous_error = error;

        duty_cycle = max(MIN_DUTY_CYCLE, min(output, MAX_DUTY_CYCLE));
        ui_data.display_duty_cycle_integral = (ui_data.display_duty_cycle_integral * DISPLAY_DUTY_CYCLE_MULTIPLIER + duty_cycle) / (DISPLAY_DUTY_CYCLE_MULTIPLIER + 1);
        set_motor_speed(duty_cycle);
    }

#if DEBUG_PID_CONTROLLER
    if (pid_enable_serial_output && pid_test_timer_start) {
        if (pid_test_counter++ == 0) {
            Serial.println(F("---PID_TEST_START---"));
            Serial_printf_P(PSTR("PARAM=%x,Kp=%f,Ki=%f,Kd=%f\n"), set_point_rpm_pulse_length, Kp, Ki, Kd);
        }
        auto millis_diff = millis() - pid_test_timer_start;
        Serial.print(delta_t_short);
        Serial.write(',');
        Serial.print(measured_pulse_length);
        auto sp = RPM_SENSE_RPM_TO_US(pid_test_set_points[(millis_diff / (pid_test_duration / PID_TEST_SET_POINTS)) % PID_TEST_SET_POINTS]);
        if (sp != set_point_rpm_pulse_length) {
            set_point_rpm_pulse_length = sp;
            Serial.write(',');
            Serial.print(sp);
        }
        Serial.println();
    }
#endif
}
