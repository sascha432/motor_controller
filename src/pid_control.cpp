/**
 * Author: sascha_lammers@gmx.de
 */

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

// PID controller
PidController_t pid = { 20, 0.0035, 0.055 };
const float pid_dt_mul = 1 / 1000000.0 * 1;
const float output_multiplier = 0.042;
// float voltage_multiplier = 1.0;
// long previous_error;
// MicrosTimer pid_last_update;
// uint16_t set_point_rpm_pulse_length = 0;
// uint8_t duty_cycle = 0;
#if DEBUG
bool pid_enable_serial_output = false;
#endif

void reset_pid() {
    pid.duty_cycle = MIN_DUTY_CYCLE;
    pid.integral = 0;
    pid.previous_error = 0;
    pid.last_update.start();
}

void update_pid_controller() {

    auto _micros = micros();
    float delta_t = pid.last_update.getTime(_micros) * pid_dt_mul;
#if DEBUG_PID_CONTROLLER
    uint16_t delta_t_short = pid_last_update.getTime(_micros);
#endif
    pid.last_update.start(_micros);

    auto measured_pulse_length = capture_timer_get_micros();
    ui_data.display_pulse_length_integral = (ui_data.display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + measured_pulse_length) / (DISPLAY_RPM_MULTIPLIER + 1);

    if (data.motor_state == ON) {
        int32_t error = measured_pulse_length - pid.set_point_rpm_pulse_length; // values are inverted
#if DEBUG
        if (abs(error) > 1000000) {
            Serial_printf_P(PSTR("ERR %lu, %lu, %u\n"), error, measured_pulse_length, set_point_rpm_pulse_length);
            motor_stop(ERROR);
        }
#endif

        pid.integral = pid.integral + error * delta_t;
        float derivative = (error - pid.previous_error) / delta_t;

        int32_t output = (pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative) * output_multiplier;
//        int32_t output = (Kp * error + Ki * integral + Kd * derivative) * voltage_multiplier;
#if DEBUG
        if (abs(output) > 10000) {
            Serial_printf_P(PSTR("OVF %ld\n"), output);
        }
#endif
        pid.previous_error = error;

        pid.duty_cycle = max(MIN_DUTY_CYCLE, min(output, MAX_DUTY_CYCLE));
        ui_data.display_duty_cycle_integral = (ui_data.display_duty_cycle_integral * DISPLAY_DUTY_CYCLE_MULTIPLIER + pid.duty_cycle) / (DISPLAY_DUTY_CYCLE_MULTIPLIER + 1);
        set_motor_speed(pid.duty_cycle);
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

