/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "rpm_sensing.h"
#include "pid_control.h"

Data_t::Data_t() :
    control_mode(ControlModeEnum::PID),
    motor_state(MotorStateEnum::OFF),
    motor_start_time(0),
    brake_enaged(false),
    brake_enabled(true),
    pid_config(PidConfigEnum::OFF),
    set_point_input(30 * 100 / MAX_DUTY_CYCLE),
    led_brightness(50 * 100 / LED_MAX_PWM),
    led_brightness_pwm(0),
#if HAVE_LED_FADING
    led_fade_timer(0),
#endif
    current_limit(CURRENT_LIMIT_DISABLED),
    max_stall_time(1000),
    max_pwm(MAX_DUTY_CYCLE),
    rpm_sense_average(4)
{
}

void Data_t::copyTo(EEPROMData_t &eeprom_data)
{
    eeprom_data.control_mode = control_mode;
    eeprom_data.set_point_input = set_point_input;
    eeprom_data.led_brightness = led_brightness;
    eeprom_data.current_limit = current_limit;
    eeprom_data.brake_enabled = brake_enabled;
    eeprom_data.max_stall_time = max_stall_time;
    eeprom_data.max_pwm = max_pwm;
}

void Data_t::copyFrom(const EEPROMData_t &eeprom_data)
{
    control_mode = eeprom_data.control_mode;
    set_point_input = eeprom_data.set_point_input;
    led_brightness = eeprom_data.led_brightness;
    led_brightness_pwm = 0;
    current_limit = eeprom_data.current_limit;
    max_stall_time = eeprom_data.max_stall_time;
    brake_enabled = eeprom_data.brake_enabled;
    max_pwm = max(MIN_DUTY_CYCLE, eeprom_data.max_pwm);
}

inline void update_pid_controller() {
    pid.update();
}

inline void update_duty_cycle() {
    ui_data.updateDutyCyle();
}

void Data_t::setControlMode(ControlModeEnum mode)
{
    if (motor_state != MotorStateEnum::ON) {
        control_mode = mode;
        if (mode == ControlModeEnum::PID) {
            capture_timer_set_callback(update_pid_controller);
        }
        else {
            capture_timer_set_callback(update_duty_cycle);
        }
    }
}

void Data_t::setLedBrightness()
{
#if HAVE_LED_FADING
    if (get_time_diff(led_fade_timer, millis()) > LED_FADE_TIME) {
        led_fade_timer = millis();
        if (!led_brightness_pwm) {
            return;
        }
        else if (led_brightness_pwm < led_brightness) {
            led_brightness_pwm++;
        }
        else if (led_brightness_pwm > led_brightness) {
            led_brightness_pwm--;
        }
        if (led_brightness_pwm < LED_MIN_PWM) {
            analogWrite(PIN_LED_DIMMER, 0);
        }
#if 0
        int pwm = pow(log10(led_brightness_pwm / 5.0f), 2.1f) * 82.898f;
        if (pwm > LED_MAX_PWM) {
            pwm = LED_MAX_PWM;
        }
        analogWrite(PIN_LED_DIMMER, pwm);
#else
        analogWrite(PIN_LED_DIMMER, led_brightness_pwm);
#endif
    }
#else
    if (led_brightness_pwm != led_brightness) {
        if (led_brightness_pwm < LED_MIN_PWM) {
            analogWrite(PIN_LED_DIMMER, 0);
        }
        else {
            analogWrite(PIN_LED_DIMMER, led_brightness_pwm);
        }
    }
#endif
}
