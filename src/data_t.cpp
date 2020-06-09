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
    led_brightness(50 * 100 / LED_MAX_PWM),
    current_limit(CURRENT_LIMIT_DISABLED),
    max_stall_time(1000),
    max_pwm(MAX_DUTY_CYCLE),
    rpm_sense_average(4),
#if HAVE_LED_FADING
    led_fade_timer(0),
#endif
    led_brightness_pwm(0),
    set_point_input_velocity(RPM_TO_POTI(1000)),
    set_point_input_pwm(30 * 100 / MAX_DUTY_CYCLE)
{
}

void Data_t::copyTo(EEPROMData_t &eeprom_data)
{
    eeprom_data.control_mode = control_mode;
    eeprom_data.set_point_input_velocity = set_point_input_velocity;
    eeprom_data.set_point_input_pwm = set_point_input_pwm;
    eeprom_data.led_brightness = led_brightness;
    eeprom_data.current_limit = current_limit;
    eeprom_data.brake_enabled = brake_enabled;
    eeprom_data.max_stall_time = max_stall_time;
    eeprom_data.max_pwm = max_pwm;
}

void Data_t::copyFrom(const EEPROMData_t &eeprom_data)
{
    set_point_input_pwm = eeprom_data.set_point_input_pwm;
    set_point_input_velocity = eeprom_data.set_point_input_velocity;
    led_brightness = eeprom_data.led_brightness;
    led_brightness_pwm = 0;
    current_limit = eeprom_data.current_limit;
    max_stall_time = eeprom_data.max_stall_time;
    brake_enabled = eeprom_data.brake_enabled;
    max_pwm = max(MIN_DUTY_CYCLE, eeprom_data.max_pwm);

    setControlMode(eeprom_data.control_mode);
}

inline void update_pid_controller()
{
    pid.update();
}

inline void update_duty_cycle()
{
    ui_data.updateDutyCyle();
}

void Data_t::setControlMode(ControlModeEnum mode)
{
    if (motor_state != MotorStateEnum::ON) {
        if (isPID()) {
            capture_timer_set_callback(update_pid_controller);
        }
        else {
            capture_timer_set_callback(update_duty_cycle);
        }
        control_mode = mode;
    }
}

uint8_t Data_t::getSetPoint() const
{
    return isPID() ? set_point_input_velocity : set_point_input_pwm;
}

void Data_t::setSetPoint(uint8_t value)
{
    if (isPID()) {
        set_point_input_velocity = value;
    }
    else {
        set_point_input_pwm = value;
    }
}

void Data_t::changeSetPoint(int8_t value)
{
    uint8_t &set_point_input = isPID() ? set_point_input_velocity : set_point_input_pwm;
    int16_t tmp = set_point_input;
    tmp += value;
    set_point_input = min(POTI_MAX, max(POTI_MIN, tmp));
}


void Data_t::setLedBrightness()
{
#if HAVE_LED_FADING
    if (millis() > led_fade_timer) {
        led_fade_timer = millis() + LED_FADE_TIME;
        if (led_brightness_pwm != led_brightness) {
            if (led_brightness_pwm < led_brightness) {
                led_brightness_pwm++;
            }
            else if (led_brightness_pwm > led_brightness) {
                led_brightness_pwm--;
            }
            if (led_brightness_pwm < LED_MIN_PWM) {
                analogWrite(PIN_LED_DIMMER, 0);
            }
            else {
                analogWrite(PIN_LED_DIMMER, led_brightness_pwm);
            }
        }
    }
#else
    if (led_brightness_pwm != led_brightness) {
        led_brightness_pwm = led_brightness;
        if (led_brightness_pwm < LED_MIN_PWM) {
            analogWrite(PIN_LED_DIMMER, 0);
        }
        else {
            analogWrite(PIN_LED_DIMMER, led_brightness_pwm);
        }
    }
#endif
}
