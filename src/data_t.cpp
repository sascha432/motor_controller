/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "rpm_sensing.h"
#include "pid_control.h"
#include "motor.h"
#include "current_limit.h"

Data_t::Data_t() :
    pid_config(PidConfigEnum::OFF),
    led_brightness(50 * 100 / LED_MAX_PWM),
    rpm_sense_average(0),
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
    eeprom_data.control_mode = motor.getMode();
    eeprom_data.set_point_input_velocity = set_point_input_velocity;
    eeprom_data.set_point_input_pwm = set_point_input_pwm;
    eeprom_data.led_brightness = led_brightness;
    eeprom_data.current_limit = current_limit.getLimit();
    eeprom_data.brake_enabled = motor.isBrakeEnabled();
    eeprom_data.max_stall_time = motor.getMaxStallTime();
    eeprom_data.max_pwm = motor.getMaxDutyCycle();
}

void Data_t::copyFrom(const EEPROMData_t &eeprom_data)
{
    set_point_input_pwm = eeprom_data.set_point_input_pwm;
    set_point_input_velocity = eeprom_data.set_point_input_velocity;
    led_brightness = eeprom_data.led_brightness;
    led_brightness_pwm = 0;
    current_limit.setLimit(eeprom_data.current_limit);
    motor.setMaxStallTime(eeprom_data.max_stall_time);
    motor.enableBrake(eeprom_data.brake_enabled);
    motor.setMaxDutyCycle(eeprom_data.max_pwm);
    motor.setMode(eeprom_data.control_mode);
}

uint8_t Data_t::getSetPoint() const
{
    return motor.isVelocityMode() ? set_point_input_velocity : set_point_input_pwm;
}

void Data_t::setSetPoint(uint8_t value)
{
    if (motor.isVelocityMode()) {
        set_point_input_velocity = value;
    }
    else {
        set_point_input_pwm = value;
    }
}

void Data_t::changeSetPoint(int8_t value)
{
    uint8_t &set_point_input = motor.isVelocityMode() ? set_point_input_velocity : set_point_input_pwm;
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
