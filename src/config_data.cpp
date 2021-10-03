/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "rpm_sensing.h"
#include "pid_control.h"
#include "motor.h"
#include "current_limit.h"
#include "config_data.h"

ConfigData::ConfigData() :
    pid_config(PidConfigEnum::OFF),
    #if HAVE_RPM_PER_VOLT
        rpm_per_volt(300),
    #endif
    #if HAVE_LED
        _ledBrightness(30 * 255 / LED_MAX_PWM),
        _ledBrightnessPwm(0),
        _ledFadeTimer(0),
    #endif
    _setPointDutyCycle(30 * 100 / MAX_DUTY_CYCLE),
    _setPointRpm(1000),
    _setPointTicks(RPM_SENSE_RPM_TO_TICKS(1000)),
    _motorStatus(MotorStatusEnum::WATT)
{
}

EEPROMData &EEPROMData::operator=(const ConfigData &data)
{
    control_mode = motor.getMode();

    set_point_rpm = data.getSetPointRPM();
    set_point_pwm = data.getSetPointDutyCycle();
    #if HAVE_LED
        _ledBrightness = data.getLedBrightness();
    #endif
    #if HAVE_CURRENT_LIMIT
        current_limit = ::current_limit.getLimit();
    #endif
    brake_enabled = motor.isBrakeEnabled();
    max_stall_time = motor.getMaxStallTime();
    max_pwm = motor.getMaxDutyCycle();
    #if HAVE_RPM_PER_VOLT
        rpm_per_volt = data.getRpmPerVolt();
    #endif
    _motorStatus = data._motorStatus;
    return *this;
}

ConfigData &ConfigData::operator=(const EEPROMData &eeprom_data)
{
    setSetPointDutyCycle(eeprom_data.set_point_pwm);
    setSetPointRPM(eeprom_data.set_point_rpm);
    #if HAVE_LED
        _ledBrightness = eeprom_data._ledBrightness;
        _ledBrightnessPwm = 0;
        _ledFadeTimer = 0;
    #endif
    #if HAVE_CURRENT_LIMIT
        current_limit.setLimit(eeprom_data.current_limit);
    #endif
    motor.setMaxStallTime(eeprom_data.max_stall_time);
    motor.enableBrake(eeprom_data.brake_enabled);
    motor.setMaxDutyCycle(eeprom_data.max_pwm);
    motor.setMode(eeprom_data.control_mode);
    #if HAVE_RPM_PER_VOLT
        data.setRpmPerVolt(eeprom_data.rpm_per_volt);
    #endif
    data._motorStatus = _motorStatus;
    return *this;
}

void ConfigData::changeSetPoint(int16_t value)
{
    if (motor.isVelocityMode()) {
        setSetPointRPM(getSetPointRPM() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED));
    }
    else {
        setSetPointDutyCycle(std::clamp<int16_t>(getSetPointDutyCycle() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE));
        if (motor.isOn()) {
            motor.setSpeed(getSetPointDutyCycle());
        }
    }
}

#if HAVE_LED

bool ConfigData::updateLedBrightness()
{
    if (_ledBrightnessPwm != _ledBrightness) {
        if (_ledBrightnessPwm < _ledBrightness) {
            _ledBrightnessPwm++;
        }
        else if (_ledBrightnessPwm > _ledBrightness) {
            _ledBrightnessPwm--;
        }
    }
    analogWriteLedPwm(_ledBrightnessPwm);
    return _ledBrightnessPwm != _ledBrightness;
}

#endif

void ConfigData::loop()
{
#if HAVE_LED
    auto millis = millis16();
    if (millis - _ledFadeTimer >= LED_FADE_TIME) {
        _ledFadeTimer = millis;
        updateLedBrightness();
    }
#endif
}
