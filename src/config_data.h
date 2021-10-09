/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

class ConfigData {
public:
    ConfigData();

    ConfigData &operator=(const EEPROMData &data);

private:
    friend class PidController;

    void setSetPointRPM(int16_t rpm);
    void setSetPointDutyCycle(uint8_t pwm);

public:
    uint16_t getSetPointRPM() const;
    int32_t getSetPointRPMTicks() const;

    uint8_t getSetPointDutyCycle() const;
    float getSetPointDutyCyclePercent() const;

    // sets duty cycle or RPM depending on the mode
    // updates the motor or pid controller as well
    void changeSetPoint(int16_t value);

    #if HAVE_RPM_PER_VOLT
        void setRpmPerVolt(uint16_t rpmV);
        uint16_t getRpmPerVolt() const;
    #endif

    MotorStatusEnum getDisplayMotorStatus() const;
    void setDisplayMotorStatus(MotorStatusEnum value);
    void toggleDisplayMotorStatus();

    inline PidConfigEnum &pidConfig() {
        return pid_config;
    }

    void loop();

    #if HAVE_LED

        bool updateLedBrightness();

        void updateLedBrightnessNoDelay() {
            _ledBrightnessPwm = _ledBrightness;
            updateLedBrightness();
        }

        void setLedBrightness(uint8_t value) {
            if (_ledBrightness != value) {
                _ledBrightness = value;
                updateLedBrightness();
            }
        }

        uint8_t getLedBrightness() const {
            return _ledBrightness;
        }

        float getLedBrightessPercent() const {
            static constexpr float kRange = (LED_MAX_PWM - LED_MIN_PWM);
            static constexpr float kOffset = ((LED_MAX_PWM - LED_MIN_PWM) / 100) + 1;
            return ((_ledBrightness - LED_MIN_PWM + kOffset) * 100) / static_cast<float>(kRange + kOffset);
        }

    #endif

private:
    friend EEPROMData;

    PidConfigEnum pid_config;
    #if HAVE_RPM_PER_VOLT
        uint16_t rpm_per_volt;
    #endif
    #if HAVE_LED
        uint8_t _ledBrightness;
        uint8_t _ledBrightnessPwm;
        uint16_t _ledFadeTimer;
    #endif
    uint8_t _setPointDutyCycle;
    uint16_t _setPointRpm;
    int32_t _setPointTicks;
    MotorStatusEnum _motorStatus;
};

inline void ConfigData::setSetPointRPM(int16_t rpm)
{
    _setPointRpm = std::clamp<int16_t>(rpm, RPM_MIN, RPM_MAX);
    _setPointTicks = RPM_SENSE_RPM_TO_TICKS(_setPointRpm);
}

inline uint16_t ConfigData::getSetPointRPM() const
{
    return _setPointRpm;
}

inline int32_t ConfigData::getSetPointRPMTicks() const
{
    return _setPointTicks;
}

inline void ConfigData::setSetPointDutyCycle(uint8_t pwm)
{
    _setPointDutyCycle = pwm;
}

inline uint8_t ConfigData::getSetPointDutyCycle() const
{
    return _setPointDutyCycle;
}

inline float ConfigData::getSetPointDutyCyclePercent() const
{
    return std::clamp<float>((getSetPointDutyCycle() * 100) / static_cast<float>(MAX_DUTY_CYCLE), (MIN_DUTY_CYCLE * 100.0 / MAX_DUTY_CYCLE), 100.0);
}

#if HAVE_RPM_PER_VOLT

inline void ConfigData::setRpmPerVolt(uint16_t rpmV)
{
    rpm_per_volt = rpmV;
}

inline uint16_t ConfigData::getRpmPerVolt() const
{
    return rpm_per_volt;
}

#endif

inline MotorStatusEnum ConfigData::getDisplayMotorStatus() const
{
    return _motorStatus;
}

inline void ConfigData::setDisplayMotorStatus(MotorStatusEnum value)
{
    _motorStatus = value;
}

inline void ConfigData::toggleDisplayMotorStatus()
{
    _motorStatus = static_cast<MotorStatusEnum>((static_cast<uint8_t>(_motorStatus) + 1) % static_cast<uint8_t>(MotorStatusEnum::MAX));
}
