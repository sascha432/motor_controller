/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"

class Motor {
public:
    Motor();

    void begin();
    void loop();

    void start();
    void stop(MotorStateEnum state = MotorStateEnum::OFF);

    void setSpeed(uint8_t speed);
    // set duty cycle or RPM pulse length depending on the mode
    void updateMotorSpeed();

    void toggleMode();
    void setMode(ControlModeEnum mode);

    inline bool isOn() const {
        return _state == MotorStateEnum::ON;
    }
    inline bool isOff() const {
        return _state == MotorStateEnum::OFF;
    }
    inline MotorStateEnum getState() const {
        return _state;
    }
    inline ControlModeEnum getMode() const {
        return _mode;
    }

    inline bool isVelocityMode() const {
        return _mode == ControlModeEnum::PID;
    }
    inline bool isDutyCycleMode() const {
        return _mode == ControlModeEnum::DUTY_CYCLE;
    }

    inline uint8_t getMaxDutyCycle() const {
        return _maxPWM;
    }
    void setMaxDutyCycle(uint8_t maxPWM);

    inline uint8_t getDutyCycle() const {
        return _dutyCycle;
    }
    inline void setDutyCycle(uint8_t dutyCycle) {
        _dutyCycle = dutyCycle;
    }

    inline uint16_t getMaxStallTime() const {
        return _maxStallTime;
    }
    void setMaxStallTime(uint16_t maxStallTime);

    inline bool isBrakeEnabled() const {
        return _brake;
    }
    void enableBrake(bool enable);

private:
    // set pulse length in velocity mode from set point/poti value
    void _calcPulseLength();

private:
    ControlModeEnum _mode;
    volatile MotorStateEnum _state;
    uint32_t _startTime;
    uint8_t _maxPWM;
    uint8_t _dutyCycle;
    uint16_t _maxStallTime;
    bool _brake;
};

extern Motor motor;
