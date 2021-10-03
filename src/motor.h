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

    void toggleMode();
    void setMode(ControlModeEnum mode);

    operator bool() const;
    bool isOn() const;
    bool isOff() const;
    MotorStateEnum getState() const;
    ControlModeEnum getMode() const;

    bool isVelocityMode() const;
    bool isDutyCycleMode() const;

    uint8_t getMaxDutyCycle() const;
    void setMaxDutyCycle(uint8_t maxPWM);

    // uint8_t getDutyCycle() const;
    // void setDutyCycle(uint8_t dutyCycle);

    uint16_t getMaxStallTime() const;
    void setMaxStallTime(uint16_t maxStallTime);

    bool isBrakeEnabled() const;
    void enableBrake(bool enable);
    void setBrake(bool state);

    static const __FlashStringHelper *_getMode(ControlModeEnum mode);
    static const __FlashStringHelper *_getState(MotorStateEnum state);

private:
    // set pulse length in velocity mode from set point/poti value
    void _calcPulseLength();

public:
    uint32_t _startTime;

private:
    ControlModeEnum _mode;
    volatile MotorStateEnum _state;
    uint8_t _maxPWM;
    // uint8_t _dutyCycle;
    uint16_t _maxStallTime;
    bool _brake;
};

extern Motor motor;

inline Motor::Motor() :
    _mode(ControlModeEnum::PID),
    _state(MotorStateEnum::OFF),
    _maxPWM(MAX_DUTY_CYCLE),
    // _dutyCycle(0),
    _maxStallTime(1000),
    _brake(true)
{
}

inline void Motor::begin()
{
    setupMotorPwm();
    setupBrake();
}

inline Motor::operator bool() const
{
    return _state == MotorStateEnum::ON;
}

inline bool Motor::isOn() const
{
    return _state == MotorStateEnum::ON;
}

inline bool Motor::isOff() const
{
    return _state == MotorStateEnum::OFF;
}

inline MotorStateEnum Motor::getState() const
{
    return _state;
}

inline ControlModeEnum Motor::getMode() const
{
    return _mode;
}

inline bool Motor::isVelocityMode() const
{
    return _mode == ControlModeEnum::PID;
}

inline bool Motor::isDutyCycleMode() const
{
    return _mode == ControlModeEnum::DUTY_CYCLE;
}

inline uint8_t Motor::getMaxDutyCycle() const
{
    return _maxPWM;
}

// inline uint8_t getMotorPWM_timer();
// inline void setMotorPWM(uint8_t pwm);
// inline void stopMotor();

// inline uint8_t Motor::getDutyCycle() const
// {
//     return getMotorPWM_timer();
// }

// inline void Motor::setDutyCycle(uint8_t dutyCycle)
// {
//     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//         if (dutyCycle) {
//             setMotorPWM(dutyCycle);
//         }
//         else {
//             stopMotor();
//         }
//     }
//     // _dutyCycle = dutyCycle;
// }

inline uint16_t Motor::getMaxStallTime() const
{
    return _maxStallTime;
}

inline void Motor::setMaxDutyCycle(uint8_t maxPWM)
{
    _maxPWM = maxPWM;
}

inline void Motor::setMaxStallTime(uint16_t maxStallTime)
{
    _maxStallTime = maxStallTime;
}

inline bool Motor::isBrakeEnabled() const
{
    return _brake;
}

inline void Motor::enableBrake(bool enable)
{
    _brake = enable;
    setBrake(false);
}

inline void Motor::setBrake(bool state)
{
    if (state) {
        if (isBrakeEnabled()) {
            #if DEBUG_MOTOR_SPEED
                Serial.printf_P(PSTR("%lu: set brake high\n"), millis());
            #endif
            stopMotorAtomic();
            // wait for the mosfet to be turned off
            delayMicroseconds(500);
            // short motor
            setBrakeOn();
        }
        else {
            #if DEBUG_MOTOR_SPEED
                Serial.printf_P(PSTR("%lu: set brake high skipped\n"), millis());
            #endif
            stopMotorAtomic();
            setBrakeOff();
        }
    }
    else {
        #if DEBUG_MOTOR_SPEED
            Serial.printf_P(PSTR("%lu: set brake low\n"), millis());
        #endif
        setBrakeOff();
        delayMicroseconds(50);
    }
}

inline const __FlashStringHelper *Motor::_getMode(ControlModeEnum mode)
{
    switch(mode) {
        case ControlModeEnum::DUTY_CYCLE:
            return F("DC");
        case ControlModeEnum::PID:
            return F("PID");
    }
    return F("<ERR>");
}

inline const __FlashStringHelper *Motor::_getState(MotorStateEnum state) {
    switch(state) {
        case MotorStateEnum::ON:
            return F("ON");
        case MotorStateEnum::OFF:
            return F("OFF");
        case MotorStateEnum::STALLED:
            return F("STALL");
        case MotorStateEnum::ERROR:
            return F("ERROR");
        case MotorStateEnum::BRAKING:
            return F("BRAKING");
    }
    return F("<ERR>");
}

inline void Motor::toggleMode()
{
    if (isVelocityMode()) {
        setMode(ControlModeEnum::DUTY_CYCLE);
    }
    else {
        setMode(ControlModeEnum::PID);
    }
}

inline void Motor::setMode(ControlModeEnum mode)
{
    if (_state != MotorStateEnum::ON) {
        #if DEBUG_MOTOR_SPEED
            Serial.printf_P(PSTR("set_mode=%u\n"), (int)_mode);
        #endif
        _mode = mode;
    }
}
