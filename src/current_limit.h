/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include "main.h"

class CurrentLimit {
public:
    // interval to ramp up the duty cycle after the current limited had been tripped
    static constexpr uint16_t kCurrentLimitMicros = 128;
    static constexpr uint16_t kCurrentLimitTicks = TIMER1_TICKS_PER_US * kCurrentLimitMicros;

    static constexpr uint8_t kCurrentLimitShift = 4;
    static constexpr uint16_t kCurrentLimitSteps = (1 << kCurrentLimitShift);
    static constexpr uint8_t kCurrentLimitMaxMultiplier = kCurrentLimitSteps - 1;
    static_assert(kCurrentLimitSteps <= 256, "limited to 256 steps");
    static_assert(kCurrentLimitSteps >= 4, "at least 4 steps are required");

    // total ramp up time in milliseconds
    static constexpr float kCurrentLimitRampupTimeMillis = kCurrentLimitMicros / 1000.0 * kCurrentLimitSteps;

public:
    CurrentLimit();

    void begin();

    void enable();
    void disable();
    uint8_t getDutyCycle(uint8_t duty_cycle);

    bool isDisabled() const;
    uint8_t getLimit() const;
    void setLimit(uint8_t limit);

#if HAVE_CURRENT_LIMIT

    void checkCurrentLimit(bool state);
    void timer1CompareMatchA();
    bool isLimitActive() const;

private:
    enum class CurrentLimitStateEnum : uint8_t {
        DISABLED,
        NOT_TRIPPED,
        SIGNAL_HIGH,
        SIGNAL_LOW
    };

    void _resetDutyCycle();

    volatile uint8_t _limit;
    volatile uint8_t _limitMultiplier;
    volatile CurrentLimitStateEnum _state;
#endif
};

extern CurrentLimit current_limit;

#if HAVE_CURRENT_LIMIT

#include "motor.h"

inline CurrentLimit::CurrentLimit() :
    _limit(CURRENT_LIMIT_DISABLED),
    _limitMultiplier(kCurrentLimitMaxMultiplier),
    _state(CurrentLimitStateEnum::NOT_TRIPPED)
{
}

inline void CurrentLimit::begin()
{
    setupCurrentLimitPwm();
    setupCurrentLimitLed();

    #if PIN_CURRENT_LIMIT_OVERRIDE_PORT
        sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
        sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    #endif

    cbi(PIN_CURRENT_LIMIT_INDICATOR_PORT, PIN_CURRENT_LIMIT_INDICATOR_BIT);
    cbi(PIN_CURRENT_LIMIT_INDICATOR_DDR, PIN_CURRENT_LIMIT_INDICATOR_BIT);
}

inline void CurrentLimit::enable()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setCurrentLimitLedOff();
        _state = (_limit == CURRENT_LIMIT_DISABLED) ? CurrentLimitStateEnum::DISABLED : CurrentLimitStateEnum::NOT_TRIPPED;
        _limitMultiplier = kCurrentLimitMaxMultiplier;
        TIMSK1 &= ~_BV(OCIE1A);
    }
}

inline void CurrentLimit::disable()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setCurrentLimitLedOff();
        _state = CurrentLimitStateEnum::DISABLED;
        _limitMultiplier = kCurrentLimitMaxMultiplier;
        TIMSK1 &= ~_BV(OCIE1A);
    }
}

inline bool CurrentLimit::isDisabled() const
{
    return _limit == CURRENT_LIMIT_DISABLED;
}

inline uint8_t CurrentLimit::getLimit() const
{
    return _limit;
}

inline void CurrentLimit::setLimit(uint8_t limit)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _limitMultiplier = kCurrentLimitMaxMultiplier;
        _limit = limit;
        if (_limit == CURRENT_LIMIT_DISABLED) {
            _state = CurrentLimitStateEnum::DISABLED;
        }
        else {
            _state = CurrentLimitStateEnum::NOT_TRIPPED;
        }
        analogWriteCurrentLimitPwm(_limit);

        #if PIN_CURRENT_LIMIT_OVERRIDE_PORT
            if (_limit == CURRENT_LIMIT_DISABLED) {
                // set comparator vref to 5V >1000A
                sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
                sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
            }
            else {
                // floating = use current limit
                cbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
                cbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
            }
        #endif
    }
}

inline bool CurrentLimit::isLimitActive() const
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _state > CurrentLimitStateEnum::NOT_TRIPPED;
    }
    __builtin_unreachable();
}


inline void CurrentLimit::checkCurrentLimit(bool tripped)
{
    // check if limit is enabled
    switch(_state) {
        case CurrentLimitStateEnum::DISABLED:
            return;
        case CurrentLimitStateEnum::SIGNAL_HIGH:
            if (tripped) {
                _limitMultiplier = 0;
            }
            else {
                _state = CurrentLimitStateEnum::SIGNAL_LOW;
            }
            break;
        case CurrentLimitStateEnum::SIGNAL_LOW:
        case CurrentLimitStateEnum::NOT_TRIPPED:
            if (tripped) {
                if (motor.isOn()) {
                    setMotorPWM_timer(CURRENT_LIMIT_MIN_DUTY_CYCLE);
                }
                _limitMultiplier = 0;
                setCurrentLimitLedOn();
                _state = CurrentLimitStateEnum::SIGNAL_HIGH;
            }
            break;
    }
    OCR1A = TCNT1 + kCurrentLimitTicks;
    TIMSK1 |= _BV(OCIE1A);
}

inline void CurrentLimit::_resetDutyCycle()
{
    // turn off timer
    // TIMSK1 &= ~_BV(OCIE1A);
    // clear state
    // _state = CurrentLimitStateEnum::NOT_TRIPPED;
    // _limitMultiplier = kCurrentLimitMaxMultiplier;
    enable();
    if (motor.isOn()) {
        setMotorPWM_timer(getDutyCycle(motor.getDutyCycle()));
    }
}

inline uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle)
{
    if (_limitMultiplier == 0) {
        return CURRENT_LIMIT_MIN_DUTY_CYCLE;
    }
    if (_limitMultiplier == kCurrentLimitMaxMultiplier) {
        return duty_cycle;
    }
    return std::max(CURRENT_LIMIT_MIN_DUTY_CYCLE, (motor.getDutyCycle() * (_limitMultiplier + 1)) >> kCurrentLimitShift);
}

inline void CurrentLimit::timer1CompareMatchA()
{
    // this method gets executed every kCurrentLimitTicks once the limit has been tripped
    switch(_state) {
        case CurrentLimitStateEnum::SIGNAL_LOW:
            if (_limitMultiplier < kCurrentLimitMaxMultiplier) {
                // increase multiplier until it reaches 255/100%
                _limitMultiplier++;
                if (motor.isOn()) {
                    setMotorPWM_timer(getDutyCycle(motor.getDutyCycle()));
                }
            }
            else {
                // marked as not tripped
                _resetDutyCycle();
                return;
            }
            break;
        case CurrentLimitStateEnum::SIGNAL_HIGH:
            if (_limitMultiplier) {
sei();
Serial.println("C1");
            }
            _limitMultiplier = 0;
            if (motor.isOn()) {
                setMotorPWM_timer(CURRENT_LIMIT_MIN_DUTY_CYCLE);
            }
            break;
        case CurrentLimitStateEnum::NOT_TRIPPED:
            sei();
            Serial.println("C0");
            break;
        case CurrentLimitStateEnum::DISABLED:
            _resetDutyCycle();
            break;
    }
    // reschedule
    OCR1A = TCNT1 + kCurrentLimitTicks;
}

// #else

// inline CurrentLimit::CurrentLimit()
// {
// }

// inline void CurrentLimit::begin()
// {
//     #ifdef PIN_CURRENT_LIMIT_PWM
//         analogWriteCurrentLimitPwm(255);
//     #endif

//     // #ifdef PIN_CURRENT_LIMIT_OVERRIDE_PORT
//     //     sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
//     //     sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
//     // #endif
// }

// inline void CurrentLimit::disable()
// {
// }

// inline void CurrentLimit::enable(bool state)
// {
// }

// inline uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle, bool reset)
// {
// }

// inline bool CurrentLimit::isDisabled() const
// {
//     return true;
// }

// inline uint8_t CurrentLimit::getLimit() const
// {
//     return 0;
// }

// inline void CurrentLimit::setLimit(uint8_t limit)
// {
// }

// inline void CurrentLimit::updateLimit()
// {
// }

#endif