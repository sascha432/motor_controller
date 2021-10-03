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
    static constexpr uint16_t kCurrentLimitTicks = Timer1::kTicksPerMicrosecond * kCurrentLimitMicros;

    // while the comparator will limit at the set current (measured over 1-2ms depending on the RC filter), the
    // average current will be less when running into the limit before the PWM has been ramped up again
    static constexpr uint8_t kCurrentLimitShift = 1;
    static constexpr uint16_t kCurrentLimitSteps = (1 << kCurrentLimitShift);
    static constexpr uint8_t kCurrentLimitMaxMultiplier = kCurrentLimitSteps - 1;
    static_assert(kCurrentLimitSteps <= 256, "limited to 256 steps");
    static_assert(kCurrentLimitSteps >= 2, "at least 2 steps are required");

    // total ramp up time in milliseconds
    static constexpr float kCurrentLimitRampupTimeMillis = kCurrentLimitMicros / 1000.0 * kCurrentLimitSteps;

public:
    CurrentLimit();

    // initialize current limit
    void begin();

    // enable current limit
    void enable();

    // disable current limit
    void disable();

    // get duty cycle with current limit
    uint8_t getDutyCycle(uint8_t duty_cycle);

    // check if the limit is disabled
    bool isDisabled() const;

    // get vref pwm value
    uint8_t getLimit() const;
    float getLimitAmps() const;

    // set vref pwm
    void setLimit(uint8_t limit);

    // callback for the ISR of the overcurrent pin
    void checkCurrentLimit(bool isTripped);

    // timer ISR that handles ramping up the duty cycle after over current
    void timer1CompareMatchA();

    // returns true if the limit has been triggered
    bool isLimitActive() const;

private:
    enum class CurrentLimitStateEnum : uint8_t {
        DISABLED,
        NOT_TRIPPED,
        SIGNAL_HIGH,
        SIGNAL_LOW
    };

    uint8_t _getDutyCycle();
    void _resetDutyCycle();
    void _enableTimer();
    void _disableTimer();
    void _resetTimer();

    volatile uint8_t _limit;
    volatile uint8_t _limitMultiplier;
    volatile CurrentLimitStateEnum _state;
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
        _disableTimer();
    }
}

inline void CurrentLimit::disable()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setCurrentLimitLedOff();
        _state = CurrentLimitStateEnum::DISABLED;
        _limitMultiplier = kCurrentLimitMaxMultiplier;
        _disableTimer();
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

inline float CurrentLimit::getLimitAmps() const
{
    return _limit * DAC::kPwmCurrentMultiplier;
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
    _resetTimer();
    _enableTimer();
}

inline void CurrentLimit::_enableTimer()
{
    TIMSK1 |= _BV(OCIE1A);
}

inline void CurrentLimit::_disableTimer()
{
    TIMSK1 &= ~_BV(OCIE1A);
}

inline void CurrentLimit::_resetTimer()
{
    OCR1A = TCNT1 + kCurrentLimitTicks;
}

inline uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle)
{
    if (_limitMultiplier == 0) {
        return CURRENT_LIMIT_MIN_DUTY_CYCLE;
    }
    if (_limitMultiplier == kCurrentLimitMaxMultiplier) {
        return duty_cycle;
    }
    return std::max(CURRENT_LIMIT_MIN_DUTY_CYCLE, (duty_cycle * (_limitMultiplier + 1)) >> kCurrentLimitShift);
}

inline void CurrentLimit::timer1CompareMatchA()
{
    // this method gets executed every kCurrentLimitTicks once the limit has been tripped
    switch(_state) {
        case CurrentLimitStateEnum::SIGNAL_LOW:
            if (_limitMultiplier < kCurrentLimitMaxMultiplier) {
                // increase multiplier until it reaches kCurrentLimitMaxMultiplier/100%
                _limitMultiplier++;
                if (motor.isOn()) {
                    // set limited current
                    setMotorPWM_timer(getDutyCycle(_getDutyCycle()));
                }
            }
            else {
                // marked as not tripped and restore pwm value
                _resetDutyCycle();
                return;
            }
            break;
        case CurrentLimitStateEnum::SIGNAL_HIGH:
            _limitMultiplier = 0;
            if (motor.isOn()) {
                // set to min. pwm value
                setMotorPWM_timer(CURRENT_LIMIT_MIN_DUTY_CYCLE);
            }
            break;
        case CurrentLimitStateEnum::DISABLED:
            _resetDutyCycle();
            break;
        default:
            break;
    }
    // reschedule
    _resetTimer();
}

#else

inline CurrentLimit::CurrentLimit()
{
}

inline void CurrentLimit::begin()
{
    #ifdef PIN_CURRENT_LIMIT_PWM
        analogWriteCurrentLimitPwm(255);
    #endif

    // #ifdef PIN_CURRENT_LIMIT_OVERRIDE_PORT
    //     sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    //     sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    // #endif
}

// inline void CurrentLimit::disable()
// {
// }

inline void CurrentLimit::enable()
{
}

inline uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle)
{
    return duty_cycle;
}

inline bool CurrentLimit::isDisabled() const
{
    return true;
}

inline uint8_t CurrentLimit::getLimit() const
{
    return 255;
}

inline void CurrentLimit::setLimit(uint8_t limit)
{
}


inline uint8_t CurrentLimit::_getDutyCycle()
{
    return 0;
}

inline void CurrentLimit::_resetDutyCycle()
{
}

#endif