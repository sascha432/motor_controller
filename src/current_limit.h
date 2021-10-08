/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include "main.h"

#if HAVE_CURRENT_LIMIT

//
// once the over current signal triggers the interrupt, the interrupt is disabled and the timer started
// the timer checks the over current signal and ramps up the duty cycle once the signal is off. the interrupt
// is activated again as well to stop the motor quickly if the limit gets triggered between the timer
// interval
//

class CurrentLimit {
public:
    // interval to ramp up the duty cycle after the current limited had been tripped
    static constexpr uint16_t kCurrentLimitMicros = 128;
    static constexpr uint16_t kCurrentLimitTicks = Timer1::kTicksPerMicrosecond * kCurrentLimitMicros;

    // ramp up current slowly to avoid trigger the current limit again
    static constexpr uint8_t kCurrentLimitShift = 2;
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

    // check if the limit is enabled
    bool isEnabled() const;

    // get vref pwm value
    uint8_t getLimit() const;
    float getLimitAmps() const;

    // set vref pwm
    void setLimit(uint8_t limit);

    // callback for the ISR of the over current pin
    void currentLimitTripped();

    // timer ISR that handles ramping up the duty cycle after over current
    void timer1CompareMatchA();

    // returns true if the limit has been triggered
    bool isLimitActive() const;

private:
    enum class CurrentLimitStateEnum : uint8_t {
        NOT_TRIPPED,
        SIGNAL_HIGH,
        SIGNAL_LOW
    };

    uint8_t _getDutyCycle();
    void _enableTimer();
    void _disableTimer();
    void _resetTimer();

    volatile uint8_t _limit;
    volatile uint8_t _limitMultiplier;
    volatile CurrentLimitStateEnum _state;
};

extern CurrentLimit current_limit;

#include "motor.h"

inline CurrentLimit::CurrentLimit() :
    _limit(ILimit::kDisabled),
    _limitMultiplier(kCurrentLimitMaxMultiplier),
    _state(CurrentLimitStateEnum::NOT_TRIPPED)
{
}

inline void CurrentLimit::begin()
{
    setupCurrentLimitPwm();
    setupCurrentLimitLed();

    asm volatile ("cbi %0, %1" :: "I" (SFR::Pin<PIN_CURRENT_LIMIT_INDICATOR>::DDR_IO_ADDR()), "I" (SFR::Pin<PIN_CURRENT_LIMIT_INDICATOR>::PINbit()));      // pin mode

}

inline void CurrentLimit::enable()
{
    setCurrentLimitLedOff();
    _limitMultiplier = kCurrentLimitMaxMultiplier;
    _disableTimer();
    if (_limit == ILimit::kDisabled) {
        currentLimitDisableInterrupt();
    }
    else {
        _state = CurrentLimitStateEnum::NOT_TRIPPED;
        currentLimitEnableInterrupt();
    }
}

inline void CurrentLimit::disable()
{
    setCurrentLimitLedOff();
    _disableTimer();
    currentLimitDisableInterrupt();
}

inline bool CurrentLimit::isDisabled() const
{
    return _limit == ILimit::kDisabled;
}

inline bool CurrentLimit::isEnabled() const
{
    return _limit != ILimit::kDisabled;
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
        if (_limit != ILimit::kDisabled) {
            _state = CurrentLimitStateEnum::NOT_TRIPPED;
        }
        analogWriteCurrentLimitPwm(_limit);
    }
}

inline bool CurrentLimit::isLimitActive() const
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _state > CurrentLimitStateEnum::NOT_TRIPPED;
    }
    __builtin_unreachable();
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

inline uint8_t CurrentLimit::getDutyCycle(uint8_t dutyCycle)
{
    if (_limit == ILimit::kDisabled) {
        return dutyCycle;
    }
    if (_limitMultiplier == 0) {
        return ILimit::kLimitedDutyCycle;
    }
    if (_limitMultiplier == kCurrentLimitMaxMultiplier) {
        return dutyCycle;
    }
    return std::max<uint8_t>(ILimit::kLimitedDutyCycle, (dutyCycle * (_limitMultiplier + 1)) >> kCurrentLimitShift);
}

// current limit ISR
inline void CurrentLimit::currentLimitTripped()
{
    setMotorPWM_timer(ILimit::kLimitedDutyCycle);
    // turn interrupt off, the timer will check the state
    currentLimitDisableInterrupt();
    setCurrentLimitLedOn();
    _limitMultiplier = 0;
    _state = CurrentLimitStateEnum::SIGNAL_HIGH;
    _resetTimer();
    _enableTimer();
}

inline void CurrentLimit::timer1CompareMatchA()
{
    if (isCurrentLimitTripped()) {
        currentLimitTripped();
    }
    else {
        // enable interrupt again
        currentLimitEnableInterrupt();
        if (_limitMultiplier < kCurrentLimitMaxMultiplier) {
            _state = CurrentLimitStateEnum::SIGNAL_LOW;
            // increase multiplier until it reaches kCurrentLimitMaxMultiplier/100%
            _limitMultiplier++;
            // set limited current
            setMotorPWM_timer(getDutyCycle(_getDutyCycle()));
            _resetTimer();
        }
        else {
            // marked as not tripped and restore pwm value
            _state = CurrentLimitStateEnum::NOT_TRIPPED;
            setCurrentLimitLedOff();
            _disableTimer();
            if (motor.isOn()) {
                // restore pwm value if the motor is on
                setMotorPWM(_getDutyCycle());
            }
        }
    }
}

#endif