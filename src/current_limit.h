/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include "main.h"

class CurrentLimit {
public:
    CurrentLimit();

    void begin();

    void enable(bool state);
    uint8_t getDutyCycle(uint8_t duty_cycle, bool reset = false);

    bool isDisabled() const;
    uint8_t getLimit() const;
    void setLimit(uint8_t limit);

    void updateLimit();

#if HAVE_CURRENT_LIMIT

    void pinISR(bool state);
    void compareAISR();

private:
    enum class CurrentLimitStateEnum : uint8_t {
        NOT_TRIPPED,
        SIGNAL_HIGH,
        SIGNAL_LOW,
        RESET
    };

    uint8_t _limit;
    volatile uint32_t _timer;
    volatile CurrentLimitStateEnum _state;
    volatile bool _enabled;
#endif
};

extern CurrentLimit current_limit;

#if HAVE_CURRENT_LIMIT

#include "motor.h"

// increase frequency from 4 times per second to 1000 after the limit has been tripped
#define CURRENT_LIMIT_TICKS ((F_CPU / TIMER1_PRESCALER / 1000) - 1)

inline CurrentLimit::CurrentLimit() :
    _limit(CURRENT_LIMIT_DISABLED),
    _timer(0),
    _state(CurrentLimitStateEnum::NOT_TRIPPED),
    _enabled(false)
{
}

inline void CurrentLimit::begin()
{
    setupCurrentLimitPwm();

    sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);

    cbi(PIN_CURRENT_LIMIT_INDICATOR_PORT, PIN_CURRENT_LIMIT_INDICATOR_BIT);
    cbi(PIN_CURRENT_LIMIT_INDICATOR_DDR, PIN_CURRENT_LIMIT_INDICATOR_BIT);

    cbi(PIN_CURRENT_LIMIT_LED_PORT, PIN_CURRENT_LIMIT_LED_BIT);
    sbi(PIN_CURRENT_LIMIT_LED_DDR, PIN_CURRENT_LIMIT_LED_BIT);
}

inline void CurrentLimit::enable(bool state)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (_limit == CURRENT_LIMIT_DISABLED) {
            state = false;
        }
        _enabled = state;
        _state = CurrentLimitStateEnum::NOT_TRIPPED;
        _timer = 0;
        if (state) {
            TIMSK1 |= _BV(OCIE1A);
        }
        else {
            TIMSK1 &= ~_BV(OCIE1A);
        }
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
    if (limit == CURRENT_LIMIT_DISABLED) {
        _limit = CURRENT_LIMIT_DISABLED;
    }
    else {
        _limit = limit;
    }
}

inline void CurrentLimit::pinISR(bool state)
{
    if (_enabled) { // state changed
        if (state) { // rising edge, store time if not set
            if (_state != CurrentLimitStateEnum::SIGNAL_HIGH) {
                _timer = micros();
                _state = CurrentLimitStateEnum::SIGNAL_HIGH;
                if (motor.isOn()) {
                    setMotorPWM_timer(CURRENT_LIMIT_MIN_DUTY_CYCLE);
                }
                if (ui_data.display_current_limit_timer == 0) {
                    ui_data.display_current_limit_timer = millis();
                    if (motor.isOn()) {
                        ui_data.refresh_timer = ui_data.display_current_limit_timer + 1000; // disable display, i2c causes interferences
                    }
                }
                OCR1A = TCNT1 + CURRENT_LIMIT_TICKS;
                setCurrentLimitLedOn();
            }
        }
    }
}

inline void CurrentLimit::compareAISR()
{
    if (_state != CurrentLimitStateEnum::NOT_TRIPPED) {
        // in PWM mode set motor speed continuously to update the duty cycle
        auto dc = getDutyCycle(motor.getDutyCycle(), true);
        if (motor.isOn()) {
            setMotorPWM_timer(dc);
        }
        OCR1A = TCNT1 + CURRENT_LIMIT_TICKS;
    }
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

    #ifdef PIN_CURRENT_LIMIT_OVERRIDE_PORT
        sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
        sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    #endif
}

inline void CurrentLimit::disable()
{
}

inline void CurrentLimit::enable(bool state)
{
}

inline uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle, bool reset)
{
}

inline bool CurrentLimit::isDisabled() const
{
    return true;
}

inline uint8_t CurrentLimit::getLimit() const
{
    return 0;
}

inline void CurrentLimit::setLimit(uint8_t limit)
{
}

inline void CurrentLimit::updateLimit()
{
}

#endif