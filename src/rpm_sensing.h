/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include <Arduino.h>

// use rising and falling edge for counting RPM
#ifndef RPM_SENSE_TOGGLE_EDGE
#    define RPM_SENSE_TOGGLE_EDGE 1
#endif

#ifndef RPM_SENSE_PULSES_PER_TURN
// wheel with 120 slots
#    if RPM_SENSE_TOGGLE_EDGE
#       define RPM_SENSE_PULSES_PER_TURN (120 * 2)
#    else
#       define RPM_SENSE_PULSES_PER_TURN 120
#    endif
#endif

// convert rpm to pulse length (µs)
#define RPM_SENSE_RPM_TO_US(rpm) (1000000UL * 60UL / (rpm * (uint32_t)RPM_SENSE_PULSES_PER_TURN))

// convert RPM pulse length (µs) to RPM
#define RPM_SENSE_US_TO_RPM(value) ((1000000UL * 60UL / RPM_SENSE_PULSES_PER_TURN) / value)

// convert RPM pulse length (µs) to Hz
#define RPM_SENSE_US_TO_HZ(value) ((1000000UL / RPM_SENSE_PULSES_PER_TURN) / value)

// convert RPM pulse length (ticks) to Hz
#define RPM_SENSE_TICKS_TO_HZ(value) ((1000000UL / RPM_SENSE_PULSES_PER_TURN * TIMER1_TICKS_PER_US) / value)

// convert pulse length in µs to kHz
#define RPM_SENSE_US_TO_KHZ(value) (1e3 / static_cast<float>(value))

// if RPM falls below this value, RPM sensing will detect a halt
#define RPM_SENSE_RPM_MIN std::max(10, (RPM_MIN / 10))

#define RPM_SENSE_RPM_MIN_TICKS (RPM_SENSE_RPM_TO_US(RPM_SENSE_RPM_MIN) * TIMER1_TICKS_PER_US)

// used to calculate RPM_SENSE_IGNORE_SIGNAL_TIME, should be 3x the maximum RPM to detect
#define RPM_SENSE_RPM_MAX (RPM_MAX * 3)

class RpmSense {
public:
    typedef void (*capture_timer_callback_t)();

public:
    RpmSense();

    void begin();
    void reset();

    void setCallback(capture_timer_callback_t callback);

    void _overflowISR();
    void _captureISR();

    uint32_t getLastSignalMillis();
    uint24_t getTimerIntegralTicks();
    uint24_t getTimerIntegralMicros();
    float getTimerIntegralTicksFloat();

private:
    capture_timer_callback_t _callback;
    volatile uint16_t _timerOverflow;
    volatile uint32_t _lastTicks;
    volatile float _ticksIntegral;
public:
    volatile uint32_t _lastSignalMillis;
private:
    volatile bool _callbackLocked;
    volatile bool _isrLocked;
    volatile uint8_t _events;
};

extern RpmSense rpm_sense;

inline void UIData_t::updateDutyCyle()
{
    updateDutyCyle(rpm_sense.getTimerIntegralMicros());
}

inline void UIData_t::updateDutyCyle(uint32_t length)
{
    display_pulse_length_integral = (display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + length) / static_cast<uint8_t>(DISPLAY_RPM_MULTIPLIER + 1);
}

inline void UIData_t::updateRpmPulseWidth(uint32_t length)
{
    ui_data.display_pulse_length_integral = (ui_data.display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + length) / static_cast<uint8_t>(DISPLAY_RPM_MULTIPLIER + 1);
}

inline void RpmSense::_overflowISR()
{
    _timerOverflow++;
}

inline uint32_t RpmSense::getLastSignalMillis()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _lastSignalMillis;
    }
    __builtin_unreachable();
}

inline uint24_t RpmSense::getTimerIntegralTicks()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _ticksIntegral;
    }
    __builtin_unreachable();
}

inline float RpmSense::getTimerIntegralTicksFloat()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _ticksIntegral;
    }
    __builtin_unreachable();
}

inline uint24_t RpmSense::getTimerIntegralMicros()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _ticksIntegral / TIMER1_TICKS_PER_US;
    }
    __builtin_unreachable();
}
