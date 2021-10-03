/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include <Arduino.h>

// // convert rpm to pulse length (µs)
// #define RPM_SENSE_RPM_TO_US(rpm) (rpm ? ((1000000UL * 60UL / RPM_SENSE_PULSES_PER_TURN) / (rpm)) : 0)

// // convert RPM pulse length (µs) to RPM
// #define RPM_SENSE_US_TO_RPM(pulse) ((1000000UL * 60UL / RPM_SENSE_PULSES_PER_TURN) / (pulse))

// // convert RPM pulse length (µs) to Hz
// #define RPM_SENSE_US_TO_HZ(pulse) ((1000000UL / RPM_SENSE_PULSES_PER_TURN) / (pulse))

// // convert RPM pulse length (ticks) to Hz
// #define RPM_SENSE_TICKS_TO_HZ(pulse) ((1000000UL / RPM_SENSE_PULSES_PER_TURN * Timer1::kTicksPerMicrosecond) / (pulse))

// // convert pulse length in µs to kHz
// #define RPM_SENSE_US_TO_KHZ(pulse) (1000 / static_cast<float>(pulse))

// if RPM falls below this value, RPM sensing will detect a halt
// #define RPM_SENSE_RPM_MIN std::max(10, (RPM_MIN / 10))

// #define RPM_SENSE_RPM_MIN_TICKS (RPM_SENSE_RPM_TO_US(RPM_SENSE_RPM_MIN) * Timer1::kTicksPerMicrosecond)

// used to calculate RPM_SENSE_IGNORE_SIGNAL_TIME, should be 3x the maximum RPM to detect
// #define RPM_SENSE_RPM_MAX (RPM_MAX * 3)

class PidController;

class RpmSense {
public:
    struct Events {
        uint32_t ticks;
        uint16_t count;
    };

public:
    RpmSense();

    void begin();
    void reset();

    void _overflowISR();
    void _captureISR();

    uint32_t getLastSignalMillis() const;
    volatile uint32_t _getLastSignalMillis() const;
    void setLastSignalMillis(uint32_t millis);
    int32_t getTimerIntegralTicks() const;
    volatile int32_t _getTimerIntegralTicks() const;
    uint16_t getRpm() const;

    Events getEvents();

private:
    friend PidController;

    volatile uint16_t _timerOverflow;
    volatile uint32_t _lastTicks;
    volatile uint32_t _ticksIntegral;
    volatile uint32_t _lastSignalMillis;
    volatile bool _isrLocked;
    volatile uint8_t _events;
    volatile uint32_t _ticksSum;
    volatile uint16_t _eventsSum;
};

extern RpmSense rpm_sense;

inline uint16_t UIConfigData::getRpm() const
{
    return RPM_SENSE_TICKS_TO_RPM(display_pulse_length_integral);
}

inline __attribute__((always_inline)) void RpmSense::_overflowISR()
{
    _timerOverflow++;
}

inline uint32_t RpmSense::getLastSignalMillis() const
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _lastSignalMillis;
    }
    __builtin_unreachable();
}

volatile inline uint32_t RpmSense::_getLastSignalMillis() const
{
    return _lastSignalMillis;
}

inline void RpmSense::setLastSignalMillis(uint32_t millis)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _lastSignalMillis = millis;
    }
}

inline int32_t RpmSense::getTimerIntegralTicks() const
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        return _ticksIntegral;
    }
    __builtin_unreachable();
}

volatile inline int32_t RpmSense::_getTimerIntegralTicks() const
{
    return _ticksIntegral;
}

inline uint16_t RpmSense::getRpm() const
{
    return RPM_SENSE_TICKS_TO_RPM(getTimerIntegralTicks());
}

inline RpmSense::Events RpmSense::getEvents()
{
    if (!_eventsSum) {
        return Events({0, 0});
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        Events tmp = { _ticksSum, _eventsSum };
        _ticksSum = 0;
        _eventsSum = 0;
        return tmp;
    }
    __builtin_unreachable();
}
