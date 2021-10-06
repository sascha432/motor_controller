/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include <Arduino.h>

class PidController;

class RpmSense {
public:
    struct Events {
        uint32_t _ticks;
        uint16_t _count;

        Events() {}
        Events(uint32_t ticks, uint16_t count) : _ticks(ticks), _count(count) {}

        void clear() {
            _ticks = 0;
            _count = 0;
        }
    };

public:
    RpmSense();

    void begin();
    void reset();

    void _overflowISR();
    void _captureISR();

    uint32_t getLastSignalTicks() const;

    Events getEvents();

private:
    uint32_t getTicksDiff(uint32_t current, uint32_t last) const;

private:
    friend PidController;

    volatile uint32_t _timerOverflow;
    volatile uint32_t _lastTicks;
    volatile bool _isrLocked;
    volatile uint8_t _counter;
    Events _events;
};

extern RpmSense rpm_sense;

inline uint16_t UIConfigData::getRpm() const
{
    return RPM_SENSE_TICKS_TO_RPM(_displayRpm);
}

inline __attribute__((always_inline)) void RpmSense::_overflowISR()
{
    _timerOverflow += 1UL << 16;
}

inline uint32_t RpmSense::getLastSignalTicks() const
{
    uint32_t last, current;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        last = _lastTicks;
        current = _timerOverflow;
    }
    return getTicksDiff(current | TCNT1, last);
}

inline __attribute__((always_inline)) uint32_t RpmSense::getTicksDiff(uint32_t current, uint32_t last) const
{
    uint32_t diff = current - last;
    if (diff > 0x7fffffffUL) {
        // handle _timerOverflow during locked interrupts
        diff += 1UL << 16;
    }
    return diff;
}

inline RpmSense::Events RpmSense::getEvents()
{
    if (!_events._count) {
        return Events(0, 0);
    }
    Events events;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        events = Events(_events._ticks, _events._count);
        _events.clear();
    }
    return events;
}
