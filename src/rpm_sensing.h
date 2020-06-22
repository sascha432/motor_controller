/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include "main.h"

#ifndef RPM_SENSE_PULSES_PER_TURN
// wheel with 120 slots
#define RPM_SENSE_PULSES_PER_TURN               120
#endif

// convert rpm to pulse length (µs)
#define RPM_SENSE_RPM_TO_US(rpm)                (1000000UL * 60UL / (rpm * (uint32_t)RPM_SENSE_PULSES_PER_TURN))

// convert RPM pulse length (µs) to RPM
#define RPM_SENSE_US_TO_RPM(value)              ((1000000UL * 60UL / RPM_SENSE_PULSES_PER_TURN) / value)

// convert pulse length in µs to kHz
#define RPM_SENSE_US_TO_KHZ(value)              (1e3 / (float)value)

// if RPM falls below this value, RPM sensing will detect a halt
#define RPM_SENSE_RPM_MIN                       max(1, (RPM_MIN / 100))

#define RPM_SENSE_RPM_MIN_TICKS                 (RPM_SENSE_RPM_TO_US(RPM_SENSE_RPM_MIN) * TIMER1_TICKS_PER_US)

// used to calculate RPM_SENSE_IGNORE_SIGNAL_TIME, should be 3x the maximum RPM to detect
#define RPM_SENSE_RPM_MAX                       (RPM_MAX * 3)

class RpmSense {
public:
    typedef void (*capture_timer_callback_t)();

public:
    RpmSense();

    void begin();
    void reset();

    void setCallback(capture_timer_callback_t callback);

    inline void _overflowISR()
    {
        capture_timer_overflow++;
    }
    void _captureISR();

    uint32_t getLastSignalMillis();
    uint32_t getTimerIntegral();
    uint32_t getTimerIntegralMicros();

private:
    capture_timer_callback_t capture_timer_callback;
    volatile bool capture_timer_callback_locked;
    // volatile uint8_t capture_trigger;

    volatile int16_t capture_timer_overflow;
    volatile uint32_t capture_last_signal;
    volatile uint32_t capture_timer_integral;
    volatile uint32_t capture_last_signal_ticks;
    volatile uint32_t capture_timer_signal_counter;
    volatile uint32_t capture_timer_block_ticks;
};

extern RpmSense rpm_sense;
