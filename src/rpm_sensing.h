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

inline void timer1_trigger_on_falling() {
    TCCR1B &= ~_BV(ICES1);
}

inline void timer1_trigger_on_rising() {
    TCCR1B |= _BV(ICES1);
}

inline bool timer1_trigger_is_rising() {
    return (TCCR1B & _BV(ICES1));
}

#if DEBUG_RPM_SIGNAL
void dump_capture_timer_values();
#endif

#if DEBUG
// extern volatile uint16_t capture_timer_misfire;
extern uint8_t rpm_sense_average_count;
// void rpm_sense_reset_measurement();
// void rpm_sense_dump_measurement();
#endif

typedef void (*capture_timer_callback_t)();

void reset_capture_timer();
void init_capture_timer();
void capture_timer_set_callback(capture_timer_callback_t callback);
uint32_t capture_timer_last_signal_millis();
uint32_t capture_timer_get_integral();
uint32_t capture_timer_get_micros();
