/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "rpm_sensing.h"
#include "DebugBuffer.h"
#include "timer.h"
#include "helpers.h"

RpmSense rpm_sense;

// overflow interrupt
ISR(TIMER1_OVF_vect)
{
    rpm_sense._overflowISR();

}

// capture interrupt
ISR(TIMER1_CAPT_vect)
{
    rpm_sense._captureISR();
}

RpmSense::RpmSense() : capture_timer_callback_locked(false)
{
    reset();
}

// reset RPM sensing
void RpmSense::reset()
{
    capture_last_signal = millis();
    capture_timer_overflow = 0;
    capture_timer_integral = 0;
    capture_timer_signal_counter = 0;
    capture_last_signal_ticks = 0;
    capture_timer_block_ticks = 0;
    //TCCR1B &= ~_BV(ICES1);
}

// initialize capture timer for RPM sensing
void RpmSense::begin()
{
    pinMode(PIN_RPM_SIGNAL, INPUT);

    TCCR1B |= _BV(ICES1);
    // clear flags
	TIFR1 |= _BV(ICF1) | _BV(TOV1);
    // enable input capture and timer1 overflow
	TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);

    reset();
}

void RpmSense::setCallback(capture_timer_callback_t callback)
{
    capture_timer_callback = callback;
}

void RpmSense::_captureISR()
{
    auto counter = ICR1;

    // if (!capture_trigger) {
    //     capture_trigger++;
    //     timer1_trigger_on_rising();
    // }
    // else {
    //     capture_trigger--;
    //     timer1_trigger_on_falling();
    // }

    uint32_t ticks = capture_timer_overflow;
    ticks = (ticks << 16) | counter;
    int32_t diff = get_time_diff(capture_last_signal_ticks, ticks);
    if (diff < 0) { // capture_timer_overflow has not been incremeneted yet
        diff += 0x10000;
        ticks += 0x10000;
    }
    capture_last_signal_ticks = ticks;

    if (data.rpm_sense_average == 0) {
        capture_timer_signal_counter++;
        capture_timer_integral = diff;
    }
    else {
        if (capture_timer_signal_counter++ == 0) {
            capture_timer_integral = diff;
        }
        else {
            capture_timer_integral = ((capture_timer_integral * data.rpm_sense_average) + diff) / (data.rpm_sense_average + 1);
        }

    }
    capture_last_signal = millis();

    if (!capture_timer_callback_locked) {           // allow interrupts and make sure the callback is not called again until done
        capture_timer_callback_locked = true;
        sei();

        capture_timer_callback();
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            capture_timer_callback_locked = false;
        }
    }
}

uint32_t RpmSense::getLastSignalMillis()
{
    return capture_last_signal;
}

uint32_t RpmSense::getTimerIntegral()
{
    return capture_timer_integral;
}

uint32_t RpmSense::getTimerIntegralMicros()
{
    return capture_timer_integral / TIMER1_TICKS_PER_US;
}
