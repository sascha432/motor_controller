/**
 * Author: sascha_lammers@gmx.de
 */

#include "rpm_sensing.h"
#include "DebugBuffer.h"
#include "helpers.h"
#include "main.h"
#include "timer.h"

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif
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

#pragma GCC optimize("Os")

RpmSense::RpmSense()
{
    reset();
}

// reset RPM sensing
void RpmSense::reset()
{
    // trigger on rising edge
    TCCR1B |= _BV(ICES1);
    _timerOverflow = 0;
    _events = 0;
    _isrLocked = false;
    // none measured yet
    _ticksIntegral = NAN;
    _lastSignalMillis = millis();
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
    _callback = callback;
}

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif

void RpmSense::_captureISR()
{
    // +++ time critical part starts here
    // in order to capture all events, there must be enough clock cycles between each interrupt
    // for low RPM enable RPM_SENSE_TOGGLE_EDGE, for high rpm or when clock cycles are running out,
    // disable RPM_SENSE_TOGGLE_EDGE
    auto counter = ICR1;

    #if RPM_SENSE_TOGGLE_EDGE
        // toggle edge
        if (TCCR1B & _BV(ICES1)) {
            TCCR1B &= ~_BV(ICES1);
        }
        else {
            TCCR1B |= _BV(ICES1);
        }
        sbi(TIFR1, ICF1);

        // Measurement of an external signal’s duty cycle requires that the trigger edge is changed after each capture. Changing the
        // edge sensing must be done as early as possible after the ICR1 register has been read. After a change of the edge, the input
        // capture flag (ICF1) must be cleared by software (writing a logical one to the I/O bit location). For measuring frequency only,
        // the clearing of the ICF1 flag is not required (if an interrupt handler is used).
    #endif
    _events++;
    if (_isrLocked) {
        return;
    }
    _isrLocked = true;
    // capture values we need for the calculation
    uint32_t ticks = _timerOverflow;
    auto events = _events;
    // allow interrupts now
    sei();
    // --- time critical part ends here

    // add the overflow counter as upper word and the captured counter as lower word
    ticks = (ticks << 16) | counter;

    // Serial.print(ticks);
    // Serial.print(' ');
    // Serial.println(events);

    // get ticks passed since last interrupt
    uint32_t diff = ticks - _lastTicks;
    if (diff > 0x7fffffffUL) {
        // _timerOverflow not incremented
        diff += 1UL << 16;
    }
    if (isnan(_ticksIntegral)) {
        cli();
        // first interrupt after a reset
        _ticksIntegral = diff;
    }
    else if (data.rpm_sense_average == 0) {
        cli();
        // no averaging
        _ticksIntegral = diff;
    }
    else {
        // avg over data.rpm_sense_average values
        auto tmp = ((_ticksIntegral * data.rpm_sense_average) + diff) / static_cast<uint8_t>(data.rpm_sense_average + events);
        cli();
        _ticksIntegral = tmp;
    }
    _events -= events;
    _lastTicks = ticks;

    // store time to detect a stall
    _lastSignalMillis = millis();
    _isrLocked = false;

    if (!_callbackLocked) {
        // allow interrupts and make sure the callback is not called again until done
        _callbackLocked = true;
        sei();
        _callback();
        cli();
        _callbackLocked = false;
    }
}

