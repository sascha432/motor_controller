/**
 * Author: sascha_lammers@gmx.de
 */

#include "rpm_sensing.h"
#include "helpers.h"
#include "main.h"
#include "timer.h"
#include "motor.h"
#include "pid_control.h"

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
    _isrLocked = false;
    _counter = 0;
    _lastTicks = TCNT1;
    _events.clear();
}

// initialize capture timer for RPM sensing
void RpmSense::begin()
{
    asm volatile ("cbi %0, %1" :: "I" (SFR::Pin<PIN_RPM_SIGNAL>::DDR_IO_ADDR()), "I" (SFR::Pin<PIN_RPM_SIGNAL>::PINbit()));

    TCCR1B |= _BV(ICES1);
    // clear flags
    TIFR1 |= _BV(ICF1) | _BV(TOV1);
    // enable input capture and timer1 overflow
    TIMSK1 |= _BV(ICIE1) | _BV(TOIE1);

    reset();
}

void RpmSense::_captureISR()
{
    // +++ time critical part starts here
    // in order to capture all events, there must be enough clock cycles between each interrupt
    // for low RPM enable RPM_SENSE_TOGGLE_EDGE, for high rpm or when clock cycles are running out,
    // disable RPM_SENSE_TOGGLE_EDGE
    auto timerCounter = ICR1;

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
    _counter++;
    // once the rpm signal becomes too fast for the PID controller, an average value is used
    //
    // see RpmSensing::kMinRpmSignalPeriod etc..
    //
    // 120 rpm = 33333 ticks
    // 1000 rpm = 4000 ticks
    // 3000 rpm = 1333 ticks
    // 5000 rpm = 800 ticks
    // 7500 rpm = 533 ticks

    if (_isrLocked) {
        return;
    }
    _isrLocked = true;
    auto ticks = _timerOverflow;
    auto counter = _counter;
    // --- time critical part ends here, ~115 ticks
    sei();

    ticks |= timerCounter;

    // get ticks passed since last interrupt
    uint32_t diff = getTicksDiff(ticks, _lastTicks);
    _lastTicks = ticks;

    diff /= counter;
    _counter = 0;

    _events._ticks += diff;
    _events._count++;

    if (motor.isVelocityMode()) {
        // takes at least ~2800 ticks
        pid.updateTicks(diff);
    }

    cli();
    _isrLocked = false;
}

