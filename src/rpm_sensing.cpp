/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "rpm_sensing.h"
#include "DebugBuffer.h"
#include "timer.h"
#include "helpers.h"

capture_timer_callback_t capture_timer_callback;
volatile bool capture_timer_callback_locked = false;
// volatile uint8_t capture_trigger;

volatile int16_t capture_timer_overflow;
volatile uint32_t capture_last_signal;
volatile uint32_t capture_timer_integral;
volatile uint32_t capture_last_signal_ticks;
volatile uint32_t capture_timer_signal_counter;
volatile uint32_t capture_timer_block_ticks;

// reset RPM sensing
void reset_capture_timer() {
    cli();
    capture_last_signal = millis();
    capture_timer_overflow = 0;
    capture_timer_integral = 0;
    capture_timer_signal_counter = 0;
    capture_last_signal_ticks = 0;
    capture_timer_block_ticks = 0;
#if DEBUG
    // capture_timer_misfire = 0;
#endif

    timer1_trigger_on_falling();
    // capture_trigger = 0;

    sei();
}

// initialize capture timer for RPM sensing
void init_capture_timer() {

	TCCR1A = 0;
    TCCR1B = TIMER1_PRESCALER_BV | _BV(ICES1);
	TCCR1C = 0;

    reset_capture_timer();

    // enable input capture and timer1 overflow
	TIFR1 = _BV(ICF1) | _BV(TOV1);
	TIMSK1 = _BV(ICIE1) | _BV(TOIE1);
}

// overflow interrupt
ISR(TIMER1_OVF_vect) {
    capture_timer_overflow++;
}

void capture_timer_set_callback(capture_timer_callback_t callback) {
    capture_timer_callback = callback;
}

// capture interrupt
ISR(TIMER1_CAPT_vect) {

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
        cli();
        capture_timer_callback_locked = false;
        sei();
    }
}

uint32_t capture_timer_last_signal_millis() {
    return capture_last_signal;
}

uint32_t capture_timer_get_integral() {
    return capture_timer_integral;
}

uint32_t capture_timer_get_micros() {
    return capture_timer_integral / TIMER1_TICKS_PER_US;
}
