/**
 * Author: sascha_lammers@gmx.de
 */

#include "rpm_sensing.h"
#include "DebugBuffer.h"
#include "timer.h"
#include "helpers.h"

#if DEBUG_RPM_SIGNAL
#define CAPTURE_TIMER_BUFFER        50
DebugBuffer<DebugBufferItem<uint32_t>> capture_timer_values(CAPTURE_TIMER_BUFFER);
void dump_capture_timer_values() {
    capture_timer_values.dump(Serial);
}
#endif

capture_timer_callback_t capture_timer_callback;
volatile bool capture_timer_callback_locked = false;

volatile int16_t capture_timer_overflow;
volatile uint32_t capture_last_signal;
volatile uint32_t capture_timer_integral;
volatile uint32_t capture_last_signal_ticks;
volatile uint32_t capture_timer_signal_counter;
volatile uint32_t capture_timer_block_ticks;

#if DEBUG
uint8_t rpm_sense_average_count = 4;
// volatile uint16_t capture_timer_misfire;
// volatile uint32_t capture_timer_min;
// volatile uint32_t capture_timer_max;
// volatile uint32_t capture_timer_mean;
// volatile uint32_t capture_timer_mean_counter;

// void rpm_sense_reset_measurement() {
//     capture_timer_min = ~0;
//     capture_timer_max = 0;
//     capture_timer_mean = 0;
//     capture_timer_mean_counter = 0;
// }

// void rpm_sense_dump_measurement() {
//     Serial.print(F("\ncount="));
//     Serial.print(capture_timer_mean_counter);
//     Serial.print(F(" min="));
//     Serial.print(capture_timer_min);
//     Serial.print(F(" max="));
//     Serial.print(capture_timer_max);
//     Serial.print(F(" mean="));
//     Serial.println(capture_timer_mean);
// }
#endif

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

#if DEBUG_RPM_SIGNAL
    capture_timer_values.clear();
#endif

    timer1_trigger_on_falling();
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

#if DEBUG && 0
long variance[11];
long variance_counter;
void clear_variance() {
    variance_counter = 0;
    memset(&variance, 0, sizeof(variance));
}
void display_variance() {
    Serial.println("\n\n---");
    for(uint8_t i = 0; i < 10; i++) {
        Serial_printf("%f - <%u%%\n", variance[i] * 100.0 / variance_counter, i * 2);
    }
    Serial_printf("%f - >20%%\n", variance[10] * 100.0 / variance_counter);
    Serial.println("---");
}
#endif

// capture interrupt
ISR(TIMER1_CAPT_vect) {

    // if (timer1_trigger_is_rising) {
    //     timer1_trigger_on_falling();
    //     digitalWrite(9, HIGH);
    // }
    // else {
    //     timer1_trigger_on_rising();
    //     sei();
    //     digitalWrite(9, HIGH);
    //     delayMicroseconds(100);
    //     digitalWrite(9, LOW);
    //     return;
    // }
    auto counter = ICR1;
    uint32_t ticks = capture_timer_overflow;
    ticks = (ticks << 16) | counter;
    int32_t diff = get_time_diff(capture_last_signal_ticks, ticks);
    if (diff < 0) { // capture_timer_overflow has not been incremeneted yet
        diff += 0x10000;
        ticks += 0x10000;
    }
    capture_last_signal_ticks = ticks;

#if DEBUG && 0
    auto _variance = abs((long)diff - (long)capture_timer_integral);
    unsigned _var20 = capture_timer_integral / 5;
    unsigned pos = _variance * 10 / _var20;
    if (pos >= 10) {
        pos = 10;
    }
    variance[pos]++;
    variance_counter++;
#endif


#if DEBUG_RPM_SIGNAL
    capture_timer_values.add(diff);
#endif

#if !DEBUG && RPM_SENSE_AVERAGE_COUNT == 0
    capture_timer_signal_counter++;
    capture_timer_integral = diff;
#else
    if (capture_timer_signal_counter++ == 0) {
        capture_timer_integral = diff;
    }
    else {
        capture_timer_integral = ((capture_timer_integral * RPM_SENSE_AVERAGE_COUNT) + diff) / (RPM_SENSE_AVERAGE_COUNT + 1);
        // if ((float)capture_timer_integral > tmp * 10.0) {
        //     sprintf_P(error_msg + strlen(error_msg),  PSTR("integral %lu %lu\n"), capture_timer_integral, tmp);
        //     sprintf_P(error_msg + strlen(error_msg), PSTR("ovf %u cnt %u lst %lu tcks %lu diff %ld new %ld\n"), capture_timer_overflow, counter, capture_last_signal_ticks, ticks, diff, (diff + 0xffff + 1));
        //     capture_timer_integral = tmp;
        // }
    }
#endif
#if DEBUG && 0
    capture_timer_min = min(capture_timer_mean, capture_timer_min);
    capture_timer_max = max(capture_timer_mean, capture_timer_max);
    capture_timer_mean = (capture_timer_mean * capture_timer_mean_counter) + diff;
    capture_timer_mean_counter++;
    capture_timer_mean /= capture_timer_mean_counter;
#endif
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
