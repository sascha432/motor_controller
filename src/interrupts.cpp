/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "interrupts.h"
#include "helpers.h"
#include "timer.h"
#include "pid_control.h"
#include "motor.h"
#include "current_limit.h"

#if DEBUG_TRIGGERED_INTERRUPTS
volatile InterruptTriggeredFlags_t interrupt_trigger_flags = { false, false };
#endif

volatile uint8_t pinb_state_last;

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
    PCIFR |= bit(digitalPinToPCICRbit(pin));
    PCICR |= bit(digitalPinToPCICRbit(pin));
}

#if HAVE_INTERRUPTS

void setup_interrupts()
{
    pinb_state_last = PINB;
#if HAVE_CURRENT_LIMIT
    pciSetup(PIN_CURRENT_LIMIT_INDICATOR);
#elif HAVE_DEBUG_RPM_SIGNAL_OUT
    pciSetup(PIN_RPM_SIGNAL);
#endif
}


ISR(PCINT0_vect) {

#define PINB_STATE_CHANGED(mask) (pinb_changes & mask)

    // track changes for PINB
    uint8_t pinb_changes = PINB ^ pinb_state_last;
    pinb_state_last = PINB;

#if HAVE_CURRENT_LIMIT

    if (PINB_STATE_CHANGED(PIN_CURRENT_LIMIT_INDICATOR_MASK)) {
        bool state = (PINB && PIN_CURRENT_LIMIT_INDICATOR_MASK);
#if DEBUG_TRIGGERED_INTERRUPTS
        if (state) {
            SET_INTERRUPT_TRIGGER(current_limit_flag, state);
        }
#endif
        current_limit.pinISR(state);
    }

#endif

#if HAVE_DEBUG_RPM_SIGNAL_OUT

    if (PINB_STATE_CHANGED(PIN_RPM_SIGNAL_MASK)) { // state changed
        SET_INTERRUPT_TRIGGER(rpm_sense_flag, true);
        if (PINB & PIN_RPM_SIGNAL_DEBUG_OUT_MASK) { // toggle PIN_RPM_SIGNAL_DEBUG_OUT
            PINB &= PIN_RPM_SIGNAL_DEBUG_OUT_MASK;
        } else {
            PINB |= PIN_RPM_SIGNAL_DEBUG_OUT_MASK;
        }
    }

#endif

}

#endif
