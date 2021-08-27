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

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
    PCIFR |= bit(digitalPinToPCICRbit(pin));
    PCICR |= bit(digitalPinToPCICRbit(pin));
}

#if HAVE_INTERRUPTS

volatile uint8_t pinb_state_last;

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

#define PINB_STATE_CHANGED(mask) (pinb_change_set & mask)

    // track changes for PINB
    uint8_t pinb_change_set = PINB ^ pinb_state_last;
    pinb_state_last = PINB;

#if HAVE_CURRENT_LIMIT

    if (PINB_STATE_CHANGED(PIN_CURRENT_LIMIT_INDICATOR_MASK)) {
        current_limit.pinISR((PINB && PIN_CURRENT_LIMIT_INDICATOR_MASK));
    }

#endif

#if HAVE_DEBUG_RPM_SIGNAL_OUT

    if (PINB_STATE_CHANGED(PIN_RPM_SIGNAL_MASK)) { // state changed
        if (PINB & PIN_RPM_SIGNAL_DEBUG_OUT_MASK) { // toggle PIN_RPM_SIGNAL_DEBUG_OUT
            PORTB &= PIN_RPM_SIGNAL_DEBUG_OUT_MASK;
        } else {
            PORTB |= PIN_RPM_SIGNAL_DEBUG_OUT_MASK;
        }
    }

#endif

}

#endif
