/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "interrupts.h"
#include "helpers.h"

#if DEBUG_TRIGGERED_INTERRUPTS
volatile InterruptTriggeredFlags_t interrupt_trigger_flags = { false, false };
#endif

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
    PCIFR |= bit(digitalPinToPCICRbit(pin));
    PCICR |= bit(digitalPinToPCICRbit(pin));
}

#if HAVE_CURRENT_LIMIT
volatile uint32_t current_limit_tripped = 0;
#endif

#if HAVE_INTERRUPTS

static uint8_t pinb_state_last;

void setup_interrupts()
{
    cli();
    pinb_state_last = PINB;
#if HAVE_CURRENT_LIMIT
    pciSetup(PIN_CURRENT_LIMIT_INDICATOR);
#endif
#if HAVE_DEBUG_RPM_SIGNAL_OUT
    pciSetup(PIN_RPM_SIGNAL);
#endif
    sei();
}


ISR(PCINT0_vect) {

    // track changes for PINB
    uint8_t pin_state = PINB;
    uint8_t pinb_changes = pin_state ^ pinb_state_last;
    pinb_state_last = pin_state;

#if HAVE_CURRENT_LIMIT

    if (pinb_changes & PIN_CURRENT_LIMIT_INDICATOR_MASK) { // state changed
        SET_INTERRUPT_TRIGGER(current_limit_flag, true);

        // if (millis() > data.motor_start_time + 2000) {
        //     // TODO implement reducing/limiting duty cycle
        //     // turn motor off
        //     set_motor_speed(0);
        //     data.motor_state = MotorStateEnum::CURRENT_LIMIT;
        // }

        if ((pin_state && PIN_CURRENT_LIMIT_INDICATOR_MASK) && current_limit_tripped == 0) { // rising edge, store time if not set
            current_limit_tripped = millis();
        }
    }

#endif

#if HAVE_DEBUG_RPM_SIGNAL_OUT

    if (pinb_changes & PIN_RPM_SIGNAL_MASK) { // state changed
        SET_INTERRUPT_TRIGGER(rpm_sense_flag, true);
        if (pin_state & PIN_RPM_SIGNAL_DEBUG_OUT_MASK) { // toggle PIN_RPM_SIGNAL_DEBUG_OUT
            PINB &= PIN_RPM_SIGNAL_DEBUG_OUT_MASK;
        } else {
            PINB |= PIN_RPM_SIGNAL_DEBUG_OUT_MASK;
        }
    }

#endif

}

#endif