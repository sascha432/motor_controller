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
#include <Encoder.h>

// setup pin change interrupt for given pin
void pciSetup(byte pin)
{
    pinMode(pin, INPUT);
    *digitalPinToPCMSK(pin) |= _BV(digitalPinToPCMSKbit(pin));
    PCIFR |= _BV(digitalPinToPCICRbit(pin));
    PCICR |= _BV(digitalPinToPCICRbit(pin));
}

// we need the encoder for updating values
extern Encoder knob;
// last pin states are stored in this variable
PinChangedState lastState;
// flag for the main loop
PinChangedType pinChangedFlag;

void setup_interrupts()
{
    pinChangedFlag = PinChangedType::NONE;
    lastState.update();
    #if HAVE_CURRENT_LIMIT
        pciSetup(PIN_CURRENT_LIMIT_INDICATOR_PINNO);
    #endif
    pciSetup(PIN_BUTTON1);
    pciSetup(PIN_BUTTON2);
    pciSetup(PIN_ROTARY_ENC_CLK);
    pciSetup(PIN_ROTARY_ENC_DT);
}

// pin change interrupt handler
// changesets are collected through the ISRs

void check_interrupt_level_change(PinChangedState::ChangeSetIntType change_set)
{
    #if HAVE_CURRENT_LIMIT

        if (lastState.changed<PIN_CURRENT_LIMIT_INDICATOR_PCHS_PORT, PIN_CURRENT_LIMIT_INDICATOR_BIT>(change_set)) {
            current_limit.checkCurrentLimit(isCurrentLimitTripped());
        }

    #endif

    auto tmpFlag = static_cast<uint8_t>(pinChangedFlag);

    // buttons should have a decent amount of capacitance added to avoid bouncing and creating unnecessary interrupts
    // to have a good noise immunity use small resistors and big capacitors
    // rise/fall time 5-10ms
    if (lastState.changed<PIN_BUTTON1_PORT, PIN_BUTTON1_BIT>(change_set)) {
        tmpFlag |= PinChangedType::BUTTON1;
    }
    if (lastState.changed<PIN_BUTTON2_PORT, PIN_BUTTON2_BIT>(change_set)) {
        tmpFlag |= PinChangedType::BUTTON2;
    }

    if (lastState.changed<PIN_ROTARY_ENC_CLK_PORT, PIN_ROTARY_ENC_CLK_BIT>(change_set) || lastState.changed<PIN_ROTARY_ENC_DT_PORT, PIN_ROTARY_ENC_DT_BIT>(change_set)) {
        // the encoder pins should have some hardware debouncing in order to reduce the number of unnecessary interrupts
        // this can be a lot depending on the quality of the encoder. the capacitance of the filter should be low enough to
        // catch fast turns but filter noise and bouncing
        // rise/fall time 1ms or less (just a guess, not tested)
        //
        // update encoder inside interrupt since its time critical to avoid skipping states
        knob.update(static_cast<Encoder::PinStatesType>(readRotaryEncoderPinStates()));
        tmpFlag |= PinChangedType::KNOB;
    }
    pinChangedFlag = static_cast<PinChangedType>(tmpFlag);
}

// pin change ISRs

#if PIN_CHANGED_STATE_HAVE_PORTB

    ISR(PCINT0_vect) {
        PinChangedState::ChangeSetIntType changeSet = (PINB ^ lastState.get<PIN_CHANGED_STATE_PORTB>()) << PinChangedState::PortIntToPin<PIN_CHANGED_STATE_PORTB>::shl();
        lastState.update<PIN_CHANGED_STATE_PORTB>();
        check_interrupt_level_change(changeSet);
    }

#endif

#if PIN_CHANGED_STATE_HAVE_PORTC

    ISR(PCINT1_vect) {
        PinChangedState::ChangeSetIntType changeSet = (PINC ^ lastState.get<PIN_CHANGED_STATE_PORTC>()) << PinChangedState::PortIntToPin<PIN_CHANGED_STATE_PORTC>::shl();
        lastState.update<PIN_CHANGED_STATE_PORTC>();
        check_interrupt_level_change(changeSet);
    }

#endif

#if PIN_CHANGED_STATE_HAVE_PORTD

    ISR(PCINT2_vect) {
        PinChangedState::ChangeSetIntType changeSet = (PIND ^ lastState.get<PIN_CHANGED_STATE_PORTD>()) << PinChangedState::PortIntToPin<PIN_CHANGED_STATE_PORTD>::shl();
        lastState.update<PIN_CHANGED_STATE_PORTD>();
        check_interrupt_level_change(changeSet);
    }

#endif
