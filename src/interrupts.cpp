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

    #if 0
    // pin 12 PCMSK0 PCMSKbit 4 PCICRbit 0
    // pin 9 PCMSK0 PCMSKbit 1 PCICRbit 0
    // pin 4 PCMSK2 PCMSKbit 4 PCICRbit 2
    // pin 2 PCMSK2 PCMSKbit 2 PCICRbit 2
    // pin 3 PCMSK2 PCMSKbit 3 PCICRbit 2

    Serial.print(F("pin "));
    Serial.print(pin);
    Serial.print(F(" PCMSK"));
    Serial.print((_SFR_MEM_ADDR(*digitalPinToPCMSK(pin)) - _SFR_MEM_ADDR((PCMSK0))));
    Serial.print(F(" PCMSKbit "));
    Serial.print(digitalPinToPCMSKbit(pin));
    Serial.print(F(" PCICRbit "));
    Serial.println(digitalPinToPCICRbit(pin));
    #endif
}

// PIN_CURRENT_LIMIT_INDICATOR_PINNO

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
    #if 0

        #if HAVE_CURRENT_LIMIT
            pciSetup(PIN_CURRENT_LIMIT_INDICATOR_PINNO);
        #endif
        pciSetup(PIN_BUTTON1);
        pciSetup(PIN_BUTTON2);
        pciSetup(PIN_ROTARY_ENC_CLK);
        pciSetup(PIN_ROTARY_ENC_DT);

    #else

        // optimized code -80byte
        SFR::Pin<PIN_BUTTON1>::DDR() &= ~SFR::Pin<PIN_BUTTON1>::PINmask();
        SFR::Pin<PIN_BUTTON2>::DDR() &= ~SFR::Pin<PIN_BUTTON2>::PINmask();
        SFR::Pin<PIN_ROTARY_ENC_CLK>::DDR() &= ~SFR::Pin<PIN_ROTARY_ENC_CLK>::PINmask();
        SFR::Pin<PIN_ROTARY_ENC_DT>::DDR() &= ~SFR::Pin<PIN_ROTARY_ENC_DT>::PINmask();

        #if HAVE_CURRENT_LIMIT
            SFR::Pin<PIN_CURRENT_LIMIT_INDICATOR>::DDR() &= ~SFR::Pin<PIN_CURRENT_LIMIT_INDICATOR>::PINmask();
        #endif

        // PCMSKx needs manual adjustments to match the port if pins change
        PCMSK0 |= SFR::Pin<PIN_BUTTON1>::PINmask();
        PCMSK2 |= SFR::Pin<PIN_BUTTON2>::PINmask() | SFR::Pin<PIN_ROTARY_ENC_CLK>::PINmask() | SFR::Pin<PIN_ROTARY_ENC_DT>::PINmask();

        PCIFR |= PIN_CURRENT_LIMIT_INDICATOR_PCICR_BV | PIN_BUTTON1_PCICR_BV | PIN_BUTTON2_PCICR_BV | PIN_ROTARY_ENC_CLK_PCICR_BV | PIN_ROTARY_ENC_CLK_PCICR_BV;
        PCICR |= PIN_CURRENT_LIMIT_INDICATOR_PCICR_BV | PIN_BUTTON1_PCICR_BV | PIN_BUTTON2_PCICR_BV | PIN_ROTARY_ENC_CLK_PCICR_BV | PIN_ROTARY_ENC_CLK_PCICR_BV;

    #endif

}

// pin change interrupt handler
// changesets are collected through the ISRs

void check_interrupt_level_change(PinChangedState::ChangeSetIntType change_set)
{
    #if HAVE_CURRENT_LIMIT

        if (isCurrentLimitTripped()) {
            current_limit.currentLimitTripped();
        }

    #endif

    auto tmpFlag = static_cast<uint8_t>(pinChangedFlag);

    // buttons should have a decent amount of capacitance added to avoid bouncing and creating unnecessary interrupts
    // to have a good noise immunity use small resistors and big capacitors
    // rise/fall time 5-10ms
    if (lastState.changed<PIN_BUTTON1_PORT, SFR::Pin<PIN_BUTTON1>::PINbit()>(change_set)) {
        tmpFlag |= PinChangedType::BUTTON1;
    }
    if (lastState.changed<PIN_BUTTON2_PORT, SFR::Pin<PIN_BUTTON2>::PINbit()>(change_set)) {
        tmpFlag |= PinChangedType::BUTTON2;
    }

    if (lastState.changed<PIN_ROTARY_ENC_CLK_PORT, SFR::Pin<PIN_ROTARY_ENC_CLK>::PINbit()>(change_set) || lastState.changed<PIN_ROTARY_ENC_DT_PORT, SFR::Pin<PIN_ROTARY_ENC_DT>::PINbit()>(change_set)) {
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
