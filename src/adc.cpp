/**
 * Author: sascha_lammers@gmx.de
 */

#include "adc.h"

ADCInterrupt adc;

// Interrupt service routine for the ADC completion
ISR(ADC_vect) {
    // read value
    adc._addValue(ADC);
    // rotate pins
    if (++adc._counter > adc.kReadCounter) {
        adc._selectNextSourcePin();
    }
}
