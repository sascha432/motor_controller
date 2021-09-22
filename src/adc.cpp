/**
 * Author: sascha_lammers@gmx.de
 */

#include "adc.h"

ADCInterrupt adc;

// Interrupt service routine for the ADC completion
ISR(ADC_vect) {
    #if ADC_ANALOG_SOURCES > 1
        if (adc._counter >= 0) {
    #endif
        // read value
            adc._addValue(ADC);
    #if ADC_ANALOG_SOURCES > 1
        }
    #endif
    // rotate pins
    if (++adc._counter > adc.kReadCounter) {
        adc._selectNextSourcePin();
    }
}
