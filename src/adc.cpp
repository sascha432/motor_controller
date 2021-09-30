/**
 * Author: sascha_lammers@gmx.de
 */

#include "adc.h"

ADCInterrupt adc;

ISR(ADC_vect)
{
    // if the analog pin has been changed during last ISR, the next result could be from previous pin
    if (adc._counter != adc.kSkipNextReading) {
        adc._addValue(ADC);
    }
    #if DEBUG_ADC
        adc._readCounter++;
    #endif

    // rotate pins
    if (++adc._counter >= adc.kAverageSampleCount) {
        adc._selectNextSourcePin();
    }
    adc._resetTimer();
}


#if ADC_TRIGGER_MODE == ADC_TRIGGER_MODE_TIMER1_COMPARE_MATCH_B

ISR(TIMER1_COMPB_vect)
{
}

#endif
