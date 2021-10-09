/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "main.h"
#include "motor.h"

#define DEBUG_ADC 0

#if HAVE_VOLTAGE_DETECTION && HAVE_CURRENT_DETECTION
#    define ADC_ANALOG_SOURCES 2
#elif HAVE_VOLTAGE_DETECTION
#    define ADC_ANALOG_SOURCES 1
#elif HAVE_CURRENT_DETECTION
#    define ADC_ANALOG_SOURCES 1
#else
#    define ADC_ANALOG_SOURCES 0
#endif

// trigger modes @ 16MHz
#define ADC_TRIGGER_MODE_FREE_RUNNING           0           // 9615Hz
#define ADC_TRIGGER_MODE_TIMER1_COMPARE_MATCH_B 1           // can be configured from ~244-9000Hz
#define ADC_TRIGGER_MODE_TIMER1_OVERFLOW        2           // 244Hz

// set trigger mode
#ifndef ADC_TRIGGER_MODE
#    define ADC_TRIGGER_MODE ADC_TRIGGER_MODE_TIMER1_COMPARE_MATCH_B
#endif

class ADCInterrupt {
public:
    // internal 1.1V reference
    static constexpr uint8_t kAnalogSource = INTERNAL;
    // values of the ADC 0-1023
    static constexpr uint16_t kAdcValues = 1024;
    // amount of readings before switching to the next pin
    // the first result after switching is discarded
    static constexpr uint8_t kAverageSampleCount = 4;

    static_assert(kAverageSampleCount >= 1, "kAverageSampleCount must be 1 or greater");
    static_assert(kAverageSampleCount < ((1UL << (sizeof(uint16_t) << 3)) / kAdcValues), "reduce kAverageSampleCount to fit into uint16_t (kAdcValues * kAverageSampleCount <= 0xffff)");

    #if ADC_TRIGGER_MODE == ADC_TRIGGER_MODE_TIMER1_COMPARE_MATCH_B

        // timer 1 compare match b is the the trigger source for the ADC
        static constexpr float kTriggerFrequencyHz = 500.0;
        static constexpr uint32_t kReadIntervalInTicks = ((1000000 / kTriggerFrequencyHz) * Timer1::kTicksPerMicrosecond);
        static_assert(kReadIntervalInTicks > 0x077f, "use free running mode for max. read rate");
        static_assert(kReadIntervalInTicks < 0xffff, "overflow, the timer 1 compare match b is 16 bit and limited to ~244Hz");

    #endif

    // value of the counter to indicate that the analog pin has changed
    static constexpr int8_t kSkipNextReading = -1;

    static constexpr float kVoltageMultiplier = (1.0 * (VoltageDetection::kDivider * ADCRef::kReferenceVoltage / (kAdcValues * static_cast<float>(kAverageSampleCount))));
    static constexpr float kCurrentMultiplier = ADCRef::kShuntToA / static_cast<float>(kAverageSampleCount);
    static constexpr float kPowerMultiplier = kVoltageMultiplier * kCurrentMultiplier;

    enum class AnalogPinType : uint8_t {
        #if HAVE_VOLTAGE_DETECTION
            VOLTAGE,
        #endif
        #if HAVE_CURRENT_DETECTION
            CURRENT,
        #endif
        MAX
    };

    static constexpr auto kNumChannels = static_cast<uint8_t>(AnalogPinType::MAX);

public:
    ADCInterrupt();

    void begin();

    float getVoltage_V() const;
    float getVoltageAvg_V() const;
    float getCurrent_A() const;
    float getPower_W() const;

    uint16_t getADCSum(uint8_t channel) const;
    uint16_t getADCSum(AnalogPinType channel) const;
    uint16_t getADCAvg(uint8_t channel) const;
    uint16_t getADCAvg(AnalogPinType channel) const;

public:
    // public for the ISR
    void _selectNextSourcePin();
    void _addValue(uint16_t value);
    void _resetTimer();

    volatile int8_t _counter;
    volatile uint16_t _sum;
    volatile uint8_t _analogSource;

private:
    #if HAVE_VOLTAGE_DETECTION
        uint32_t _voltageAverage;
    #endif
    uint16_t _results[kNumChannels];

#if DEBUG_ADC
public:
    uint32_t _readCounter{0};
    void printReadingsPerSecond() {
        Serial.print(_readCounter);
        Serial.print(' ');
        Serial.println(_readCounter / (millis() / 1000.0), 3);
    }
#endif
};

extern ADCInterrupt adc;

inline ADCInterrupt::ADCInterrupt() :
    _analogSource(0),
    #if HAVE_VOLTAGE_DETECTION
        _voltageAverage(0),
    #endif
    _results{}
{
}

inline void ADCInterrupt::_selectNextSourcePin()
{
    // store collected values
    auto tmp = _sum;
    _results[_analogSource] = tmp;
    switch(_analogSource) {
        #if HAVE_VOLTAGE_DETECTION
            case static_cast<uint8_t>(AnalogPinType::VOLTAGE):
                _voltageAverage = ((_voltageAverage * 64) + (static_cast<uint32_t>(tmp) << 8)) / 65;
                break;
        #endif
        // #if HAVE_CURRENT_DETECTION
        //     case static_cast<uint8_t>(AnalogPinType::CURRENT):
        //         break;
        // #endif
    }
    _sum = 0;

    // reset counter
    _counter = kSkipNextReading;
    #if ADC_ANALOG_SOURCES > 0
        _analogSource = ((_analogSource + 1) % kNumChannels);
        switch(_analogSource) {
            #if HAVE_VOLTAGE_DETECTION
                case static_cast<uint8_t>(AnalogPinType::VOLTAGE):
                    ADMUX = (kAnalogSource << 6) | (PIN_VOLTAGE - 14);
                    break;
            #endif
            #if HAVE_CURRENT_DETECTION
                case static_cast<uint8_t>(AnalogPinType::CURRENT):
                    ADMUX = (kAnalogSource << 6) | (PIN_CURRENT - 14);
                    break;
            #endif
        }
    #endif
}

inline void ADCInterrupt::_addValue(uint16_t value)
{
    _sum += value;
}

inline void ADCInterrupt::_resetTimer()
{
    #if ADC_TRIGGER_MODE == ADC_TRIGGER_MODE_TIMER1_COMPARE_MATCH_B
        OCR1B = TCNT1;
        OCR1B += kReadIntervalInTicks;
    #endif
}

inline void ADCInterrupt::begin()
{
    ATOMIC_BLOCK(ATOMIC_FORCEON) {

        _sum = 0;
        _analogSource = kNumChannels - 1;
        _selectNextSourcePin();

        // Set ADEN in ADCSRA to enable the ADC.
        // Set ADATE in ADCSRA to enable auto-triggering.
        // Set the Prescaler to 128 (16MHz/128 = 125KHz) (ADPSn)
        // Set ADIE in ADCSRA to enable the ADC interrupt.
        // Set ADSC in ADCSRA to start the ADC conversion
        ADCSRA |= _BV(ADEN) | _BV(ADATE) | (_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)) | _BV(ADIE) | _BV(ADSC);

        #if ADC_TRIGGER_MODE == ADC_TRIGGER_MODE_FREE_RUNNING
            // Clear ADTS2..0 in ADCSRB to set trigger mode to free running
            ADCSRB &= ~(_BV(ADTS0) | _BV(ADTS1) | _BV(ADTS2));
        #elif ADC_TRIGGER_MODE == ADC_TRIGGER_MODE_TIMER1_COMPARE_MATCH_B
            // set ADC trigger source - Timer/Counter1 Compare Match B
            ADCSRB |= _BV(ADTS2) | _BV(ADTS0);
            TIMSK1 |= _BV(OCIE1B);
        #elif ADC_TRIGGER_MODE == ADC_TRIGGER_MODE_TIMER1_OVERFLOW
            // set ADC trigger source - Timer/Counter1 Overflow
            ADCSRB |= _BV(ADTS2) | _BV(ADTS1);
        #endif

        _resetTimer();
    }
}

#if HAVE_VOLTAGE_DETECTION

    inline float ADCInterrupt::getVoltage_V() const
    {
        uint16_t voltage;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            voltage = getADCSum(AnalogPinType::VOLTAGE);
        }
        return voltage * kVoltageMultiplier;
    }

    inline float ADCInterrupt::getVoltageAvg_V() const
    {
        uint32_t voltage;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            voltage = _voltageAverage;
        }
        return voltage * (kVoltageMultiplier / 256.0);
    }

#else

    float ADCInterrupt::getVoltage_V() const {
        return NAN;
    }

    float ADCInterrupt::getVoltageAvg_V() const {
        return NAN;
    }

#endif

#if HAVE_CURRENT_DETECTION

    inline float ADCInterrupt::getCurrent_A() const
    {
        if (!motor.isOn()) {
            return 0;
        }
        uint16_t current;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            current = getADCSum(AnalogPinType::CURRENT);
        }
        return (current * kCurrentMultiplier) + ADCRef::kShuntOffsetA;
    }

#else

    float ADCInterrupt::getCurrent_A() const
    {
        return NAN;
    }

#endif

#if HAVE_VOLTAGE_DETECTION && HAVE_CURRENT_DETECTION

inline float ADCInterrupt::getPower_W() const
{
    if (!motor.isOn()) {
        return 0;
    }
    uint32_t voltage;
    uint16_t current;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        voltage = getADCSum(AnalogPinType::VOLTAGE);
        current = getADCSum(AnalogPinType::CURRENT);
    }
    return (voltage * current) * kPowerMultiplier;
}

#else

inline float ADCInterrupt::getPower_W() const
{
    return NAN;
}

#endif

inline uint16_t ADCInterrupt::getADCSum(uint8_t channel) const
{
    return _results[channel];
}

inline uint16_t ADCInterrupt::getADCSum(AnalogPinType channel) const
{
    return getADCSum(static_cast<uint8_t>(channel));
}

inline uint16_t ADCInterrupt::getADCAvg(uint8_t channel) const
{
    return getADCSum(channel) / kAverageSampleCount;
}

inline uint16_t ADCInterrupt::getADCAvg(AnalogPinType channel) const
{
    return getADCAvg(static_cast<uint8_t>(channel));
}
