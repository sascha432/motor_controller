/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <avr/io.h>
#include <util/atomic.h>
#include "main.h"
#include "motor.h"

#if HAVE_VOLTAGE_DETECTION && HAVE_CURRENT_DETECTION
#    define ADC_ANALOG_SOURCES 2
#elif HAVE_VOLTAGE_DETECTION
#    define ADC_ANALOG_SOURCES 1
#elif HAVE_CURRENT_DETECTION
#    define ADC_ANALOG_SOURCES 1
#else
#    define ADC_ANALOG_SOURCES 0
#endif

// analog pins to read
static constexpr uint8_t kADCInterruptAnalogPins[] = {
    #if HAVE_VOLTAGE_DETECTION
        (PIN_VOLTAGE - 14),
    #endif
    #if HAVE_CURRENT_DETECTION
        (PIN_CURRENT - 14)
    #endif
};

class ADCInterrupt {
public:
    // internal 1.1V reference
    static constexpr uint8_t kAnalogSource = INTERNAL;
    // values of the ADC 0-1023
    static constexpr uint16_t kAdcValues = 1024;
    // amount of reads before switching to the next pin
    // the average value is stored
    static constexpr uint8_t kReadCounter = 63;
    static_assert(kReadCounter < ((1UL << (sizeof(uint16_t) << 3)) / kAdcValues), "reduce kReadCounter to fit into uint16_t (1024 * kReadCounter <= 0xffffff)");
    static_assert(kReadCounter < 126, "kReadCounter must be smaller than 126");

    enum class AnalogPinType : uint8_t {
        #if HAVE_VOLTAGE_DETECTION
            VOLTAGE,
        #endif
        #if HAVE_CURRENT_DETECTION
            CURRENT,
        #endif
    };

public:
    ADCInterrupt();

    void setup();

    uint16_t getVoltage_mV() const;
    float getVoltage_V() const;

    uint16_t getCurrent_mA() const;
    float getCurrent_A() const;

public:
    // public for the ISR
    void _selectNextSourcePin();
    void _addValue(uint16_t value);

    volatile int8_t _counter;
    volatile uint16_t _sum;
    volatile uint8_t _analogSource;

private:
    union {
        uint16_t _results[sizeof(kADCInterruptAnalogPins)];
        struct {
        #if HAVE_VOLTAGE_DETECTION
            uint16_t _voltage;
        #endif
        #if HAVE_CURRENT_DETECTION
            uint16_t _current;
        #endif
        };
    };
};

extern ADCInterrupt adc;

inline ADCInterrupt::ADCInterrupt() :
    _analogSource(0),
    _results{}
{
}

inline void ADCInterrupt::_selectNextSourcePin()
{
    // store previous value
    _results[_analogSource] = _sum;
    _sum = 0;

    #if ADC_ANALOG_SOURCES == 1
        // reset counter
        _counter = 0;
    #else
        // select next source
        #if HAVE_CURRENT_DETECTION
            if (motor.isOn()) {
                // reset counter
                _counter = 0;
                // measure current only while motor is running
                _analogSource = static_cast<uint8_t>(AnalogPinType::CURRENT);
            }
            else
        #endif
        {
            // reset counter, skip first reading after switching since it is from the last pin
            _counter = -1;
            _analogSource = ((_analogSource + 1) % sizeof(kADCInterruptAnalogPins));
        }
    #endif
    // analogReference(INTERNAL) for the ADC pin
    ADMUX = (kAnalogSource << 6) | kADCInterruptAnalogPins[_analogSource];
}

inline void ADCInterrupt::_addValue(uint16_t value)
{
    _sum += value;
}

inline void ADCInterrupt::setup()
{
    ATOMIC_BLOCK(ATOMIC_FORCEON) {

        _sum = 0;
        _analogSource = sizeof(kADCInterruptAnalogPins) - 1;
        _selectNextSourcePin();

        // Set ADEN in ADCSRA to enable the ADC.
        // Set ADATE in ADCSRA to enable auto-triggering.
        // Set the Prescaler to 128 (16MHz/128 = 125KHz) (ADPSn)
        // Set ADIE in ADCSRA to enable the ADC interrupt.
        // Set ADSC in ADCSRA to start the ADC conversion
        ADCSRA |= _BV(ADEN) | _BV(ADATE) | (_BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2)) | _BV(ADIE) | _BV(ADSC);

        // Clear ADTS2..0 in ADCSRB to set trigger mode to free running.
        ADCSRB &= ~(_BV(ADTS0) | _BV(ADTS1) | _BV(ADTS2));

    }
}

#if HAVE_VOLTAGE_DETECTION

    // calculations are done during compile time and this is a simple multiplication (and shifting for mV)

    // inline uint16_t ADCInterrupt::getVoltage_mV() const
    // {
    //     constexpr uint8_t kPrecisionShift = 10;
    //     constexpr uint16_t kMultiplier = ((1000UL << kPrecisionShift) * (VoltageDetection::kDivider * ADCRef::kReferenceVoltage / (kAdcValues * static_cast<float>(kReadCounter))));
    //     uint16_t voltage;
    //     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //         voltage = _voltage;
    //     }
    //     return (static_cast<uint32_t>(voltage) * kMultiplier) >> kPrecisionShift;
    // }

    inline float ADCInterrupt::getVoltage_V() const
    {
        constexpr float kMultiplier = (1.0 * (VoltageDetection::kDivider * ADCRef::kReferenceVoltage / (kAdcValues * static_cast<float>(kReadCounter))));
        uint16_t voltage;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            voltage = _voltage;
        }
        return voltage * kMultiplier;
    }

#else

    // uint16_t ADCInterrupt::getVoltage_mV() const {
    //     return 0;
    // }

    float ADCInterrupt::getVoltage_V() const {
        return NAN;
    }

#endif

#if HAVE_CURRENT_DETECTION

    // calculations are done during compile time and this is a simple multiplication

    // inline uint16_t ADCInterrupt::getCurrent_mA() const
    // {
    //     uint16_t current;
    //     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    //         current = _current;
    //     }

    //     return CURRENT_SHUNT_TO_mA(current, kReadCounter);
    //     // return CURRENT_SHUNT_TO_mA(current, kReadCounter);
    // }

    inline float ADCInterrupt::getCurrent_A() const
    {
        if (!motor.isOn()) {
            return 0;
        }
        uint16_t current;
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            current = _current;
        }
        return CURRENT_SHUNT_TO_A(current, kReadCounter);
        // return CURRENT_SHUNT_TO_mA(current, (kReadCounter / 1000.0));
    }

#else

    // uint16_t ADCInterrupt::getCurrent_mA() const
    // {
    //     return 0;
    // }

    float ADCInterrupt::getCurrent_A() const
    {
        return NAN;
    }

#endif
