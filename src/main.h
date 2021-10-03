/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <avr/wdt.h>
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <Adafruit_SSD1306.h>
#include "helpers.h"
#include "int24_types.h"

#define VERSION_MAJOR                           1
#define VERSION_MINOR                           0
#define VERSION_PATCH                           4

// show at what time and date the firmware was compiled
#ifndef HAVE_COMPILED_ON_DATE
#define HAVE_COMPILED_ON_DATE                   1
#endif

#if DEBUG
#define DEBUG_INPUTS                            0
#define DEBUG_MOTOR_SPEED                       0
#define DEBUG_PID_CONTROLLER                    0
#else
#define DEBUG_INPUTS                            0
#define DEBUG_MOTOR_SPEED                       0
#define DEBUG_PID_CONTROLLER                    0
#endif

// enable serial commands
// >2700 byte (includes HAVE_SERIAL_HELP, HAVE_SERIAL_MOTOR_CONTROL, HAVE_VOLTAGE_DETECTION etc...)
#ifndef HAVE_SERIAL_COMMANDS
#define HAVE_SERIAL_COMMANDS                    1
#endif

// enable stats output over serial
// ~1-2kbyte
#ifndef HAVE_PID_CONTROLLER_STATS
#define HAVE_PID_CONTROLLER_STATS               0
#endif

// disable LED support
#ifndef HAVE_LED
#define HAVE_LED                                1
#endif

// dimming the LED driver is supported
// ~150 byte
#ifndef HAVE_LED_FADING
#define HAVE_LED_FADING                         1
#endif

#if HAVE_LED_FADING && !HAVE_LED
#error HAVE_LED_FADING=1 requires HAVE_LED=1
#endif

// display help for serial commands when pressing ? or h
// ~276 byte
#ifndef HAVE_SERIAL_HELP
#define HAVE_SERIAL_HELP                        0
#endif

// enable serial commands
// ~344 byte
#ifndef HAVE_SERIAL_MOTOR_CONTROL
#define HAVE_SERIAL_MOTOR_CONTROL               0
#endif

// read supply voltage from ADC
// ~572byte
#ifndef HAVE_VOLTAGE_DETECTION
#define HAVE_VOLTAGE_DETECTION                  1
#endif

// read motor current from ADC
// ~300 byte
#ifndef HAVE_CURRENT_DETECTION
#define HAVE_CURRENT_DETECTION                  1
#endif

// have pin that signals overcurrent
// ~816 byte
#ifndef HAVE_CURRENT_LIMIT
#define HAVE_CURRENT_LIMIT                      1
#endif

// display power consumption of the LEDs
// ~434 byte
#ifndef HAVE_LED_POWER
#define HAVE_LED_POWER                          1
#endif

// pins

// D11/PB3/15
// pin for the mosfet driver. analogWrite is pretty slow (and adds quite some extra code)
#define PIN_MOTOR_PWM_PIN                       PINB
#define PIN_MOTOR_PWM_DDR                       DDRB
#define PIN_MOTOR_PWM_PORT                      PORTB
#define PIN_MOTOR_PWM_BIT                       PINB3
#define PIN_MOTOR_PWM_OCR                       OCR2A
#define PIN_MOTOR_ENABLE_PWM()                  sbi(TCCR2A, COM2A1)
#define PIN_MOTOR_DISABLE_PWM()                 cbi(TCCR2A, COM2A1)

inline void stopMotor()
{
    PIN_MOTOR_DISABLE_PWM();
    PIN_MOTOR_PWM_OCR = 0;
    asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_PORT)), "I" (PIN_MOTOR_PWM_BIT));     // digital write low
}

inline void setupMotorPwm()
{
    // #if DEBUG_MOTOR_SPEED
    //     Serial.printf_P(PSTR("timer=%u\n"), digitalPinToTimer(11));
    // #endif
    asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_PORT)), "I" (PIN_MOTOR_PWM_BIT));     // digital write low
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_DDR)), "I" (PIN_MOTOR_PWM_BIT));      // pin mode
    PIN_MOTOR_PWM_OCR = 0;
}

inline void stopMotorAtomic()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        stopMotor();
    }
}

// do not use for 0 (stopMotor) or 255 (setMotorPWMAtomic)
// PWM must be enabled
inline void setMotorPWM_timer(uint8_t pwm)
{
    PIN_MOTOR_PWM_OCR = pwm; // analog write
}

// returns 0 if the motor is off and 255 if running at 100% duty cycle
inline uint8_t getMotorPWM_timer()
{
    return PIN_MOTOR_PWM_OCR;
}

// use stopMotor() for pwm = 0
inline void setMotorPWM(uint8_t pwm)
{
    if (pwm == 0) {
        stopMotor();
    }
    else if (pwm == 255) {
        PIN_MOTOR_DISABLE_PWM();
        PIN_MOTOR_PWM_OCR = 255;
        asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_PORT)), "I" (PIN_MOTOR_PWM_BIT)); // digital write high
    }
    else {
        setMotorPWM_timer(pwm);
        PIN_MOTOR_ENABLE_PWM();
    }
}

inline void setMotorPWMAtomic(uint8_t pwm)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        setMotorPWM(pwm);
    }
}


// D7/PD7/11
// pin to enable the brake. it basically shorts the motor wires for slow current decay
// it is important that the mosfets are turned off before the break is engaged + a short delay depending
// on the mosfet driver. having the mosfets on and the brake engaged will cause a dead short
#define PIN_BRAKE_PIN                           PIND
#define PIN_BRAKE_PORT                          PORTD
#define PIN_BRAKE_DDR                           DDRD
#define PIN_BRAKE_BIT                           PIND7

inline bool isBrakeOn()
{
    asm volatile goto (
        "sbic %0, %1\n\t"
        "rjmp %l[BRAKE_ENABLED]\n\t"
        :: "I" (_SFR_IO_ADDR(PIN_BRAKE_PIN)), "I" (PIN_BRAKE_BIT) :: BRAKE_ENABLED
    );
    return false;

BRAKE_ENABLED:
    return true;
}

inline void setBrakeOn()
{
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_BRAKE_PORT)), "I" (PIN_BRAKE_BIT));
}

inline void setBrakeOff()
{
    asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_BRAKE_PORT)), "I" (PIN_BRAKE_BIT));
}

inline void setupBrake()
{
    setBrakeOff();
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_BRAKE_DDR)), "I" (PIN_BRAKE_BIT));
}

// D5/PD5/9
// pin for the LED dimmer
// -1 to disable
#define PIN_LED_DIMMER                          5

#define PIN_LED_DIMMER_PWM_PIN                  PIND
#define PIN_LED_DIMMER_PWM_DDR                  DDRD
#define PIN_LED_DIMMER_PWM_PORT                 PORTD
#define PIN_LED_DIMMER_PWM_BIT                  PIND5
#define PIN_LED_DIMMER_PWM_OCR                  OCR0B
#define PIN_LED_DIMMER_ENABLE_PWM()             sbi(TCCR0A, COM0B1)
#define PIN_LED_DIMMER_DISABLE_PWM()            cbi(TCCR0A, COM0B1)

inline void setupLedPwm()
{
    // setup pin even without LED support since it has pullup
    #if PIN_LED_DIMMER != -1
        asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_LED_DIMMER_PWM_PORT)), "I" (PIN_LED_DIMMER_PWM_BIT));     // digital write low
        asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_LED_DIMMER_PWM_DDR)), "I" (PIN_LED_DIMMER_PWM_BIT));      // pin mode
    #endif
}

#if HAVE_LED

// LED PWM 980Hz for MT3608
// this is a voltage regulatur, not constant current. the circuit has been modified to provide constant current,
// but due to the output capacitor the dimming range should be limited to 90-93% PWM. above 90% the current increases
// pretty non linear. i had best resuts with 120, 180 and 240Hz. lower frequencies provider a better linear dimming curve
#define LED_MIN_PWM                             32
#define LED_MAX_PWM                             240     // more than 240 does not increase brightness much for my LEDs
#define LED_FADE_TIME                           10

#if LED_MIN_PWM < 5
#error increase min. pwm
#endif

inline void analogWriteLedPwm(uint8_t pwm)
{
    if (pwm == 255) {
        PIN_LED_DIMMER_DISABLE_PWM();
        asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_LED_DIMMER_PWM_PORT)), "I" (PIN_LED_DIMMER_PWM_BIT));     // digital write high
    }
    else if (pwm < LED_MIN_PWM) {
        PIN_LED_DIMMER_DISABLE_PWM();
        asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_LED_DIMMER_PWM_PORT)), "I" (PIN_LED_DIMMER_PWM_BIT));     // digital write low
    }
    else {
        PIN_LED_DIMMER_ENABLE_PWM();
        PIN_LED_DIMMER_PWM_OCR = pwm; // analog write
    }
}

#endif


// D6/PD6/10
// vref for the comparator and the current limit
#define PIN_CURRENT_LIMIT_PWM                   6

#define PIN_CURRENT_LIMIT_PWM_PIN               PIND
#define PIN_CURRENT_LIMIT_PWM_DDR               DDRD
#define PIN_CURRENT_LIMIT_PWM_PORT              PORTD
#define PIN_CURRENT_LIMIT_PWM_BIT               PIND6
#define PIN_CURRENT_LIMIT_PWM_OCR               OCR0A
#define PIN_CURRENT_LIMIT_ENABLE_PWM()          sbi(TCCR0A, COM0A1)
#define PIN_CURRENT_LIMIT_DISABLE_PWM()         cbi(TCCR0A, COM0A1)

inline void setupCurrentLimitPwm()
{
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_PWM_PORT)), "I" (PIN_CURRENT_LIMIT_PWM_BIT));     // digital write high
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_PWM_DDR)), "I" (PIN_CURRENT_LIMIT_PWM_BIT));      // pin mode
}

inline void analogWriteCurrentLimitPwm(uint8_t pwm)
{
    if (pwm == 255) {
        PIN_CURRENT_LIMIT_DISABLE_PWM();
        asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_PWM_PORT)), "I" (PIN_CURRENT_LIMIT_PWM_BIT));     // digital write high
    }
    else if (pwm == 0) {
        PIN_CURRENT_LIMIT_DISABLE_PWM();
        asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_PWM_PORT)), "I" (PIN_CURRENT_LIMIT_PWM_BIT));     // digital write low
    }
    else {
        PIN_CURRENT_LIMIT_ENABLE_PWM();
        PIN_CURRENT_LIMIT_PWM_OCR = pwm; // analog write
    }
}

// // D10/PB2/14
// // pin to disable the current limit
// #define PIN_CURRENT_LIMIT_OVERRIDE_PIN          PINB
// #define PIN_CURRENT_LIMIT_OVERRIDE_PORT         PORTB
// #define PIN_CURRENT_LIMIT_OVERRIDE_DDR          DDRB
// #define PIN_CURRENT_LIMIT_OVERRIDE_BIT          PINB2

// D12/PB4/MISO
// signal from comparator when the current limit has been triggered
#define PIN_CURRENT_LIMIT_INDICATOR_PINNO       12
#define PIN_CURRENT_LIMIT_INDICATOR_PIN         PINB
#define PIN_CURRENT_LIMIT_INDICATOR_PORT        PORTB
#define PIN_CURRENT_LIMIT_INDICATOR_PCHS_PORT   PIN_CHANGED_STATE_PORTB
#define PIN_CURRENT_LIMIT_INDICATOR_DDR         DDRB
#define PIN_CURRENT_LIMIT_INDICATOR_BIT         PINB4

inline bool isCurrentLimitTripped()
{
    asm volatile goto (
        "sbic %0, %1\n\t"
        "rjmp %l[LIMIT]\n\t"
        :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_INDICATOR_PIN)), "I" (PIN_CURRENT_LIMIT_INDICATOR_BIT) :: LIMIT
    );
    return false;

LIMIT:
    return true;
}

// pins that require interrupts
// its using the pin change ISRs

// D8/PB0/12
// pin for the LM393 schmitt trigger to read the ITR9608 opto interrupter
#define PIN_RPM_SIGNAL                          8

// UI interface

// D9/PB1/13
#define PIN_BUTTON1                             9
#define PIN_BUTTON1_PORT                        PIN_CHANGED_STATE_PORTB
#define PIN_BUTTON1_BIT                         PINB1

// D4/PD4/2
#define PIN_BUTTON2                             4
#define PIN_BUTTON2_PORT                        PIN_CHANGED_STATE_PORTD
#define PIN_BUTTON2_BIT                         PIND4

// D2/PD2/32
#define PIN_ROTARY_ENC_CLK                      2
#define PIN_ROTARY_ENC_CLK_PORT                 PIN_CHANGED_STATE_PORTD
#define PIN_ROTARY_ENC_CLK_BIT                  PIND2
#define PIN_ROTARY_ENC_CLK_PIN                  PIND

// D3/PD3/1
#define PIN_ROTARY_ENC_DT                       3
#define PIN_ROTARY_ENC_DT_PORT                  PIN_CHANGED_STATE_PORTD
#define PIN_ROTARY_ENC_DT_BIT                   PIND3
#define PIN_ROTARY_ENC_DT_PIN                   PIND

inline uint8_t readRotaryEncoderPinStates()
{
    register uint8_t ret = 0;
    asm volatile goto (
        "sbis %0, %1\n\t"
        "rjmp %l[PIN1_LOW]\n\t"
        :: "I" (_SFR_IO_ADDR(PIN_ROTARY_ENC_CLK_PIN)), "I" (PIN_ROTARY_ENC_CLK_BIT) :: PIN1_LOW
    );
    ret = static_cast<uint8_t>(Encoder::PinStatesType::P1_HIGH);
PIN1_LOW:
    asm volatile goto (
        "sbis %0, %1\n\t"
        "rjmp %l[PIN2_LOW]\n\t"
        :: "I" (_SFR_IO_ADDR(PIN_ROTARY_ENC_DT_PIN)), "I" (PIN_ROTARY_ENC_DT_BIT) :: PIN2_LOW
    );
    ret |= static_cast<uint8_t>(Encoder::PinStatesType::P2_HIGH);
PIN2_LOW:
    return ret;
}

// we do not have if constexpr () and macros cannot evaluate PORTX macros
#define PIN_CHANGED_STATE_PORTB                 0
#define PIN_CHANGED_STATE_PORTC                 1
#define PIN_CHANGED_STATE_PORTD                 2

#define PIN_CHANGED_STATE_IS_PORT(port)         ((PIN_BUTTON1_PORT == port) || (PIN_BUTTON2_PORT == port) || (PIN_ROTARY_ENC_PORT_CLK == port) || (PIN_ROTARY_ENC_PORT_DT == port))

#define PIN_CHANGED_STATE_HAVE_PORTB            PIN_CHANGED_STATE_IS_PORT(PIN_CHANGED_STATE_PORTB)
#define PIN_CHANGED_STATE_HAVE_PORTC            PIN_CHANGED_STATE_IS_PORT(PIN_CHANGED_STATE_PORTC)
#define PIN_CHANGED_STATE_HAVE_PORTD            PIN_CHANGED_STATE_IS_PORT(PIN_CHANGED_STATE_PORTD)

#pragma push_macro("PORTB")
#pragma push_macro("PORTC")
#pragma push_macro("PORTD")
#undef PORTB
#undef PORTC
#undef PORTD

// helper class for interrupt level changes
struct PinChangedState {

    enum class PortToPinEnum : int8_t {
        #if PIN_CHANGED_STATE_HAVE_PORTB
            PORTB,
            _PORTB = PORTB,
        #endif
        #if PIN_CHANGED_STATE_HAVE_PORTC
            PORTC,
            _PORTC = PORTC,
        #endif
        #if PIN_CHANGED_STATE_HAVE_PORTD
            PORTD,
            _PORTD = PORTD,
        #endif
        MAX,
        #if !PIN_CHANGED_STATE_HAVE_PORTB
            PORTB = -1,
        #endif
        #if !PIN_CHANGED_STATE_HAVE_PORTC
            PORTC = -1,
        #endif
        #if !PIN_CHANGED_STATE_HAVE_PORTD
            PORTD = -1,
        #endif
    };

    using ChangeSetIntType = uint16_t;

    template<uint8_t _Port>
    struct PortIntToPin {
        constexpr operator uint8_t() const {
            return _Port == PIN_CHANGED_STATE_PORTB ? static_cast<uint8_t>(PortToPinEnum::PORTB) :
                _Port == PIN_CHANGED_STATE_PORTC ? static_cast<uint8_t>(PortToPinEnum::PORTC) :
                    _Port == PIN_CHANGED_STATE_PORTD ? static_cast<uint8_t>(PortToPinEnum::PORTD) :
                        static_cast<uint8_t>(PortToPinEnum::MAX);
        }
        static constexpr ChangeSetIntType shl() {
            return (PortIntToPin<_Port>() << 3);
        }
        static constexpr volatile uint8_t *getPin() {
            return _Port == PIN_CHANGED_STATE_PORTB ? &PINB :
                _Port == PIN_CHANGED_STATE_PORTC ? &PINC :
                    _Port == PIN_CHANGED_STATE_PORTD ? &PIND : nullptr;
        }
        // static constexpr volatile uint8_t *getPort() {
        //     return _Port == PIN_CHANGED_STATE_PORTB ? &PORTD :
        //         _Port == PIN_CHANGED_STATE_PORTC ? &PORTC :
        //             _Port == PIN_CHANGED_STATE_PORTD ? &PORTD : nullptr;
        // }
    };

    template<PortToPinEnum _Port>
    struct PortToPin {
        constexpr operator uint8_t() const {
            return static_cast<uint8_t>(_Port);
        }
        static constexpr ChangeSetIntType shl() {
            return PortIntToPin<static_cast<uint8_t>(_Port)>::shl();
        }
        static constexpr volatile uint8_t *getPin() {
            return PortIntToPin<static_cast<uint8_t>(_Port)>::getPin();
        }
        // static constexpr volatile uint8_t *getPort() {
        //     return PortIntToPin<static_cast<uint8_t>(_Port)>::getPort();
        // }
    };

    void update() {
        #if PIN_CHANGED_STATE_HAVE_PORTB
            _pins[PortIntToPin<PIN_CHANGED_STATE_PORTB>()] = PINB;
        #endif
        #if PIN_CHANGED_STATE_HAVE_PORTC
            _pins[PortIntToPin<PIN_CHANGED_STATE_PORTC>()] = PINC;
        #endif
        #if PIN_CHANGED_STATE_HAVE_PORTD
            _pins[PortIntToPin<PIN_CHANGED_STATE_PORTD>()] = PIND;
        #endif
    }

    template<uint8_t _Port>
    void update() {
        _setPort<_Port>(_readPort<_Port>());
    }

    template<uint8_t _Port>
    uint8_t get() const {
        return _getPort<_Port>();
    }

    template<uint8_t _Port, uint8_t bit>
    bool changed(ChangeSetIntType set) const {
        return set & (_BV(bit) << PortIntToPin<_Port>::shl());
    }

private:

    template<uint8_t _Port>
    uint8_t _readPort() const {
        return *PortIntToPin<_Port>::getPin();
    }

    template<uint8_t _Port>
    uint8_t _getPort() const {
        return _pins[PortIntToPin<_Port>()];
    }

    template<uint8_t _Port>
    void _setPort(uint8_t value) {
        _pins[PortIntToPin<_Port>()] = value;
    }

private:
    volatile uint8_t _pins[static_cast<size_t>(PortToPinEnum::MAX)];
};

extern PinChangedState lastState;

#pragma pop_macro("PORTB")
#pragma pop_macro("PORTC")
#pragma pop_macro("PORTD")

// A1/PC1/24
// indicator LED for over current
#define PIN_CURRENT_LIMIT_LED_PORT              PORTC
#define PIN_CURRENT_LIMIT_LED_PIN               PINC
#define PIN_CURRENT_LIMIT_LED_DDR               DDRC
#define PIN_CURRENT_LIMIT_LED_BIT               PINC1
#define PIN_CURRENT_LIMIT_LED_PINNO             A1


inline void setCurrentLimitLedOff()
{
    asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_LED_PORT)), "I" (PIN_CURRENT_LIMIT_LED_BIT));
}

inline void setCurrentLimitLedOn()
{
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_LED_PORT)), "I" (PIN_CURRENT_LIMIT_LED_BIT));
}

inline void setupCurrentLimitLed()
{
    setCurrentLimitLedOff();
    pinMode(PIN_CURRENT_LIMIT_LED_PINNO, OUTPUT);
}

// A0/PC0/23
// ADC input to measure the supply voltage
#define PIN_VOLTAGE                             A0

// A2/PC2/25
// ADC input of the shunt voltage to measure the motor current
// it has an RC filter to get rid of the PWM spikes, no need for software filtering
#define PIN_CURRENT                             A2

// current shunt value in ohm
#ifndef CURRENT_LIMIT_SHUNT
#define CURRENT_LIMIT_SHUNT                     0.008
#endif

// current shunt calibration
#ifndef CURRENT_SHUNT_CALIBRATION
#define CURRENT_SHUNT_CALIBRATION               1.0
#endif

// due to very low voltages at the ADC, low currents might be rounded down to 0
// offset in mA to add
#ifndef CURRENT_SHUNT_OFFSET
#define CURRENT_SHUNT_OFFSET                    70
#endif

// calibation value for the voltage detection
#ifndef VOLTAGE_DETECTION_CALIBRATION
#define VOLTAGE_DETECTION_CALIBRATION           1.0
#endif

// DAC @ 980Hz
namespace DAC {

    static constexpr float kVoltageDividerR1 = 100000.0;
    static constexpr float kVoltageDividerR2 = 6800.0;

    static constexpr float kPwmVoltageMultiplier = ((kVoltageDividerR2 * 5.0) / (kVoltageDividerR1 + kVoltageDividerR2));
    // Amps per PWM step
    static constexpr float kPwmCurrentMultiplier = CURRENT_SHUNT_CALIBRATION * (kPwmVoltageMultiplier / 256) / CURRENT_LIMIT_SHUNT;

}

namespace ADCRef {

    // ADC analog voltage reference
    static constexpr float kReferenceVoltage = 1.1;

    // ADC value to mA/A
    static constexpr float kShuntTomA = (kReferenceVoltage * CURRENT_SHUNT_CALIBRATION * 1000.0 / CURRENT_LIMIT_SHUNT / 1024.0);
    static constexpr float kShuntToA = kShuntTomA / 1000.0;

    static constexpr uint16_t kShuntOffsetmA = CURRENT_SHUNT_OFFSET;
    static constexpr float kShuntOffsetA = CURRENT_SHUNT_OFFSET / 1000.0;
}

// min. duty cycle after the current limit has been tripped
#define CURRENT_LIMIT_MIN_DUTY_CYCLE            8
#if CURRENT_LIMIT_MIN_DUTY_CYCLE == 0 || CURRENT_LIMIT_MIN_DUTY_CYCLE == 255
#error Must not be 0 or 255
#endif
// current limit settings value
#define CURRENT_LIMIT_MIN                       3           // ~0.63A
#define CURRENT_LIMIT_MAX                       224         // ~35A
#define CURRENT_LIMIT_DISABLED                  255

#if HAVE_LED_POWER

// pwm  led power mW
// 1        -50
// 3        5
// 35       150
// 50       650
// 95       1000
// 111      1200
// 127      1650
// 165      2300
// 200      3100
// 238      4785
// 252      5985
// 254.5    6350
// website to create polynominal regression functions from a few samples
// https://arachnoid.com/polysolve/index.html


// pass the PWM value as x and get mW
#define LED_POWER_mW(pwm)                       static_cast<uint16_t>((pwm < LED_MIN_PWM) ? 0 : led_power_polynomial_regress(std::min<int>(255, pwm)))

inline float led_power_polynomial_regress(uint8_t pwm)
{
    static constexpr float pgm_terms[] PROGMEM = {
        -7.0283732122754344e+001,
        1.9031563900783787e+001,
        -3.7251068238367246e-001,
        5.2481034665014231e-003,
        -2.7738833439970790e-005,
        5.1992243727621910e-008
    };
    float t = 1;
    float r = 0;
    for(const auto &term: std::progmem_array<float, std::progmem_ptr_float<float>, sizeof(pgm_terms) / sizeof(*pgm_terms)>(pgm_terms)) {
        r += term * t;
        t *= pwm;
    }
    if (r < 0) {
        return 0;
    }
    return r;
}

#endif

// UI

// set acceleration from 0-255
#define KNOB_ACCELERATION                       64

// -1 is inverted direction in general
// 1 is non-inverted
#define KNOB_INVERTED                           1

// multiplier for menu speed
// < 256 is a multiplier (1 is the biggest multiplier of 256 times)
// > 256 is a divider (32768 is the biggest divider of 128 times)
// negative values will invert the direction
//
// (value * 256) / speed = adjusted value
// (10 * 256) / -512 = -5 - this is basiclly a by 2 divider that also inverts the direction

// negative to invert the menu only
#define KNOB_MENU_SPEED                         (256L * KNOB_INVERTED)
// negative to invert toggling values only
#define KNOB_TOGGLE_SPEDD                       (256L * KNOB_INVERTED)
// negative to invert changing values
#define KNOB_VALUE_SPEED                        (256L * KNOB_INVERTED)
// negative to invert changing pid controller values
#define KNOB_PID_SPEED                          (64L * KNOB_INVERTED)

#define KNOB_GET_VALUE(value, speed)            menu.getKnobValue(value, speed, 1)

#define MENU_LONG_PRESS_MILLIS                  450

#define DISPLAY_MENU_TIMEOUT                    7500
#define DISPLAY_SAVED_TIMEOUT                   1250
#define DISPLAY_BOOT_VERSION_TIMEOUT            1000
#define DISPLAY_REFRESH_TIME                    125

// min. and max. RPM that can be selected in the UI
#define RPM_MIN                                 150
#define RPM_MAX                                 4500

#define STALL_TIME_MIN                          250
#define STALL_TIME_MAX                          5000

// RPM sensing
#define TIMER1_PRESCALER                        1
#define TIMER1_PRESCALER_BV                     _BV(CS10)

// motor PWM
#define TIMER2_PRESCALER                        1
#define TIMER2_PRESCALER_BV                      _BV(CS20); // 31250Hz
// #define PWM_CYCLE_TICKS                         ((F_CPU / TIMER1_PRESCALER) / PWM_FREQUENCY)
// #define PWM_DUTY_CYCLE_TO_TICKS(duty_cycle)     (duty_cycle * static_cast<uint32_t>(PWM_CYCLE_TICKS) / (MAX_DUTY_CYCLE)

namespace Timer1 {

    static constexpr uint8_t kPreScaler = TIMER1_PRESCALER;
    static constexpr float kTicksPerMicrosecond = F_CPU / kPreScaler / 1000000.0;
    static constexpr uint32_t kTicksPerMinute = F_CPU * 60.0 / kPreScaler;

}

namespace Timer2 {

    static constexpr uint8_t kPreScaler = TIMER2_PRESCALER;
    static constexpr float kTicksPerMicrosecond = F_CPU / kPreScaler / 1000000.0;
    static constexpr uint32_t kTicksPerMinute = F_CPU * 60.0 / kPreScaler;

}

// limit duty cycle for pid controller
#define MIN_DUTY_CYCLE_PID                      1UL
#define MAX_DUTY_CYCLE_PID                      255UL
#define START_DUTY_CYCLE_PID                    32UL

// limit duty cycle
#define MIN_DUTY_CYCLE                          24
#define MAX_DUTY_CYCLE                          255

// use rising and falling edge for counting RPM
#ifndef RPM_SENSE_TOGGLE_EDGE
#    define RPM_SENSE_TOGGLE_EDGE 1
#endif

#ifndef RPM_SENSE_PULSES_PER_TURN
// wheel with 120 slots
#    if RPM_SENSE_TOGGLE_EDGE
#       define RPM_SENSE_PULSES_PER_TURN (120 * 2)
#    else
#       define RPM_SENSE_PULSES_PER_TURN 120
#    endif
#endif

// convert rpm to pulse length (ticks)
#define RPM_SENSE_RPM_TO_TICKS(rpm) (Timer1::kTicksPerMinute / (RpmSensing::kPulsesPerTurn * (rpm)))

// convert RPM pulse length (ticks) to RPM
#define RPM_SENSE_TICKS_TO_RPM(pulse) ((Timer1::kTicksPerMinute / RpmSensing::kPulsesPerTurn) / (pulse))

namespace RpmSensing {

    static constexpr uint16_t kPulsesPerTurn = RPM_SENSE_PULSES_PER_TURN;

    static constexpr uint32_t kOneRpmTicks = RPM_SENSE_RPM_TO_TICKS(1);
    static constexpr uint32_t k60RpmTicks = RPM_SENSE_RPM_TO_TICKS(60);
    static constexpr uint32_t k1000RpmTicks = RPM_SENSE_RPM_TO_TICKS(1000);
    static constexpr uint32_t kMinRpmSignalPeriod = RPM_SENSE_RPM_TO_TICKS(RPM_MIN);
    static constexpr uint32_t kMaxRpmSignalPeriod = RPM_SENSE_RPM_TO_TICKS(RPM_MAX);

    // using less RPM than this value will cause an int24_t overflow in the PID controller
    static constexpr uint32_t kMinRpmBeforeOverflow = RPM_SENSE_TICKS_TO_RPM(0xffff);

    static_assert(RPM_MIN > kMinRpmBeforeOverflow, "increase min. RPM");

}

namespace VoltageDetection {

    static constexpr float kR1 = 2700.0;
    static constexpr float kR2 = 100000.0;

    static constexpr float kDivider = (((kR2 + kR1) / kR1) * VOLTAGE_DETECTION_CALIBRATION);

}

inline uint16_t mapValue16(uint16_t value, uint16_t fromMin, uint16_t fromMax, uint16_t toMin, uint16_t toMax)
{
    return
        (value >= fromMax ? toMax : (
            ((value * static_cast<uint32_t>(toMax - toMin) / (fromMax - fromMin)) + (toMin - (fromMin * static_cast<uint32_t>(toMax - toMin) / (fromMax - fromMin))))
        ));
}

#if 0
#define MAP(value, FROM_MIN, FROM_MAX, TO_MIN, TO_MAX) \
    (value <= FROM_MIN ? TO_MIN : ( \
        (value >= FROM_MAX ? TO_MAX : (  \
            ((value * static_cast<uint32_t>(TO_MAX - TO_MIN) / (FROM_MAX - FROM_MIN)) + (TO_MIN - (FROM_MIN * static_cast<uint32_t>(TO_MAX - TO_MIN) / (FROM_MAX - FROM_MIN)))) \
        )) \
    ))
#endif

#define EEPROM_MAGIC                            0xf1c9a548

// add type for variable to start
// (uint8_t)0
// 0UL
// 0.0f
// etc...
#define __for_range_arg_count(...)                          __for_range_arg_count_(,##__VA_ARGS__,3,2,1,0)
#define __for_range_arg_count_(a,b,c,d,cnt,...)             cnt

#define __for_range_first_arg(n0, n1, ...)                  n1
#define __for_range_second_arg(n0, n1, n2, ...)             n2

#define for_range(var, start, end, ...)                     for(auto var = ((__for_range_arg_count(end, ...) == 2 ? (uint8_t)start : start); var < end; var += (__for_range_arg_count(end, ...) == 0 ? 1 : __for_range_first_arg(0, ##__VA_ARGS__, 0)))

enum class ControlModeEnum : uint8_t {
    DUTY_CYCLE = 0,
    PID,
};

enum class MotorStateEnum : uint8_t {
    OFF = 0,
    ON,
    // any "off" state other OFF requires to reset the motor before it can be turned on again
    STALLED,
    ERROR,
    BRAKING
};

enum class PidConfigEnum : uint8_t {
    OFF = 0,
    KP = 1,
    KI,
    KD,
    SAVE,
    RESTORE,
    PID_DEBUG,
    MAX
};

enum class MotorStatusEnum : uint8_t {
    OFF = 0,
    AMPERE,
    WATT,
    MAX
};

class ConfigData;


#include "pid_settings.h"
#include "eeprom_data.h"
#include "config_data.h"
#include "uiconfig_data.h"

extern Adafruit_SSD1306 display;
extern Encoder knob;

template<typename _Ta>
void display_print_hl(bool highlight, _Ta text)
{
    if (highlight) {
        display.setTextColor(BLACK, WHITE);
    }
    display.print(' ');
    display.print(text);
    display.print(' ');
    if (highlight) {
        display.setTextColor(WHITE);
    }
    display.print(' ');
}

void menu_display_submenu();
void refresh_display();
void write_eeprom(const __FlashStringHelper *message = nullptr);
void read_rotary_encoder();
void update_duty_cycle();

class ConfigData;
class UIConfigData;

extern ConfigData data;
extern UIConfigData ui_data;

#include "menu.h"
#include "interrupt_push_button.h"

extern Menu menu;
extern InterruptPushButton<PIN_BUTTON1_PORT, _BV(PIN_BUTTON1_BIT)> button1;
extern InterruptPushButton<PIN_BUTTON2_PORT, _BV(PIN_BUTTON2_BIT)> button2;

inline void display_message(const char *message, uint16_t time, uint8_t size = 2, size_t len = ~0U)
{
    if (len == ~0U) {
        len = strlen(message);
    }
    const uint8_t y = (SCREEN_HEIGHT / 2) - (size * FONT_HEIGHT) + 3;
    const uint8_t x = (SCREEN_WIDTH / 2) - (len * size * (FONT_WIDTH / 2));
    display.clearDisplay();
    display.setCursor(x, y);
    display.setTextSize(size);
    display.print(message);
    display.display();
    ui_data.setRefreshTimeoutOnce(time);
}

inline void display_message(const __FlashStringHelper *message, uint16_t time, uint8_t size = 2)
{
    const size_t len = strlen_P(reinterpret_cast<PGM_P>(message));
    const size_t strSize = len + 1;
    char buf[strSize];
    memcpy_P(buf, message, strSize);
    display_message(buf, time, size, len);
}

inline void restart_device()
{
    for(int8_t i = 0; i < 5; i++) {
        setCurrentLimitLedOn();
        delay(75);
        setCurrentLimitLedOff();
        delay(75);
    }
    wdt_enable(WDTO_15MS);
    for(;;) {}
}
