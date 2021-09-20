/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <util/atomic.h>
#include <wiring_private.h>
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>
#include <Encoder.h>
#include <Adafruit_SSD1306.h>
#include "helpers.h"
#include "progmem_strings.h"
#include "int24_types.h"

#define VERSION_MAJOR                           1
#define VERSION_MINOR                           0
#define VERSION_PATCH                           4

// show at what time and date the firmware was compiled
#define HAVE_COMPILED_ON_DATE                   1

#if DEBUG
#define DEBUG_INPUTS                            1
#define DEBUG_RPM_SIGNAL                        0
#define DEBUG_MOTOR_SPEED                       1
#else
#define DEBUG_INPUTS                            0
#define DEBUG_RPM_SIGNAL                        0
#define DEBUG_MOTOR_SPEED                       0
#endif

// enable serial commands
// >2700 byte (includes HAVE_SERIAL_HELP, HAVE_SERIAL_MOTOR_CONTROL, HAVE_VOLTAGE_DETECTION etc...)
#ifndef HAVE_SERIAL_COMMANDS
#define HAVE_SERIAL_COMMANDS                    1
#endif

// 31382-28644

// dimming the LED driver is supported
// ~150 byte
#ifndef HAVE_LED_FADING
#define HAVE_LED_FADING                         1
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
#ifndef HAVE_CURRENT_LIMIT
#define HAVE_CURRENT_LIMIT                      1
#endif

// flash over current LED to indicate that the device is idling
// ~200 byte
#ifndef CURRENT_LIMIT_LED_IDLE_INDICATOR
#define CURRENT_LIMIT_LED_IDLE_INDICATOR        0
#endif

// for CURRENT_LIMIT_LED_IDLE_INDICATOR

// flash on/off-time interval
#define FLASH_ON_TIME                           250
// number of subsequent flashes
#define FLASH_NUM_TIMES                         3
// mask to decrease counter, use 0 to disable it
// set FLASH_WAIT_PERIOD in milliseconds if the mask is disabled
#define FLASH_COUNTER_MASK                      ((1 << 6) - 1)
// time between flashing the LED n times
#if FLASH_COUNTER_MASK != 0
// wait for counter overflow, FLASH_COUNTER_MASK * FLASH_ON_TIME milliseconds
#    define FLASH_WAIT_PERIOD                   ((0xff & FLASH_COUNTER_MASK) * FLASH_ON_TIME)
#else
// wait period in milliseconds
#    define FLASH_WAIT_PERIOD                   15000
#endif

// pins

// D11/PB3/15
#define PIN_MOTOR_PWM_PIN                       PINB
#define PIN_MOTOR_PWM_DDR                       DDRB
#define PIN_MOTOR_PWM_PORT                      PORTB
#define PIN_MOTOR_PWM_BIT                       PINB3
#define PIN_MOTOR_PWM_OCR                       OCR1A
#define PIN_MOTOR_ENABLE_PWM()                  sbi(TCCR1A, COM1A1)
#define PIN_MOTOR_DISABLE_PWM()                 cbi(TCCR1A, COM1A1)

inline void stopMotor()
{
    PIN_MOTOR_DISABLE_PWM();
    asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_PORT)), "I" (PIN_MOTOR_PWM_BIT));
}

inline void setupMotorPwm()
{
    asm volatile ("cbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_PORT)), "I" (PIN_MOTOR_PWM_BIT));
    asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_DDR)), "I" (PIN_MOTOR_PWM_BIT));
}

inline void stopMotorAtomic()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        stopMotor();
    }
}

// do not use for 0 or 255
// PWM must be enabled
inline void setMotorPWM_timer(uint8_t pwm)
{
    PIN_MOTOR_PWM_OCR = pwm;
}

// do not use for 0
inline void setMotorPWMAtomic(uint8_t pwm)
{
#if DEBUG && 0
    if (pwm == 0) {
        Serial.println(F("setMotorPWMAtomic(0) called"));
        for (;;) { delay(1); }
    }
#endif
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (pwm == 255) {
            PIN_MOTOR_DISABLE_PWM();
            asm volatile ("sbi %0, %1" :: "I" (_SFR_IO_ADDR(PIN_MOTOR_PWM_PORT)), "I" (PIN_MOTOR_PWM_BIT));
        }
        else {
            PIN_MOTOR_ENABLE_PWM();
            setMotorPWM_timer(pwm);
        }
    }
}

// D7/PD7/11
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
#define PIN_LED_DIMMER                          5

// D6/PD6/10
#define PIN_CURRENT_LIMIT_PWM                   6

// A1/PC1/24
#define PIN_CURRENT_SHUNT                       A1

// D10/PB2/14
#define PIN_CURRENT_LIMIT_OVERRIDE_PIN          PINB
#define PIN_CURRENT_LIMIT_OVERRIDE_PORT         PORTB
#define PIN_CURRENT_LIMIT_OVERRIDE_DDR          DDRB
#define PIN_CURRENT_LIMIT_OVERRIDE_BIT          PINB2

// D12/PB4/MISO
#define PIN_CURRENT_LIMIT_INDICATOR_PINNO       12
#define PIN_CURRENT_LIMIT_INDICATOR_PIN         PINB
#define PIN_CURRENT_LIMIT_INDICATOR_PORT        PORTB
#define PIN_CURRENT_LIMIT_INDICATOR_PCHS_PORT   PIN_CHANGED_STATE_PORTB
#define PIN_CURRENT_LIMIT_INDICATOR_DDR         DDRB
#define PIN_CURRENT_LIMIT_INDICATOR_BIT         PINB4

inline bool isCurrentLimitTripped()
{
    asm volatile goto (
        "sbis %0, %1\n\t"
        "rjmp %l[LIMIT]\n\t"
        :: "I" (_SFR_IO_ADDR(PIN_CURRENT_LIMIT_INDICATOR_PIN)), "I" (PIN_CURRENT_LIMIT_INDICATOR_BIT) :: LIMIT
    );
    return false;

LIMIT:
    return true;
}

#define PIN_RPM_SIGNAL                          8           // D8/PB0/12

#define PIN_BUTTON1                             9           // D9/PB1/13
#define PIN_BUTTON1_PORT                        PIN_CHANGED_STATE_PORTB
#define PIN_BUTTON1_BIT                         PINB1

#define PIN_BUTTON2                             4           // D4/PD4/2
#define PIN_BUTTON2_PORT                        PIN_CHANGED_STATE_PORTD
#define PIN_BUTTON2_BIT                         PIND4

#define PIN_ROTARY_ENC_CLK                      2           // D2/PD2/32
#define PIN_ROTARY_ENC_CLK_PORT                 PIN_CHANGED_STATE_PORTD
#define PIN_ROTARY_ENC_CLK_BIT                  PIND2
#define PIN_ROTARY_ENC_CLK_PIN                  PIND

#define PIN_ROTARY_ENC_DT                       3           // D3/PD3/1
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

// A0/PC0/23
#define PIN_VOLTAGE                             A0

// A2/PC2/25
#define PIN_CURRENT                             A2

// current shunt value in ohm
#define CURRENT_LIMIT_SHUNT                     0.008

// current shunt calibration
#ifndef CURRENT_SHUNT_CALIBRATION
#define CURRENT_SHUNT_CALIBRATION               1.0
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

    static constexpr float kShuntTomA = (kReferenceVoltage * CURRENT_SHUNT_CALIBRATION * 1000.0 / CURRENT_LIMIT_SHUNT / 1024.0);
    static constexpr float kShuntToA = kShuntTomA / 1000.0;

}

// convert ADC value to mA/A
#define CURRENT_SHUNT_TO_mA(adc, counter)       ((adc) * (ADCRef::kShuntTomA / counter))
#define CURRENT_SHUNT_TO_A(adc, counter)        ((adc) * (ADCRef::kShuntToA / counter))

// convert PWM to A
#define CURRENT_LIMIT_DAC_TO_CURRENT(pwm)       (pwm * DAC::kPwmCurrentMultiplier)

// after the current limit has been trigger, ramp up the pwm signal slowly. time in microseconds
#define CURRENT_LIMIT_RAMP_UP_PERIOD            1500

// min. duty cycle after the current limit has been tripped
#define CURRENT_LIMIT_MIN_DUTY_CYCLE            20
#if CURRENT_LIMIT_MIN_DUTY_CYCLE == 0 || CURRENT_LIMIT_MIN_DUTY_CYCLE == 255
#error Must not be 0 or 255
#endif

// delay after starting the motor in milliseconds
#define CURRENT_LIMIT_DELAY                     500

// current limit settings value
#define CURRENT_LIMIT_MIN                       3           // ~0.63A
#define CURRENT_LIMIT_MAX                       224         // ~35A
#define CURRENT_LIMIT_DISABLED                  255

// LED PWM 980Hz for MT3608
#define LED_MIN_PWM                             32
#define LED_MAX_PWM                             240     // more than 240 does not increase brightness much for my LEDs
#define LED_FADE_TIME                           10

#if LED_MIN_PWM < 5
#error increase min. pwm
#endif

// display power consumption of the LEDs
#ifndef HAVE_LED_POWER
#define HAVE_LED_POWER 1
#endif

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

#define KNOB_GET_VALUE(value, speed)            menu.getKnobValue(value, speed, 1)

#define MENU_LONG_PRESS_MILLIS                  450

// poti
#define POTI_MIN                                0
#define POTI_MAX                                255
#define POTI_RANGE                              POTI_MAX
#define POTI_VALUE(value)                       value
// #define POTI_MAX                                1023
// #define POTI_RANGE                              (POTI_MAX - POTI_MIN)
// #define POTI_VALUE(value)                       (value <= POTI_MIN ? POTI_MIN : (value >= POTI_MAX ? POTI_MAX : value - POTI_MIN))

#define DISPLAY_MENU_TIMEOUT                    7500
#define DISPLAY_SAVED_TIMEOUT                   1250
#define DISPLAY_BOOT_VERSION_TIMEOUT            1000
#define DISPLAY_REFRESH_TIME                    100

#define DISPLAY_RPM_MULTIPLIER                  50UL
#define DISPLAY_DUTY_CYCLE_MULTIPLIER           10UL

// min. and mx. RPM that can be selected
#define RPM_MIN                                 150
#define RPM_MAX                                 4500

#define STALL_TIME_MIN                          250
#define STALL_TIME_MAX                          5000

// pulse length / RPM
#define RPM_MIN_PULSE_LENGTH                    RPM_SENSE_US_TO_RPM(RPM_MAX)
#define RPM_MAX_PULSE_LENGTH                    RPM_SENSE_US_TO_RPM(RPM_MIN)

// dynamic averaging of the rpm values. 0 to disable
// in case a 3D printed sensor is used and the pulses are not 100% even, this will
// help to improve the reponse of the PID controller with low RPMs and make it
// more smooth with high RPMs. can be tuned while the motor is running using the
// serial console and HAVE_SERIAL_COMMANDS (key a/s)
#define RPM_SENSE_AVERAGING_FACTOR              8

// RPM sensing
#define TIMER1_PRESCALER                        1
#define TIMER1_PRESCALER_BV                     _BV(CS10)
#define TIMER1_TICKS_PER_US                     (F_CPU / TIMER1_PRESCALER / 1000000.0)

// motor PWM
#define TIMER2_PRESCALER                        1
#define TIMER2_PRESCALER_BV                      _BV(CS20); // 31250Hz
#define PWM_CYCLE_TICKS                         ((F_CPU / TIMER1_PRESCALER) / PWM_FREQUENCY)
#define PWM_DUTY_CYCLE_TO_TICKS(duty_cycle)     (duty_cycle * (uint32_t)PWM_CYCLE_TICKS / MAX_DUTY_CYCLE)

// limit duty cycle
#define VELOCITY_START_DUTY_CYCLE               40
#define MIN_DUTY_CYCLE                          1
#define MAX_DUTY_CYCLE                          255

namespace VoltageDetection {

    static constexpr float kR1 = 2700.0;
    static constexpr float kR2 = 100000.0;

    static constexpr float kDivider = (((kR2 + kR1) / kR1) * VOLTAGE_DETECTION_CALIBRATION);

}

#define MAP(value, FROM_MIN, FROM_MAX, TO_MIN, TO_MAX) \
    (value <= FROM_MIN ? TO_MIN : ( \
        (value >= FROM_MAX ? TO_MAX : (  \
            ((value * (uint32_t)(TO_MAX - TO_MIN) / (FROM_MAX - FROM_MIN)) + (TO_MIN - (FROM_MIN * (uint32_t)(TO_MAX - TO_MIN) / (FROM_MAX - FROM_MIN)))) \
        )) \
    ))


#define POTI_TO_RPM(value)                      MAP(value, POTI_MIN, POTI_MAX, RPM_MIN, RPM_MAX)
#define RPM_TO_POTI(value)                      MAP(value, RPM_MIN, RPM_MAX, POTI_MIN, POTI_MAX)

#define POTI_TO_DUTY_CYCLE(value)               MAP(value, POTI_MIN, POTI_MAX, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)

#define EEPROM_MAGIC                            0xf1c9a544

#if DEBUG_PID_CONTROLLER

#define PID_TEST_OUTPUT_INTERVAL                250
#define PID_TEST_WAIT                           1500            // wait for motor to stop
#define PID_TEST_SET_POINTS                     10
#define IS_PID_TUNING                           (pid_tune_increment != 0)

extern float pid_tune_increment;
extern uint16_t pid_test_set_points[PID_TEST_SET_POINTS];
extern uint32_t pid_test_timer_start;
extern uint32_t pid_test_timer_end;
extern uint32_t pid_test_duration;
extern uint16_t pid_test_micros_offset;
extern uint16_t pid_test_counter;

#else

#define IS_PID_TUNING                           false

#endif

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
    STARTUP,
    STALLED,
    ERROR,
    BRAKING
    // CURRENT_LIMIT
};

enum class PidConfigEnum : uint8_t {
    OFF = 0,
    KP = 1,
    KI,
    KD,
    OMUL,
    DTMUL,
    SAVE,
    RESTORE,
    MAX
};

class ConfigData;

class EEPROMData {
public:
    uint32_t magic;
    ControlModeEnum control_mode;
    uint8_t set_point_input_velocity;
    uint8_t set_point_input_pwm;
    uint8_t led_brightness;
    uint8_t current_limit;
    uint8_t brake_enabled;
    uint16_t max_stall_time;
    float Kp;
    float Ki;
    float Kd;
    uint8_t max_pwm;
    uint16_t rpm_per_volt;

    constexpr size_t size() const {
        return sizeof(*this);
    }

    bool operator!=(const EEPROMData &data) const;
    EEPROMData &operator=(const ConfigData &data);
};

inline bool EEPROMData::operator!=(const EEPROMData &data) const
{
    return memcmp(this, &data, size());
}

class ConfigData {
public:
    ConfigData();

    ConfigData &operator=(const EEPROMData &data);

    void setLedBrightness();
    void setLedBrightnessNoDelay();

    uint8_t getSetPoint() const;
    uint16_t getSetPointRPM() const;
    uint16_t getSetPointDutyCycle() const;
    void setSetPoint(uint8_t value);
    void changeSetPoint(int8_t value);

    void setRpmPerVolt(uint16_t rpmV);
    uint16_t getRpmPerVolt() const;

    PidConfigEnum pid_config;
    uint8_t led_brightness;
    uint8_t rpm_sense_average;

private:
    friend EEPROMData;

    uint16_t rpm_per_volt;
    #if HAVE_LED_FADING
        uint32_t led_fade_timer;
    #endif
    uint8_t led_brightness_pwm;
    uint8_t set_point_input_velocity;
    uint8_t set_point_input_pwm;
};

inline void ConfigData::setLedBrightnessNoDelay()
{
    #if HAVE_LED_FADING
        led_fade_timer = 0;
        led_brightness_pwm = led_brightness;
        if (led_brightness_pwm > 0) {
            led_brightness_pwm--;
        }
        else {
            led_brightness_pwm++;
        }
    #endif
    setLedBrightness();
}

inline uint16_t ConfigData::getSetPointRPM() const
{
    return POTI_TO_RPM(getSetPoint());
}

inline uint16_t ConfigData::getSetPointDutyCycle() const
{
    return POTI_TO_DUTY_CYCLE(getSetPoint());
}

inline void ConfigData::setRpmPerVolt(uint16_t rpmV)
{
    rpm_per_volt = rpmV;
}

inline uint16_t ConfigData::getRpmPerVolt() const
{
    return rpm_per_volt;
}


class UIConfigData {
public:
    UIConfigData() = default;

    void refreshDisplay();
    void disableRefreshDisplay();
    bool checkReadKnobTimeout();
    void updateDutyCyle();
    void updateDutyCyle(uint32_t length);
    void updateRpmPulseWidth(uint32_t length);

    volatile uint32_t refresh_timer;
    uint16_t refresh_counter;
    uint8_t display_duty_cycle_integral;
    uint16_t display_pulse_length_integral;
    volatile uint32_t display_current_limit_timer;
};

inline void UIConfigData::refreshDisplay()
{
    refresh_timer = 0;
}

inline void UIConfigData::disableRefreshDisplay()
{
    refresh_timer = ~0;
}

extern ConfigData data;
extern UIConfigData ui_data;

template<uint8_t _Port, uint8_t _BitMask>
class InterruptPushButton : public PushButton {
public:
    InterruptPushButton(uint8_t button, uint8_t options) : PushButton(button, options) {
    }

protected:
    virtual boolean _update_button_state() override {
        return !(lastState.get<_Port>() & _BitMask);
    }

private:
    uint8_t _port;
};

extern Adafruit_SSD1306 display;
extern Encoder knob;

void menu_display_submenu();
void print_pid_cfg(char *buffer, size_t len);
void refresh_display();
void write_eeprom(const __FlashStringHelper *message = _F(SAVED));
void read_rotary_encoder();
void update_duty_cycle();

#include "menu.h"

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
    ui_data.refresh_timer = millis() + time;
}

inline void display_message(const __FlashStringHelper *message, uint16_t time, uint8_t size = 2)
{
    const size_t len = strlen_P(reinterpret_cast<PGM_P>(message));
    const size_t strSize = len + 1;
    char buf[strSize];
    memcpy_P(buf, message, strSize);
    display_message(buf, time, size, len);
}
