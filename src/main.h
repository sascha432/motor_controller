/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <util/atomic.h>
#include "progmem_strings.h"

#define VERSION_MAJOR                           1
#define VERSION_MINOR                           0
#define VERSION_PATCH                           4

#define HAVE_COMPILED_ON_DATE                   1

#if DEBUG
#define DEBUG_INPUTS                            0
#define DEBUG_RPM_SIGNAL                        0
#define DEBUG_MOTOR_SPEED                       1
#else
#define DEBUG_INPUTS                            0
#define DEBUG_RPM_SIGNAL                        0
#define DEBUG_MOTOR_SPEED                       0
#endif

#ifndef HAVE_SERIAL_COMMANDS
#define HAVE_SERIAL_COMMANDS                    1
#endif

#ifndef HAVE_LED_FADING
// +162 byte code size
#define HAVE_LED_FADING                         1
#endif

#ifndef HAVE_DEBUG_RPM_SIGNAL_OUT
// output RPM signal on port PIN_RPM_SGINAL_DEBUG_OUT (D13/PB5/SCK) which is available on the debug pin header
#define HAVE_DEBUG_RPM_SIGNAL_OUT               0
#endif

#ifndef HAVE_CURRENT_LIMIT
// +328 byte code size
#define HAVE_CURRENT_LIMIT                      1
#endif

// pins

// always use set_motor_speed() to change PIN_MOTOR_PWM or PIN_BRAKE to avoid damage of the motor controller
#define PIN_MOTOR_PWM                           11          // D11/PB3/15
#if PIN_MOTOR_PWM == 11
// TIMER2A
// set duty cycle directly. only works if analogWrite() was called before with a value between 1 and 254.
// use analogWrite() or digitalWrite() to set to 0/LOW or 255/HIGH
// the value is not limited to data.max_pwm
#define MOTOR_SET_DUTY_CYCLE(pwm)               OCR2A = pwm;
#else
#warning add timer for PIN_MOTOR_PWM
#define MOTOR_SET_DUTY_CYCLE(pwm)               analogWrite(PIN_MOTOR_PWM, pwm);
#endif

#define PIN_BRAKE                               7           // D7/PD7/11

#define PIN_LED_DIMMER                          5           // D5/PD5/9

#define PIN_CURRENT_LIMIT                       6           // D6/PD6/10
#define PIN_CURRENT_LIMIT_OVERRIDE              10          // D10/PB2/14
#define PIN_CURRENT_LIMIT_INDICATOR             12          // D12/PB4/MISO

#define PIN_RPM_SIGNAL                          8           // D8/PB0/12
#define PIN_RPM_SIGNAL_DEBUG_OUT                13          // D13/PB5/SCK

#define PIN_BUTTON1                             9           // D9/PB1/13
#define PIN_BUTTON2                             4           // D4/PD4/2
#define PIN_ROTARY_ENC_CLK                      2           // D2/PD2/32
#define PIN_ROTARY_ENC_DT                       3           // D3/PD3/1

#define PIN_CURRENT_LIMIT_LED                   15/*A1*/    // A1/PC1/24
#if PIN_CURRENT_LIMIT_LED == 15
#define PIN_CURRENT_LIMIT_LED_PORT              PORTC
#define PIN_CURRENT_LIMIT_LED_PIN               PINC
#define PIN_CURRENT_LIMIT_LED_PIN_BV            _BV(PINC1)
#else
#error PORT not defined
#endif



#define PIN_VOLTAGE                             A0          // A0/PC0/23

// pin interrupt setup
#if HAVE_CURRENT_LIMIT
#if PIN_CURRENT_LIMIT_INDICATOR == 12
#define PIN_CURRENT_LIMIT_INDICATOR_MASK        _BV(PINB4)
#else
#error add mask for PIN_CURRENT_LIMIT_INDICATOR, can only be PINB
#endif
#endif

#if HAVE_DEBUG_RPM_SIGNAL_OUT
#if PIN_RPM_SIGNAL == 8
#define PIN_RPM_SIGNAL_MASK                     (1 << PINB0)
#else
#error add mask for PIN_RPM_SIGNAL, can only be PINB
#endif
#if PIN_RPM_SIGNAL_DEBUG_OUT == 13
#define PIN_RPM_SIGNAL_DEBUG_OUT_MASK           (1 << PINB5)
#else
#error add mask for PIN_RPM_SIGNAL, can only be PINB
#endif
#endif

#define MCU_VOLTAGE                             5.0

// current limit
#define CURRENT_LIMIT_MIN                       3           // ~0.63A
#define CURRENT_LIMIT_MAX                       224         // ~35A
#define CURRENT_LIMIT_DISABLED                  255
// DAC @ 980Hz, 1020µs / 3.99µs steps
// voltage dividers

#define CURRENT_LIMIT_DAC_R1a                   100.0
#define CURRENT_LIMIT_DAC_R1b                   1000.0
#define CURRENT_LIMIT_DAC_R1                    (CURRENT_LIMIT_DAC_R1a + CURRENT_LIMIT_DAC_R1b)
#define CURRENT_LIMIT_DAC_R2                    75.0
#define CURRENT_LIMIT_SHUNT                     8           // 0.008R / 3 milliohm

// VREF, maximum voltage for the DAC
#define CURRENT_LIMIT_DAC_MAX_VOLTAGE           ((MCU_VOLTAGE * CURRENT_LIMIT_DAC_R2) / (CURRENT_LIMIT_DAC_R1 + CURRENT_LIMIT_DAC_R2))

// mV to current
//#define CURRENT_LIMIT_SHUNT_mV_TO_A(value)      (value / CURRENT_LIMIT_SHUNT)

// voltage per PWM step in mV, might need some adjustments due to tolerances
//#define CURRENT_LIMIT_DAC_TO_mV(value)          (value * (CURRENT_LIMIT_DAC_MAX_VOLTAGE / 255.0)) / ((CURRENT_LIMIT_DAC_R2 + CURRENT_LIMIT_DAC_R3) / CURRENT_LIMIT_DAC_R3)

 //#define CURRENT_LIMIT_DAC_TO_CURRENT(value)    CURRENT_LIMIT_SHUNT_mV_TO_A(CURRENT_LIMIT_DAC_TO_mV(value))

#define CURRENT_LIMIT_DAC_TO_CURRENT(value)     ((value * 0.156446f) + (value < 7 ? 0.35 : (value < 13 ? 0.2 : 0)))


// after the current limit has been trigger, ramp up the pwm signal slowly. time in microseconds
#define CURRENT_LIMIT_RAMP_UP_PERIOD            1500

// min. duty cycle after the current limit has been tripped
#define CURRENT_LIMIT_MIN_DUTY_CYCLE            20
#if CURRENT_LIMIT_MIN_DUTY_CYCLE == 0 || CURRENT_LIMIT_MIN_DUTY_CYCLE == 255
#error Must not be 0 or 255
#endif

// delay after starting the motor in milliseconds
#define CURRENT_LIMIT_DELAY                     500

// LED PWM 980Hz for MT3608
#define LED_MIN_PWM                             25
#define LED_MAX_PWM                             255
#define LED_MENU_SPEED                          3
#define LED_FADE_TIME                           10

// UI
#define KNOB_ACCELERATION                       15
#define KNOB_INVERTED                           1                       // -1 to invert direction
#define KNOB_MENU_MULTIPLIER                    (5 * KNOB_INVERTED)     // negative to invert the menu only
#define KNOB_READ_TIME                          150

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
#define RPM_MIN                                 100
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
// rpm    avg (factor 3)
// >300   1
// >426   2
// >521   3
// >600   4
// >672   5
// >735   6
// >795   7
// >849   8
// rpm    avg (factor 5)
// >390   1
// >550   2
// >672   3
// >775   4
// >868   5
// >950   6
// >1025  7
// >1096  8
// rpm    avg (factor 8)
// >492   1
// >696   2
// >850   3
// >984   4
// >1096  5
// >1200  6
// >1297  7
// >1388  8
// rpm    avg (factor 15)
// >675   1
// >953   2
// >1169  3
// >1349  4
// >1500  5
// >1650  6
// >1780  7
// >1905  8
// rpm    avg (factor 20)
// >780   1
// >1100  2
// >1344  3
// >1559  4
// >1740  5
// >1900  6
// >2059  7
// >2200  8

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

#ifndef HAVE_VOLTAGE_DETECTION
#define HAVE_VOLTAGE_DETECTION                  1
#endif
// voltage divider 100K/1000K, needs to be adjusted to real values to be accurate
#define VOLTAGE_DETECTION_R1                    100000UL
#define VOLTAGE_DETECTION_R2                    1000000UL
#ifndef VOLTAGE_DETECTION_CALIBRATION
#define VOLTAGE_DETECTION_CALIBRATION           1.0
#endif
#define VOLTAGE_DETECTION_DIVIDER               (((VOLTAGE_DETECTION_R2 + VOLTAGE_DETECTION_R1) / VOLTAGE_DETECTION_R1) * VOLTAGE_DETECTION_CALIBRATION)

#define HAVE_INTERRUPTS                         ((HAVE_CURRENT_LIMIT && PIN_CURRENT_LIMIT_INDICATOR) || HAVE_DEBUG_RPM_SIGNAL_OUT)


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

#if HAVE_INT24

#include <stdint.h>
#include <stddef.h>

using uint24_t = __uint24;
using int24_t = __int24;

#else

using uint24_t = uint32_t;
using int24_t = int32_t;

#endif

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

typedef struct {
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
} EEPROMData_t;

class Data_t {
public:
    Data_t();

    void copyTo(EEPROMData_t &eeprom_data);
    void copyFrom(const EEPROMData_t &eeprom_data);
    void setLedBrightness();
    inline void setLedBrightnessNoDelay() {
#if HAVE_LED_FADING
        led_fade_timer = 0;
        led_brightness_pwm = led_brightness;
        if (led_brightness_pwm > 0) {
            led_brightness_pwm--;
        } else {
            led_brightness_pwm++;
        }
#endif
        setLedBrightness();
    }

    uint8_t getSetPoint() const;
    uint16_t getSetPointRPM() const {
        return POTI_TO_RPM(getSetPoint());
    }
    uint16_t getSetPointDutyCycle() const {
        return POTI_TO_DUTY_CYCLE(getSetPoint());
    }
    void setSetPoint(uint8_t value);
    void changeSetPoint(int8_t value);

    PidConfigEnum pid_config;
    uint8_t led_brightness;
    uint8_t rpm_sense_average;

private:
#if HAVE_LED_FADING
    uint32_t led_fade_timer;
#endif
    uint8_t led_brightness_pwm;
    uint8_t set_point_input_velocity;
    uint8_t set_point_input_pwm;
};

class UIData_t {
public:
    UIData_t() = default;

    void refreshDisplay();
    void disableRefreshDisplay();
    void menuResetAutoCloseTimer();
    bool readKnobValue();
    void updateDutyCyle();
    void updateDutyCyle(uint32_t length);
    void updateRpmPulseWidth(uint32_t length);

    uint32_t knob_read_timer;
    volatile uint32_t refresh_timer;
    uint16_t refresh_counter;
    uint8_t display_duty_cycle_integral;
    uint16_t display_pulse_length_integral;
    volatile uint32_t display_current_limit_timer;
};

inline void UIData_t::refreshDisplay()
{
    refresh_timer = 0;
}

inline void UIData_t::disableRefreshDisplay()
{
    refresh_timer = ~0;
}

inline void UIData_t::menuResetAutoCloseTimer()
{
    refresh_timer = millis() + DISPLAY_MENU_TIMEOUT;
}

inline bool UIData_t::readKnobValue()
{
    if (millis() <= knob_read_timer) {
        return false;
    }
    knob_read_timer = millis() + KNOB_READ_TIME;
    return true;
}

extern Data_t data;
extern UIData_t ui_data;

#include "menu.h"
#include <Adafruit_SSD1306.h>

extern Menu menu;
extern Adafruit_SSD1306 display;

void update_duty_cycle();

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

void menu_display_value();
void print_pid_cfg(char *buffer, size_t len);
void refresh_display();
float getVoltage();