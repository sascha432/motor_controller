/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

#define VERSION_MAJOR                           1
#define VERSION_MINOR                           0
#define VERSION_PATCH                           0

#if DEBUG
#define DEBUG_PID_CONTROLLER                    1
#define DEBUG_INPUTS                            0
#define DEBUG_RPM_SIGNAL                        0
#else
#define DEBUG_PID_CONTROLLER                    0
#define DEBUG_INPUTS                            0
#define DEBUG_RPM_SIGNAL                        0
#endif

// pins

// always use set_motor_speed() to change PIN_MOTOR_PWM or PIN_BRAKE to avoid damage of the motor controller
#define PIN_MOTOR_PWM                           11
#define PIN_BRAKE                               9

#define PIN_LED_DIMMER                          5
#define PIN_CURRENT_LIMIT                       6
#define PIN_CURRENT_LIMIT_OVERRIDE              10

#define PIN_RPM_SIGNAL                          8
#define PIN_BUTTON1                             7
#define PIN_BUTTON2                             4
#define PIN_ROTARY_ENC_CLK                      2
#define PIN_ROTARY_ENC_DT                       3
#define PIN_VOLTAGE                             A0

#define MCU_VOLTAGE                            5.0

// current limit
#define CURRENT_LIMIT_MIN                       6       // ~1.0A
// max. ~230
#define CURRENT_LIMIT_MAX                       220     // ~38.4A
#define CURRENT_LIMIT_DISABLED                  255
// DAC @ 980Hz, 1020µs / 3.99µs steps
// resistor divdivers, needs to be adjusted to real values
#define CURRENT_LIMIT_DAC_R1                    100.0
#define CURRENT_LIMIT_DAC_R2                    1000.0
#define CURRENT_LIMIT_DAC_R3                    27.0
#define CURRENT_LIMIT_SHUNT                     3       // 0.003R / 3 milliohm
// mV to current
#define CURRENT_LIMIT_SHUNT_mV_TO_A(value)      (value / CURRENT_LIMIT_SHUNT)
// max. voltage of the DAC ~4.5563V
#define CURRENT_LIMIT_DAC_VOLTAGE               (MCU_VOLTAGE * (1 - (1 / ((CURRENT_LIMIT_DAC_R1 + CURRENT_LIMIT_DAC_R2 + CURRENT_LIMIT_DAC_R3) / CURRENT_LIMIT_DAC_R1))))
// voltage per PWM step in mV, might need some adjustments due to tolerances
#define CURRENT_LIMIT_DAC_TO_mV(value)          ((value * (MCU_VOLTAGE / 256.0)) / ((CURRENT_LIMIT_DAC_R2 + CURRENT_LIMIT_DAC_R3) / CURRENT_LIMIT_DAC_R3) * 1020.0)
// vref = current limit at the LM393, max ~120mV/40A with a 3 milliohm shunt
#define CURRENT_LIMIT_DAC_TO_CURRENT(value)    CURRENT_LIMIT_SHUNT_mV_TO_A(CURRENT_LIMIT_DAC_TO_mV(value))


// LED PWM 980Hz for MT3608
#define LED_MIN_PWM                             13 // 5%

// UI
#define KNOB_ACCELERATION                       5
#define KNOW_MENU_MULTIPLIER                    -5

#define KNOB_MENU_CENTER                        (MENU_COUNT * 1000L * abs(KNOW_MENU_MULTIPLIER))

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

#define DISPLAY_RPM_MULTIPLIER                  50UL
#define DISPLAY_DUTY_CYCLE_MULTIPLIER           10UL

// min. and mx. RPM that can be selected
#define RPM_MIN                                 600
#define RPM_MAX                                 3500

#define STALL_TIME_MIN                          250
#define STALL_TIME_MAX                          2500

// pulse length / RPM
#define RPM_MIN_PULSE_LENGTH                    RPM_SENSE_US_TO_RPM(RPM_MAX)
#define RPM_MAX_PULSE_LENGTH                    RPM_SENSE_US_TO_RPM(RPM_MIN)

// RPM sensing
#define TIMER1_PRESCALER                        1
#define TIMER1_PRESCALER_BV                     _BV(CS10)
#define TIMER1_TICKS_PER_US                     (F_CPU / TIMER1_PRESCALER / 1000000.0)

// motor PWM
#define TIMER2_PRESCALER                        _BV(CS20); // 31250Hz
// #define TIMER2_PRESCALER                        _BV(CS21); // 3906Hz
#define PWM_CYCLE_TICKS                         ((F_CPU / TIMER1_PRESCALER) / PWM_FREQUENCY)
#define PWM_DUTY_CYCLE_TO_TICKS(duty_cycle)     (duty_cycle * (uint32_t)PWM_CYCLE_TICKS / MAX_DUTY_CYCLE)

// limit duty cycle
#define MIN_DUTY_CYCLE                          10
#define MAX_DUTY_CYCLE                          255

#ifndef HAVE_VOLTAGE_DETECTION
#define HAVE_VOLTAGE_DETECTION                  1
// resistor divider 10K/100K, needs to be adjusted to real values
#define VOLTAGE_DETECTION_R1                    10.5f
#define VOLTAGE_DETECTION_R2                    100.0f
#define VOLTAGE_DETECTION_DIVIDER               ((VOLTAGE_DETECTION_R2 + VOLTAGE_DETECTION_R1) / VOLTAGE_DETECTION_R1)
#endif

#define MAP(value, FROM_MIN, FROM_MAX, TO_MIN, TO_MAX) \
    (value <= FROM_MIN ? TO_MIN : ( \
        (value >= FROM_MAX ? TO_MAX : (  \
            ((value * (uint32_t)(TO_MAX - TO_MIN) / (FROM_MAX - FROM_MIN)) + (TO_MIN - (FROM_MIN * (uint32_t)(TO_MAX - TO_MIN) / (FROM_MAX - FROM_MIN)))) \
        )) \
    ))

#define POTI_TO_RPM(value)                      MAP(value, POTI_MIN, POTI_MAX, RPM_MIN, RPM_MAX)
#define RPM_TO_POTI(value)                      MAP(value, RPM_MIN, RPM_MAX, POTI_MIN, POTI_MAX)

#define POTI_TO_DUTY_CYCLE(value)               MAP(value, POTI_MIN, POTI_MAX, MIN_DUTY_CYCLE, MAX_DUTY_CYCLE)

#define EEPROM_MAGIC                            0xf1c9a542

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

typedef enum : uint8_t {
    DUTY_CYCLE = 0,
    PID,
} ControlModeEnum_t;

typedef enum : uint8_t {
    MENU_SPEED = 0,
    MENU_MODE,
    MENU_LED,
    MENU_CURRENT,
    MENU_STALL,
    MENU_INFO,
    MENU_EXIT,
    MENU_COUNT,
} MenuEnum_t;

typedef enum : uint8_t {
    OFF = 0,
    ON,
    // any "off" state other OFF required to reset the motor before it can be turned on again
    STARTUP,
    STALLED,
    ERROR,
} MotorStateEnum_t;

typedef struct {
    ControlModeEnum_t control_mode;
    MotorStateEnum_t motor_state;
    uint8_t brake_enaged: 1;
    uint8_t set_point_input;
    uint8_t led_brightness;
    uint8_t current_limit;
    uint16_t max_stall_time;
} Data_t;

typedef struct {
    uint32_t knob_read_timer;
    uint32_t refresh_timer;
    uint16_t refresh_counter;
    uint8_t display_duty_cycle_integral;
    uint16_t display_pulse_length_integral;
} UIData_t;

typedef struct {
    uint32_t magic;
    ControlModeEnum_t control_mode;
    uint8_t set_point_input;
    uint8_t led_brightness;
    uint8_t current_limit;
    uint16_t max_stall_time;
    float Kp;
    float Ki;
    float Kd;
} EEPROMData_t;

extern Data_t data;
extern UIData_t ui_data;

void set_motor_speed(int speed);
void motor_stop(MotorStateEnum_t state = OFF);
void motor_start();
void update_duty_cycle();
void display_message(char *message, uint16_t time, uint8_t size = 2);
void set_current_limit();
void set_led_brightness();
