/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "current_limit.h"
#include "motor.h"
#include "pid_control.h"

CurrentLimit current_limit;

#if HAVE_CURRENT_LIMIT

ISR(TIMER1_COMPA_vect)
{
    current_limit.compareAISR();
}

// increase frequency from 4 times per second to 1000 after the limit has been tripped
#define CURRENT_LIMIT_TICKS     ((F_CPU / TIMER1_PRESCALER / 1000) - 1)

CurrentLimit::CurrentLimit() : _limit(CURRENT_LIMIT_DISABLED), _timer(0), _state(CurrentLimitStateEnum::NOT_TRIPPED), _enabled(false)
{
}

void CurrentLimit::begin()
{
    pinMode(PIN_CURRENT_LIMIT, OUTPUT);
    pinMode(PIN_CURRENT_LIMIT_INDICATOR, INPUT);
    pinMode(PIN_CURRENT_LIMIT_LED, OUTPUT);
}

void CurrentLimit::enable(bool state)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _enabled = state;
        _state = CurrentLimitStateEnum::NOT_TRIPPED;
        _timer = 0;
        if (state) {
            TIMSK1 |= _BV(OCIE1A);
        }
        else {
            TIMSK1 &= ~_BV(OCIE1A);
        }
    }
}

uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle, bool reset)
{
    switch(_state) {
        case CurrentLimitStateEnum::NOT_TRIPPED:
        case CurrentLimitStateEnum::RESET:
            return duty_cycle;
        case CurrentLimitStateEnum::SIGNAL_HIGH:
            if (reset) {
                if (PINB & PIN_CURRENT_LIMIT_INDICATOR_MASK) {
                    _timer = micros();
                    ui_data.display_current_limit_timer = millis();
                    if (motor.isOn()) {
                        ui_data.refresh_timer = ui_data.display_current_limit_timer + 1000;
                    }
                    return CURRENT_LIMIT_MIN_DUTY_CYCLE;
                }
                else {
                    _state = CurrentLimitStateEnum::SIGNAL_LOW;
                }
            }
            break;
        default:
            break;

    }
    // slowly ramp up duty cycle again
    uint32_t time = get_time_diff(_timer, micros());
    if (time < CURRENT_LIMIT_RAMP_UP_PERIOD) {
        constexpr const uint16_t fp_mul = 256;
        uint16_t multiplier = (time * fp_mul) / CURRENT_LIMIT_RAMP_UP_PERIOD;
        uint8_t dc = (((duty_cycle - CURRENT_LIMIT_MIN_DUTY_CYCLE) * multiplier) / fp_mul) + CURRENT_LIMIT_MIN_DUTY_CYCLE;
        if (dc < duty_cycle) {
            return dc;
        }
    }
    // mark for reset
    if (reset) {
        if (_state == CurrentLimitStateEnum::SIGNAL_LOW) {
            _state = CurrentLimitStateEnum::NOT_TRIPPED;
            ui_data.refresh_timer = millis() + 250;
            PIN_CURRENT_LIMIT_LED_PORT &= ~PIN_CURRENT_LIMIT_LED_PIN_BV;
            if (motor.isVelocityMode()) {
                // reset pid controller to get rid of any Ki/Kd error that built up during the current limiting
                pid.reset();
            }
        }
    }
    return duty_cycle;
}

bool CurrentLimit::isDisabled() const
{
    return _limit == CURRENT_LIMIT_DISABLED;
}

uint8_t CurrentLimit::getLimit() const
{
    return _limit;
}

void CurrentLimit::setLimit(uint8_t limit)
{
    if (limit == CURRENT_LIMIT_DISABLED) {
        _limit = CURRENT_LIMIT_DISABLED;
    }
    else {
        _limit = max(CURRENT_LIMIT_MIN, min(CURRENT_LIMIT_MAX, limit));
    }
}

void CurrentLimit::updateLimit()
{
    analogWrite(PIN_CURRENT_LIMIT, _limit);
    if (_limit == CURRENT_LIMIT_DISABLED) {
        pinMode(PIN_CURRENT_LIMIT_OVERRIDE, OUTPUT);
        digitalWrite(PIN_CURRENT_LIMIT_OVERRIDE, HIGH);     // set comparator vref to 5V >1000A
    }
    else {
        digitalWrite(PIN_CURRENT_LIMIT_OVERRIDE, LOW);
        pinMode(PIN_CURRENT_LIMIT_OVERRIDE, INPUT);         // floating = use current limit
    }
}

void CurrentLimit::pinISR(bool state)
{
    if (_enabled) { // state changed
        if (state) { // rising edge, store time if not set
            if (_state != CurrentLimitStateEnum::SIGNAL_HIGH) {
                _timer = micros();
                _state = CurrentLimitStateEnum::SIGNAL_HIGH;
                if (motor.isOn()) {
                    MOTOR_SET_DUTY_CYCLE(CURRENT_LIMIT_MIN_DUTY_CYCLE);
                }
                if (ui_data.display_current_limit_timer == 0) {
                    ui_data.display_current_limit_timer = millis();
                    if (motor.isOn()) {
                        ui_data.refresh_timer = ui_data.display_current_limit_timer + 1000; // disable display, i2c causes interferences
                    }
                }
                OCR1A = TCNT1 + CURRENT_LIMIT_TICKS;
                PIN_CURRENT_LIMIT_LED_PORT |= PIN_CURRENT_LIMIT_LED_PIN_BV;
            }
        }
    }
}

void CurrentLimit::compareAISR()
{
    if (_state != CurrentLimitStateEnum::NOT_TRIPPED) {
        // in PWM mode set motor speed continuously to update the duty cycle
        auto dc = getDutyCycle(motor.getDutyCycle(), true);
        if (motor.isOn()) {
            MOTOR_SET_DUTY_CYCLE(dc);
        }
        OCR1A = TCNT1 + CURRENT_LIMIT_TICKS;
    }
}

#else

CurrentLimit::CurrentLimit()
{
}

void CurrentLimit::begin()
{
    digitalWrite(PIN_CURRENT_LIMIT, HIGH);
    pinMode(PIN_CURRENT_LIMIT, OUTPUT);
    digitalWrite(PIN_CURRENT_LIMIT_OVERRIDE, HIGH);
    pinMode(PIN_CURRENT_LIMIT_OVERRIDE, OUTPUT);
}

void CurrentLimit::disable()
{
}

void CurrentLimit::enable(bool state)
{
}

uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle, bool reset)
{
}

bool CurrentLimit::isDisabled() const
{
    return true;
}

uint8_t CurrentLimit::getLimit() const
{
    return 0;
}

void CurrentLimit::setLimit(uint8_t limit)
{
}

void CurrentLimit::updateLimit()
{
}

#endif