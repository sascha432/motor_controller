/**
 * Author: sascha_lammers@gmx.de
 */

#include "current_limit.h"
#include "main.h"
#include "motor.h"
#include "pid_control.h"

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif

#if HAVE_CURRENT_LIMIT

CurrentLimit current_limit;

ISR(TIMER1_COMPA_vect)
{
    current_limit.compareAISR();
}

uint8_t CurrentLimit::getDutyCycle(uint8_t duty_cycle, bool reset)
{
    switch(_state) {
        case CurrentLimitStateEnum::NOT_TRIPPED:
        case CurrentLimitStateEnum::RESET:
            return duty_cycle;
        case CurrentLimitStateEnum::SIGNAL_HIGH:
            if (reset) {
                if (isCurrentLimitTripped()) {
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
            setCurrentLimitLedOff();
            if (motor.isVelocityMode()) {
                // reset pid controller to get rid of any Ki/Kd error that built up during the current limiting
                pid.reset();
            }
        }
    }
    return duty_cycle;
}


void CurrentLimit::updateLimit()
{
    analogWriteCurrentLimitPwm(_limit);
    if (_limit == CURRENT_LIMIT_DISABLED) {
        // set comperator vref to 5V >1000A
        sbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
        sbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    }
    else {
        // floating = use current limit
        cbi(PIN_CURRENT_LIMIT_OVERRIDE_PORT, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
        cbi(PIN_CURRENT_LIMIT_OVERRIDE_DDR, PIN_CURRENT_LIMIT_OVERRIDE_BIT);
    }
}

#pragma GCC optimize("Os")

#endif