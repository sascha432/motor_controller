    /**
 * Author: sascha_lammers@gmx.de
 */

#include <avr/wdt.h>
#include "main.h"
#include "motor.h"
#include "interrupts.h"
#include "pid_control.h"
#include "rpm_sensing.h"
#include "current_limit.h"
#include "adc.h"

Motor motor;

void Motor::loop()
{
    if (motor.isOn()) {

        // check if the RPM signal has not been updated for a given period of time
        auto last = rpm_sense.getLastSignalTicks();
        if (last > _maxStallTime * Timer1::kTicksPerMillisecond) {
            // Serial.print(F("last signal "));
            // Serial.println(last/Timer1::kTicksPerMillisecond);
            #if DEBUG_MOTOR_SPEED
                if (motor.isOn()) {
                    Serial.printf_P(PSTR("dur=%lu max=%u\n"), dur, _maxStallTime);
                }
            #endif

            if (motor.isOn()) {
                stop(MotorStateEnum::STALLED);
            }
            else if (isBrakeOn()) { // release brake
                setBrake(false);
                if (menu.isClosed()) {
                    ui_data.refreshDisplay();
                }
            }
        }

        wdt_reset();
    }
}

void Motor::start()
{
    // if (isBrakeOn()) {
    //     // if the brake is still engaged, for example when turning the motor off and on quickly
    //     // turn the brake off, update the display and wait 100ms
    //     #if DEBUG_MOTOR_SPEED
    //         Serial.print(F("brake_on"));
    //     #endif
    //     setBrake(false);
    //     ui_data.refreshDisplay();
    //     refresh_display();
    //     delay(100);

    //     // make sure it is off
    //     if (isBrakeOn()) {
    //         stop(MotorStateEnum::ERROR);
    //         return;
    //     }
    // }
    setBrake(false);

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        // enable WDT to restart in case of a crash while the motor is running
        wdt_enable(WDTO_1S);

        // reset ui data
        ui_data = {};
        _state = MotorStateEnum::ON;
        // _startTime = micros();

        // reset rpm sensing and pid controller
        rpm_sense.reset();
        pid.reset();

        // reset / enable current limit
        current_limit.enable();

        // set motor speed
        if (isDutyCycleMode()) {
            setSpeed(data.getSetPointDutyCycle());
        }
        else {
            setSpeed(START_DUTY_CYCLE_PID);
        }
    }
    ui_data.refreshDisplay();
}

void Motor::stop(MotorStateEnum state)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _state = state;
        setSpeed(0);
        current_limit.disable();
    }

    const __FlashStringHelper *message;
    switch (state) {
        case MotorStateEnum::ERROR:
            message = F("ERROR");
            break;
        case MotorStateEnum::STALLED:
            message = F("STALLED");
            break;
        case MotorStateEnum::OFF:
            message = F("OFF");
            break;
        default:
            message = motor.isBrakeEnabled() ? F("BRAKING") : F("OFF");
            break;
    }
    display_message(message, Timeouts::Menu::kMessage);
    data.pidConfig() = PidConfigEnum::OFF;

    wdt_disable();
}

// set PWM duty cycle with enabling/disabling the brake and turning the motor signal led on/off
// called inside ISR
// the PID controller does not use this method
void Motor::setSpeed(uint8_t speed)
{
    #if DEBUG
        if (!isOn() && speed != 0) {
            #if DEBUG_MOTOR_SPEED
                Serial.print(F("error_speed"));
            #endif
            stop(MotorStateEnum::ERROR);
            return;
        }
    #endif
    if (speed > _maxPWM) {
        speed = _maxPWM;
    }
    if (speed != 0) {
        if (isBrakeOn()) {
            #if DEBUG_MOTOR_SPEED
                Serial.print(F("error_brake"));
            #endif
            stop(MotorStateEnum::ERROR);
            return;
        }
        speed = current_limit.getDutyCycle(speed);
        #if DEBUG_MOTOR_SPEED
            Serial.printf_P(PSTR("speed=%u\n"), (int)speed);
        #endif
        setMotorPWMAtomic(speed);
    }
    else if (speed == 0) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (isOn()) {
                #if DEBUG_MOTOR_SPEED
                    Serial.printf_P(PSTR("set_mode=%u\n"), (int)_mode);
                #endif
                _state = MotorStateEnum::OFF;
            }
        }
        // turns motor off as well
        setBrake(true);
    }
}

