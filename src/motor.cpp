/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "motor.h"
#include "interrupts.h"
#include "pid_control.h"
#include "rpm_sensing.h"
#include "current_limit.h"

Motor motor;

inline void update_pid_controller()
{
    pid.update();
}

inline void update_duty_cycle()
{
    ui_data.updateDutyCyle();
}

void Motor::loop()
{
    #if HAVE_CURRENT_LIMIT
        if (_startTime) {
            if (millis() - _startTime > CURRENT_LIMIT_DELAY) {
                current_limit.enable(true);
                _startTime = 0;
            }
        }
    #endif

    // check if the RPM signal has not been updated for a given period of time
    if (millis() - rpm_sense.getLastSignalMillis() > _maxStallTime) {
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
}

void Motor::start()
{
    if (isBrakeOn()) {
        #if DEBUG_MOTOR_SPEED
            Serial.print(F("brake_on"));
        #endif
        // if the brake is still engaged, for example turning the motor off and on quickly, turn it off, update the display and wait 100ms
        setBrake(false);
        ui_data.refreshDisplay();
        refresh_display();
        delay(100);

        // make sure it is off
        if (isBrakeOn()) {
            stop(MotorStateEnum::ERROR);
            return;
        }
    }
    current_limit.enable(false);
    ui_data.display_current_limit_timer = 0;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        ui_data = {};
        _state = MotorStateEnum::ON;
        _startTime = millis();
        rpm_sense.reset();
        pid.reset();
        if (isDutyCycleMode()) {
            updateMotorSpeed();
        }
        else {
            _calcPulseLength();
            setSpeed(VELOCITY_START_DUTY_CYCLE);
        }
    }
    ui_data.refreshDisplay();
    // refresh_display();
}

void Motor::stop(MotorStateEnum state)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _state = state;
        setSpeed(0);
    }
    // current_limit.enable(false);
    // refresh_display();

    auto message = _F(OFF);
    switch (state) {
    case MotorStateEnum::ERROR:
        message = _F(ERROR);
        break;
    case MotorStateEnum::STALLED:
        message = _F(STALLED);
        break;
    case MotorStateEnum::BRAKING:
        message = motor.isBrakeEnabled() ? _F(BRAKING) : _F(OFF);
        break;
    default:
        break;
    }
    display_message(message, DISPLAY_MENU_TIMEOUT / 2);
    data.pid_config = PidConfigEnum::OFF;

    current_limit.enable(true);
}

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif
// set PWM duty cycle with enabling/disabling the brake and turning the motor signal led on/off
// called inside ISR
void Motor::setSpeed(uint8_t speed)
{
    if (!isOn() && speed != 0) {
        #if DEBUG_MOTOR_SPEED
            Serial.print(F("error_speed"));
        #endif
        stop(MotorStateEnum::ERROR);
        return;
    }
    if (speed > _maxPWM) {
        speed = _maxPWM;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _dutyCycle = speed;
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

void Motor::updateMotorSpeed()
{
    if (isDutyCycleMode()) {
        #if RPM_SENSE_AVERAGING_FACTOR
            data.rpm_sense_average = 0;
        #endif
        pid.duty_cycle = data.getSetPointDutyCycle();
        if (isOn()) {
            setSpeed(pid.duty_cycle);
        }
    }
    else {
        _calcPulseLength();
    }
}

#pragma GCC optimize("Os")

void Motor::setMode(ControlModeEnum mode)
{
    if (_state != MotorStateEnum::ON) {
        #if DEBUG_MOTOR_SPEED
            Serial.printf_P(PSTR("set_mode=%u\n"), (int)_mode);
        #endif
        _mode = mode;
        if (isVelocityMode()) {
            rpm_sense.setCallback(update_pid_controller);
        }
        else {
            rpm_sense.setCallback(update_duty_cycle);
        }
    }
}

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif

void Motor::_calcPulseLength()
{
    auto rpm = data.getSetPointRPM();
    pid.set_point_rpm_pulse_length = RPM_SENSE_RPM_TO_US(rpm);
    #if RPM_SENSE_AVERAGING_FACTOR
        data.rpm_sense_average = (rpm * (rpm / RPM_SENSE_AVERAGING_FACTOR)) / 30000;
        // Serial.print("rpm_sense_average ");
        // Serial.println(data.rpm_sense_average);
    #endif
    #if 0
        Serial.print(rpm);
        Serial.print(' ');
        Serial.println(pid.set_point_rpm_pulse_length);
    #endif
}

