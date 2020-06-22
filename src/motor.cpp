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

Motor::Motor() : _mode(ControlModeEnum::PID), _state(MotorStateEnum::OFF), _startTime(0), _maxPWM(MAX_DUTY_CYCLE), _dutyCycle(0), _maxStallTime(1000), _brake(true)
{
}

void Motor::begin()
{
    digitalWrite(PIN_BRAKE, LOW);
    pinMode(PIN_BRAKE, OUTPUT);

    digitalWrite(PIN_MOTOR_PWM, LOW);
    pinMode(PIN_MOTOR_PWM, OUTPUT);
}

void Motor::loop()
{
#if HAVE_CURRENT_LIMIT
    if (_startTime) {
        if (get_time_diff(_startTime, millis()) > CURRENT_LIMIT_DELAY) {
            current_limit.enable(true);
            _startTime = 0;
        }
    }
#endif

    // check if the RPM signal has not been updated for a given period of time
    if (millis() > rpm_sense.getLastSignalMillis() + _maxStallTime) {
        if (motor.isOn()) {
            stop(MotorStateEnum::STALLED);
        }
        else if (digitalRead(PIN_BRAKE)) { // release brake
            digitalWrite(PIN_BRAKE, LOW);
            if (menu.isClosed()) {
                ui_data.refreshDisplay();
            }
        }
    }
}

void Motor::start()
{
    if (digitalRead(PIN_BRAKE)) {
        // if the brake is still engaged, for example turning the motor off and on quickly, turn it off, update the display and wait 100ms
        digitalWrite(PIN_BRAKE, LOW);
        ui_data.refreshDisplay();
        delay(100);

        // make sure it is off
        if (digitalRead(PIN_BRAKE)) {
            stop(MotorStateEnum::ERROR);
            return;
        }
    }
    current_limit.enable(false);
    ui_data.display_current_limit_timer = 0;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        ui_data = UIData_t();
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
    refresh_display();
}

void Motor::stop(MotorStateEnum state)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _state = state;
        setSpeed(0);
    }
    // current_limit.enable(false);
    refresh_display();

    char message[16];
    switch (state) {
    case MotorStateEnum::ERROR:
        strcpy_P(message, _T(ERROR));
        break;
    case MotorStateEnum::STALLED:
        strcpy_P(message, _T(STALLED));
        break;
    case MotorStateEnum::BREAKING:
    default:
        strcpy_P(message, _T(BREAKING));
        break;
    }
    display_message(message, DISPLAY_MENU_TIMEOUT / 2);
    data.pid_config = PidConfigEnum::OFF;

    current_limit.enable(true);
}

// set PWM duty cycle with enabling/disabling the brake and turning the motor signal led on/off
// called inside ISR
void Motor::setSpeed(uint8_t speed)
{
    if (!isOn() && speed != 0) {
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
        if (digitalRead(PIN_BRAKE)) {
            stop(MotorStateEnum::ERROR);
            return;
        }
        speed = current_limit.getDutyCycle(speed);
    }
    analogWrite(PIN_MOTOR_PWM, speed);
    if (speed == 0) {
        digitalWrite(PIN_MOTOR_PWM, LOW);
        if (_brake) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (isOn()) {
                    _state = MotorStateEnum::OFF;
                }
                digitalWrite(PIN_MOTOR_PWM, LOW);
                delayMicroseconds(50);
            }
            digitalWrite(PIN_BRAKE, HIGH);
        }
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

void Motor::toggleMode()
{
    if (isVelocityMode()) {
        setMode(ControlModeEnum::DUTY_CYCLE);
    }
    else {
        setMode(ControlModeEnum::PID);
    }
}

void Motor::setMode(ControlModeEnum mode)
{
    if (_state != MotorStateEnum::ON) {
        _mode = mode;
        if (isVelocityMode()) {
            rpm_sense.setCallback(update_pid_controller);
        }
        else {
            rpm_sense.setCallback(update_duty_cycle);
        }
    }
}

void Motor::_calcPulseLength()
{
    auto rpm = data.getSetPointRPM();
    pid.set_point_rpm_pulse_length = RPM_SENSE_RPM_TO_US(rpm);
#if RPM_SENSE_AVERAGING_FACTOR
    data.rpm_sense_average = (rpm * (rpm / RPM_SENSE_AVERAGING_FACTOR)) / 30000;
    // Serial.print("rpm_sense_average ");
    // Serial.println(data.rpm_sense_average);
#endif
}

void Motor::setMaxDutyCycle(uint8_t maxPWM)
{
    _maxPWM = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, maxPWM));
}

void Motor::setMaxStallTime(uint16_t maxStallTime)
{
    _maxStallTime = max(STALL_TIME_MIN, min(STALL_TIME_MAX, maxStallTime));
}

void Motor::enableBrake(bool enable)
{
    _brake = enable;
    if (!_brake) {
        digitalWrite(PIN_BRAKE, LOW);
    }
}