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

CurrentLimit current_limit;

#if HAVE_CURRENT_LIMIT

uint8_t CurrentLimit::_getDutyCycle()
{
    if (motor.isVelocityMode()) {
        return pid.getDutyCycle();
    }
    else {
        return data.getSetPointDutyCycle();
    }
}

void CurrentLimit::_resetDutyCycle()
{
    enable();
    if (motor.isOn()) {
        // restore pwm value if the motor is on
        setMotorPWM_timer(_getDutyCycle());
    }
}

ISR(TIMER1_COMPA_vect)
{
    current_limit.timer1CompareMatchA();
}

#pragma GCC optimize("Os")

#endif