/**
 * Author: sascha_lammers@gmx.de
 */

#include "current_limit.h"
#include "main.h"
#include "motor.h"
#include "pid_control.h"

#if HAVE_CURRENT_LIMIT

CurrentLimit current_limit;

uint8_t CurrentLimit::_getDutyCycle()
{
    if (motor.isVelocityMode()) {
        return pid.getDutyCycle();
    }
    else {
        return data.getSetPointDutyCycle();
    }
}

ISR(TIMER1_COMPA_vect)
{
    current_limit.timer1CompareMatchA();
}

#endif