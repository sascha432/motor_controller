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
    current_limit.timer1CompareMatchA();
}

#pragma GCC optimize("Os")

#endif