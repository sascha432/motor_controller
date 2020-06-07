/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "rpm_sensing.h"

void UIData_t::refreshDisplay()
{
    refresh_timer = 0;
}

void UIData_t::disableRefreshDisplay()
{
    refresh_timer = ~0;
}

void UIData_t::menuResetAutoCloseTimer()
{
    refresh_timer = millis() + DISPLAY_MENU_TIMEOUT;
}

bool UIData_t::readKnobValue()
{
    if (millis() <= knob_read_timer) {
        return false;
    }
    knob_read_timer = millis() + KNOB_READ_TIME;
    return true;
}

void UIData_t::updateDutyCyle()
{
    display_pulse_length_integral = (display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + capture_timer_get_micros()) / (DISPLAY_RPM_MULTIPLIER + 1);
}
