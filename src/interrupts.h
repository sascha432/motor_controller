/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"

enum PinChangesEnum : uint8_t {
    NONE = 0,
    BUTTON1 = _BV(0),
    BUTTON2 = _BV(1),
    KNOB = _BV(2),
};

using PinChangedType = volatile PinChangesEnum;

extern PinChangedType pinChangedFlag;

void setup_interrupts();
