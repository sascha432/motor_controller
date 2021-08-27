/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

#define FPSTR(str)                              reinterpret_cast<const __FlashStringHelper *>(str)

#define _DECL_T(name, value)                    extern const char _text_##name[] PROGMEM;
#define _DEF_T(name, value)                     const char _text_##name[] PROGMEM = { value }
#define _T(name)                                _text_##name
#define _F(name)                                FPSTR(_T(name))

_DECL_T(OFF, "OFF");
_DECL_T(DISABLED, "DISABLED");
_DECL_T(ENABLED, "ENABLED");
_DECL_T(ERROR, "ERROR");
_DECL_T(STALLED, "STALLED");
_DECL_T(BRAKING, "BRAKING");
_DECL_T(SAVED, "SAVED");
_DECL_T(_rpm, " rpm");
