/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"

typedef struct {
    uint8_t current_limit_flag: 1;
    uint8_t rpm_sense_flag: 1;
} InterruptTriggeredFlags_t;

#if DEBUG_TRIGGERED_INTERRUPTS
#define SET_INTERRUPT_TRIGGER(var, val)             interrupt_trigger_flags.var = val;
#define GET_INTERRUPT_TRIGGER(var)                  interrupt_trigger_flags.var
extern volatile InterruptTriggeredFlags_t interrupt_trigger_flags;
#else
#define SET_INTERRUPT_TRIGGER(var, val)             ;
#define GET_INTERRUPT_TRIGGER(var)                  false
#endif

void setup_interrupts();
