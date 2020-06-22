/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include "main.h"

class CurrentLimit {
public:
    CurrentLimit();

    void begin();

    void enable(bool state);
    uint8_t getDutyCycle(uint8_t duty_cycle, bool reset = false);

    bool isDisabled() const;
    uint8_t getLimit() const;
    void setLimit(uint8_t limit);

    void updateLimit();

#if HAVE_CURRENT_LIMIT

    void pinISR(bool state);
    void compareAISR();

private:
    enum class CurrentLimitStateEnum : uint8_t {
        NOT_TRIPPED,
        SIGNAL_HIGH,
        SIGNAL_LOW,
        RESET
    };

    uint8_t _limit;
    volatile uint32_t _timer;
    volatile CurrentLimitStateEnum _state;
    volatile bool _enabled;
#endif
};

extern CurrentLimit current_limit;
