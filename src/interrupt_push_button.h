/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

template<uint8_t _Port, uint8_t _BitMask>
class InterruptPushButton : public PushButton {
public:
    InterruptPushButton(uint8_t button, uint8_t options) : PushButton(button, options) {
    }

protected:
    virtual boolean _update_button_state() override {
        return !(lastState.get<_Port>() & _BitMask);
    }

private:
    uint8_t _port;
};
