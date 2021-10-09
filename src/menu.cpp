/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "motor.h"
#include "pid_control.h"
#include "current_limit.h"

void Menu::display()
{
    if (isActive()) {
        menu_display_submenu();
        return;
    }

    __display().clearDisplay();
    __display().setTextSize(1);
    _displayItem(0, _position + size() - 1);
    _displayItem(12, _position);
    _displayItem(24, _position + 1);
    // highlight
    __display().fillRoundRect(5, 10, SCREEN_WIDTH - 15, 12, 4, SSD1306_INVERSE);

    // scrollbar
    __display().drawRoundRect(SCREEN_WIDTH - 8, 0, 8, SCREEN_HEIGHT, 3, WHITE);
    uint8_t y = 2 + (_position * (SCREEN_HEIGHT - 6) / size());
    __display().fillRoundRect(SCREEN_WIDTH - 7, y, 6, 6, 3, WHITE);

    __display().display();
}

void rotary_button_released(Button& btn, uint16_t duration)
{
    bool longPress = duration > Timeouts::UI::kLongPress;
    if (motor.isOn() || (longPress && !menu.isOpen())) {
        // motor running
        if (longPress) {
            switch(data.pidConfig()) {
                case PidConfigEnum::SAVE:
                case PidConfigEnum::RESTORE:
                    update_motor_settings(1);
                    break;
                default:
                    break;
            }
        }
        else if (motor.isVelocityMode()) {
            data.pidConfig() = static_cast<PidConfigEnum>((static_cast<uint8_t>(data.pidConfig()) + 1) % static_cast<uint8_t>(PidConfigEnum::MAX));
            ui_data.refreshDisplay();
        }
        return;
    }
    else if (menu.isOpen()) {
        // inside menu
        if (longPress) {
            // exit menu on long press or exit menu item
            menu.close();
        }
        else if (menu.isMainMenuActive()) {
            // inside main menu
            switch(menu.getPosition()) {
                case MenuEnum::MENU_EXIT:
                    menu.close();
                    break;
                case MenuEnum::MENU_RESTORE: {
                        menu.closeNoRefresh();
                        data = ConfigData();
                        motor = Motor();
                        pid.resetPidValues();
                        write_eeprom(F("RESTORED"));
                        delay(500);
                        restart_device();
                    }
                    break;
                default:
                    // enter sub menu
                    menu.enter();
                    break;
            }
        }
        else if (menu.isActive()) {
            // inside sub menu, exit...
            menu.exit();
        }
    }
    else {
        // main screen, open menu
        menu.open();
    }
    // reset encoder after a short delay in case pressing cause it to move slightly
    delay(25);
    knob.write(0);
}

void start_stop_button_pressed(Button& btn)
{
    // skip check if the menu is open
    if (menu.isClosed()) {
        if (motor.isOn()) {
            motor.stop(MotorStateEnum::BRAKING);
            menu.close();
        }
        else {
            knob.write(0);
            motor.start();
        }
    }
    else {
        menu.close();
    }
}

void read_rotary_encoder()
{
    int16_t value = knob.readAndReset();
    if (!value) {
        return;
    }
    if (menu.isMainMenuActive()) { // main menu
        menu.setPosition(menu.getPositionInt() + (KNOB_GET_VALUE(value, KNOB_MENU_SPEED) > 0 ? 1 : -1));
    }
    else if (menu.isActive()) { // sub menus
        switch(menu.getPosition()) {
            case MenuEnum::MENU_SPEED:
                update_motor_settings(value);
                break;
            #if HAVE_LED
                case MenuEnum::MENU_LED:
                    data.setLedBrightness(std::clamp<int16_t>(data.getLedBrightness() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), LED_MIN_PWM - 1, LED_MAX_PWM));
                    data.updateLedBrightnessNoDelay();
                    break;
            #endif
            #if HAVE_CURRENT_LIMIT
                case MenuEnum::MENU_CURRENT: {
                    int16_t new_value = current_limit.getLimit() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED);
                    if (new_value >= ILimit::kMax && value > 0) {
                        current_limit.setLimit(ILimit::kDisabled);
                    }
                    else {
                        current_limit.setLimit(std::clamp<int16_t>(new_value, ILimit::kMin, ILimit::kMax));
                    }
                } break;
            #endif
            case MenuEnum::MENU_PWM:
                motor.setMaxDutyCycle(std::clamp<int16_t>(motor.getMaxDutyCycle() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE));
                break;
            case MenuEnum::MENU_STALL:
                motor.setMaxStallTime(std::clamp<int16_t>(motor.getMaxStallTime() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), STALL_TIME_MIN, STALL_TIME_MAX));
                break;
            #if HAVE_RPM_PER_VOLT
                case MenuEnum::MENU_MOTOR:
                    data.setRpmPerVolt(std::clamp<int16_t>(data.getRpmPerVolt() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), 0, 32500));
                    break;
            #endif
            case MenuEnum::MENU_MODE:
                motor.toggleMode();
                break;
            case MenuEnum::MENU_DISPLAY:
                data.toggleDisplayMotorStatus();
                break;
            case MenuEnum::MENU_BRAKE:
                motor.enableBrake(!motor.isBrakeEnabled());
                break;
            default:
                break;
        }
        menu.resetTimer();
    }
    else {
        // main screen
        update_motor_settings(value);
    }
}
