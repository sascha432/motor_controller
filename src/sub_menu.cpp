/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "pid_control.h"
#include "current_limit.h"
#include "adc.h"

void menu_display_submenu()
{
    char message[64];

    display.clearDisplay();
    if (menu.getPosition() == MenuEnum::MENU_INFO) {
        // display info menu
        display.setTextSize(1);
        display.setCursor(0, 0);

        set_version(message);
        display.println(message);

        pid.printValues(display, false);

        #if HAVE_LED
            _ledBrightness_str(message, sizeof(message));
        #endif
        #if HAVE_LED_POWER
            display.printf_P(PSTR("LED %s %umW\n"), message, LED_POWER_mW(data.getLedBrightness()));
        #else
            display.printf_P(PSTR("LED %s\n"), message);
        #endif

        #if HAVE_VOLTAGE_DETECTION
            display.print(F("VCC "));
            display.print(adc.getVoltageAvg_V(), 2);
            display.print('V');
        #endif
        #if HAVE_CURRENT_LIMIT
            if (current_limit.isEnabled()) {
                display.print(F(" max "));
                display.print(current_limit.getLimitAmps(), 1);
                display.print('A');
            }
        #endif

        // int8_t overflow = display.getCursorY() - (SCREEN_HEIGHT);
        // if (overflow > 0) {
        //     Serial.print(overflow);
        //     Serial.print(' ');
        //     Serial.println(overflow / FONT_HEIGHT); //TODO add scrolling
        // }

        ui_data.setRefreshTimeout(250);
    }
    else {
        // display menu
        menu.displayTitle();
        switch(menu.getPosition()) {
            case MenuEnum::MENU_SPEED:
                if (motor.isDutyCycleMode()) {
                    PrintBuffer buf(message, sizeof(message));
                    buf.print(data.getSetPointDutyCyclePercent(), 1);
                    buf.print('%');
                }
                else {
                    sprintf_P(message, PSTR("%u rpm"), data.getSetPointRPM());
                }
                break;
            case MenuEnum::MENU_MODE:
                strcpy_P(message, motor.isVelocityMode() ? PSTR("Velocity") : PSTR("PWM"));
                break;
            #if HAVE_LED
                case MenuEnum::MENU_LED:
                    _ledBrightness_str(message, sizeof(message));
                    break;
            #endif
            #if HAVE_CURRENT_LIMIT
                case MenuEnum::MENU_CURRENT:
                    current_limit_str(message, sizeof(message));
                    break;
            #endif
            case MenuEnum::MENU_PWM:
                sprintf_P(message, PSTR("%u%%"), motor.getMaxDutyCycle() * 100 / MAX_DUTY_CYCLE);
                break;
            case MenuEnum::MENU_BRAKE:
                strcpy_P(message, motor.isBrakeEnabled() ? PSTR("ENABLED") : PSTR("DISABLED"));
                break;
            case MenuEnum::MENU_STALL:
                sprintf_P(message, PSTR("%u ms"), motor.getMaxStallTime());
                break;
            #if HAVE_RPM_PER_VOLT
                case MenuEnum::MENU_MOTOR: {
                        auto rpmV = data.getRpmPerVolt();
                        if (rpmV) {
                            sprintf_P(message, PSTR("%u rpm/V"), rpmV);
                            message[10] = 0;
                        }
                        else {
                            strcpy_P(message, PSTR("DISABLED"));
                        }
                    }
                    break;
            #endif
            case MenuEnum::MENU_DISPLAY:
                switch(data.getDisplayMotorStatus()) {
                    case MotorStatusEnum::WATT:
                        strcpy_P(message, PSTR("Watt"));
                        break;
                    case MotorStatusEnum::AMPERE:
                        strcpy_P(message, PSTR("Ampere"));
                        break;
                    default:
                    case MotorStatusEnum::OFF:
                        strcpy_P(message, PSTR("OFF"));
                        break;
                }
                break;
            case MenuEnum::MENU_RESTORE:
                strcpy_P(message, PSTR("Press to restore"));
            default:
                break;
        }
        constexpr uint8_t y = 12;
        uint8_t x = (SCREEN_WIDTH / 2) - (strlen(message) * FONT_WIDTH);
        display.setTextSize(2);
        display.setCursor(x, y);
        display.print(message);
    }
    display.display();
}

bool update_motor_settings(int16_t value)
{
    if (value) {
        switch(data.pidConfig()) {
            case PidConfigEnum::SAVE:
                // reset menu
                data.pidConfig() = PidConfigEnum::OFF;
                write_eeprom();
                return false;
            case PidConfigEnum::OFF:
                data.changeSetPoint(value);
                break;
            case PidConfigEnum::RESTORE:
                pid.resetPidValues();
                pid.reset();
                // jump back to first item
                data.pidConfig() = PidConfigEnum::KP;
                break;
            default:
                pid.updatePidValue(data.pidConfig(), value);
                pid.reset();
                break;
        }
        return true;
    }
    return false;
}
