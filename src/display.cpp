/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "motor.h"
#include "pid_control.h"
#include "current_limit.h"
#include "adc.h"

void setup_display_init()
{
    display.setTextColor(WHITE);
    display.cp437(true);

    display.clearDisplay();
    display.display();
    ui_data.refreshDisplay();
}

// setup and clear display
void setup_display()
{
    // the display needs some time to power up...
    // fade in the LED meanwhile and blink the current limit indicator LED
    auto start = millis16();
    uint16_t time = 0;
    do {
        data.loop();
        if ((time >> 7) % 2 == 0) {
            setCurrentLimitLedOn();
        }
        else {
            setCurrentLimitLedOff();
        }
    }
    while((time = (millis16() - start)) < Timeouts::kBootInitDisplayDelay);

    Wire.begin();
    Wire.setClock(400000);
    if (display.begin(OLED_ADDRESS, OLED_ADDRESS)) {
        setup_display_init();
    }
}

inline void display_rpm()
{
    display.print(ui_data.getRpm());
}

void display_duty_cycle_bar(uint8_t dc)
{
    uint8_t width = (dc * (SCREEN_WIDTH - 4)) / MAX_DUTY_CYCLE;
    display.drawRect(0, SCREEN_HEIGHT - 11, SCREEN_WIDTH, 11, WHITE);
    display.fillRect(2, SCREEN_HEIGHT - 9, width, 7, WHITE);

    if (motor.getMaxDutyCycle() < MAX_DUTY_CYCLE) {
        uint8_t max_pwm_pos = ((motor.getMaxDutyCycle() * (SCREEN_WIDTH - 4)) / MAX_DUTY_CYCLE) + 2;
        if (max_pwm_pos < (SCREEN_WIDTH - 2)) {
            display.drawFastVLine(max_pwm_pos, SCREEN_HEIGHT - 10, 9, INVERSE);
        }
    }
}

void display_set_point_rpm()
{
    char buf[16];
    int len = sprintf_P(buf, PSTR("rpm %u"), data.getSetPointRPM());
    display.setCursor(SCREEN_WIDTH - (FONT_WIDTH * len), 0);
    display.print(buf);
}

void display_set_point_pwm_percent()
{
    char buffer[16];
    PrintBuffer buf(buffer, sizeof(buffer));
    buf.print(data.getSetPointDutyCyclePercent(), 1);
    buf.print('%');
    display.setCursor(SCREEN_WIDTH - (FONT_WIDTH * buf.length()), 0);
    display.print(buffer);
}

void display_motor_status(uint8_t y)
{
    #if ADC_ANALOG_SOURCES
        float value;
        char unit;
        switch(data.getDisplayMotorStatus()) {
            case MotorStatusEnum::AMPERE:
                value = adc.getCurrent_A();
                unit = 'A';
                break;
            case MotorStatusEnum::WATT:
                value = adc.getCurrent_A() * adc.getVoltage_V();
                unit = 'W';
                break;
            default:
                return;
        }
        char buffer[32];
        PrintBuffer buf(buffer, sizeof(buf));
        buf.print(value, 1);
        buf.print(unit);
        display.setCursor(SCREEN_WIDTH - (buf.length() * FONT_WIDTH), y);
        display.print(buffer);
    #endif
}

void display_refresh()
{
    if (ui_data.isTimeoutOnce() && !ui_data.doRefreshDisplay()) {

    }
    else if (!menu.isOpen() && ui_data.doRefreshDisplay())  {
        ui_data.updateTimer();

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (motor.isVelocityMode()) {
            // velocity
            #if 0
            // debug screen
            if (1) {

                display.print(data.getSetPointRPM());
                display.print(' ');
                display.println(ui_data.getDutyCycle());
                display.println(rpm_sense.getLastSignalTicks() / Timer1::kTicksPerMillisecond);
                // auto U = adc.getVoltage_V();
                // auto I = adc.getCurrent_A();
                // display.print(U, 2);
                // display.print(F("V "));
                // display.print(I, 2);
                // display.print(F("A "));
                // display.print(U * I, 2);
                // display.println(F("W "));

                display.setTextSize(2);
                display.print(ui_data.getRpm());

            } else
            #endif
            if (data.pidConfig() == PidConfigEnum::OFF) {
                if (motor.isOn()) {
                    // motor on
                    display_set_point_rpm();
                    display.setTextSize(2);
                    display.setCursor(0, 5);
                    display_rpm();
                    display.setTextSize(1);
                    display_duty_cycle_bar(ui_data.getDutyCycle());
                }
                else {
                    // motor off
                    display.println(F("Set point velocity"));
                    display.print(data.getSetPointRPM());
                    display.println(F(" rpm"));
                }
                display_motor_status(FONT_HEIGHT + 2);
            }
            else {
                // pid sub menu
                pid.displayPidSettingsMenu(data.pidConfig());
            }
        }
        else {
            // PWM
            if (motor.isOn()) {
                // motor on
                display.setTextSize(2);
                display.setCursor(0, 5);
                display_rpm();
                display.setTextSize(1);
                display_set_point_pwm_percent();
                display_duty_cycle_bar(ui_data.getDutyCycle());
            }
            else {
                // motor off
                display.println(F("Set point PWM"));
                display.print(data.getSetPointDutyCyclePercent(), 1);
                display.println('%');
                // display_motor_status(0);
            }
            display_motor_status(FONT_HEIGHT + 2);
        }

        constexpr uint8_t y = (SCREEN_HEIGHT - (FONT_HEIGHT * 2)) + 2;
        display.setTextSize(2);
        if (motor.getState() == MotorStateEnum::ERROR) {
            display.setCursor((SCREEN_WIDTH - (5 * FONT_WIDTH * 2)) / 2, y);
            display.print(F("ERROR"));
        }
        else if (motor.getState() == MotorStateEnum::STALLED) {
            display.setCursor((SCREEN_WIDTH - (7 * FONT_WIDTH * 2)) / 2, y);
            display.print(F("STALLED"));
        }
        else if (motor.getState() == MotorStateEnum::OFF) {
            display.setCursor((SCREEN_WIDTH - (3 * FONT_WIDTH * 2)) / 2, y);
            display.print(F("OFF"));
        }

        display.display();
    }
}

void display_message(const char *message, uint16_t time, uint8_t size, size_t len)
{
    if (len == ~0U) {
        len = strlen(message);
    }
    const uint8_t y = (SCREEN_HEIGHT / 2) - (size * FONT_HEIGHT) + 3;
    const uint8_t x = (SCREEN_WIDTH / 2) - (len * size * (FONT_WIDTH / 2));
    display.clearDisplay();
    display.setCursor(x, y);
    display.setTextSize(size);
    display.print(message);
    display.display();
    ui_data.setRefreshTimeoutOnce(time);
}

void display_message(const __FlashStringHelper *message, uint16_t time, uint8_t size)
{
    const size_t len = strlen_P(reinterpret_cast<PGM_P>(message));
    const size_t strSize = len + 1;
    char buf[strSize];
    memcpy_P(buf, message, strSize);
    display_message(buf, time, size, len);
}
