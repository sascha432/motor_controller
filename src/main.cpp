/**
 * Author: sascha_lammers@gmx.de
 */

// DC motor controller with RPM feedback

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include "main.h"
#include "rpm_sensing.h"
#include "interrupts.h"
#include "pid_control.h"
#include "motor.h"
#include "current_limit.h"
#include "timer.h"
#include "helpers.h"
#include "adc.h"
#include "serial_commands.h"

#if HAVE_COMPILED_ON_DATE
    const char __compile_date__[] PROGMEM = { __DATE__ " " __TIME__ };
#endif

ConfigData data;
UIConfigData ui_data;
Encoder knob(PIN_ROTARY_ENC_CLK, PIN_ROTARY_ENC_DT);
InterruptPushButton<PIN_BUTTON1_PORT, SFR::Pin<PIN_BUTTON1>::PINmask()> button1(PIN_BUTTON1, PRESSED_WHEN_LOW | ENABLE_INTERNAL_PULLUP);
InterruptPushButton<PIN_BUTTON2_PORT, SFR::Pin<PIN_BUTTON2>::PINmask()> button2(PIN_BUTTON2, PRESSED_WHEN_LOW | ENABLE_INTERNAL_PULLUP);

#if (ADAFRUIT_SSD1306_FIXED_SIZE == 0) || (ADAFRUIT_SSD1306_FIXED_WIDTH != SCREEN_WIDTH || ADAFRUIT_SSD1306_FIXED_HEIGHT != SCREEN_HEIGHT)
#    error invalid settings
#endif

static const char menuItemsString[] PROGMEM = { MENU_ITEMS_STRING };

Adafruit_SSD1306 display(OLED_ADDRESS, OLED_RESET_PIN);
Menu menu;

void read_eeprom()
{
    EEPROMData eeprom_data;
    EEPROM.get(0, eeprom_data);
    if (eeprom_data.magic == EEPROM_MAGIC) {
        data = eeprom_data;
        pid.setPidValues(eeprom_data.pid_settings);
    }
}

void write_eeprom(const __FlashStringHelper *message)
{
    EEPROMData eeprom_data, eeprom_data_current;

    if (message == nullptr) {
        message = F("SAVED");
    }

    eeprom_data.magic = EEPROM_MAGIC;
    eeprom_data = data;
    eeprom_data.pid_settings = pid.getPidValues();

    EEPROM.get(0, eeprom_data_current);
    if (eeprom_data != eeprom_data_current) {
        EEPROM.put(0, eeprom_data);
        display_message(message, Timeouts::Menu::kSaved);
    }
    else {
        menu.close();
    }
}

void set_version(char *buffer)
{
    sprintf_P(buffer, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

#if HAVE_LED

void _ledBrightness_str(char *message, uint8_t size)
{
    if (data.getLedBrightness() < LED_MIN_PWM) {
        strcpy_P(message, PSTR("OFF"));
    }
    else {
        PrintBuffer buf(message, size);
        buf.print(data.getLedBrightessPercent(), 1);
        buf.println('%');
    }
}

#endif

#if HAVE_CURRENT_LIMIT

void current_limit_str(char *message, uint8_t size)
{
    if (current_limit.isDisabled()) {
        strcpy_P(message, PSTR("DISABLED"));
    }
    else {
        PrintBuffer buf(message, size);
        buf.print(current_limit.getLimitAmps(), 1);
        buf.print('A');
    }
}

#endif

void setup()
{
    motor.begin();
    setupLedPwm();

    Serial.begin(115200);

    current_limit.begin();

    #if HAVE_VOLTAGE_DETECTION
        asm volatile ("cbi %0, %1" :: "I" (SFR::Pin<PIN_VOLTAGE>::DDR_IO_ADDR()), "I" (SFR::Pin<PIN_VOLTAGE>::PINbit()));
    #endif
    #if HAVE_CURRENT_DETECTION
        asm volatile ("cbi %0, %1" :: "I" (SFR::Pin<PIN_CURRENT>::DDR_IO_ADDR()), "I" (SFR::Pin<PIN_CURRENT>::PINbit()));
    #endif
    #if HAVE_VOLTAGE_DETECTION || HAVE_CURRENT_DETECTION
        adc.begin();
    #endif

    button1.onPress(start_stop_button_pressed);
    button2.onRelease(rotary_button_released);

    read_eeprom();
    setup_display();
    menu.add(menuItemsString);

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        TCCR1A = 0;
        TCCR1B = TIMER1_PRESCALER_BV;
        TCCR2B = (TCCR2B & 0b11111000) | TIMER2_PRESCALER_BV;
        rpm_sense.begin();
        setup_interrupts();
    }

    char message[32];
    set_version(message);
    Serial.println(message);
    #if HAVE_COMPILED_ON_DATE
        Serial.print(F("Compiled on "));
        Serial.println(FPSTR(__compile_date__));
    #endif
    display_message(message, Timeouts::kVersion, 1);

    knob.setAcceleration(KNOB_ACCELERATION);
}

void process_ui_events()
{
    PinChangesEnum tmpFlag;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        tmpFlag = pinChangedFlag;
        pinChangedFlag = PinChangedType::NONE;
    }
    if (ui_data.isTimeoutOnce()) {
        // ignore events during delay
        return;
    }

    if (tmpFlag & PinChangedType::BUTTON1) {
        button1.update();
    }
    if (tmpFlag & PinChangedType::BUTTON2) {
        button2.update();
    }
    if (tmpFlag & PinChangedType::KNOB) {
        read_rotary_encoder();
    }
}


void loop() {

    serial_commands();
    process_ui_events();
    data.loop();
    motor.loop();
    menu.loop();
    ui_data.loop();
    display_refresh();

    #if 0

    static uint16_t debugIimer;
    if (micros16() - debugIimer >= 5000) {
        debugIimer = micros16();

        static int16_t dc = -1;
        if (motor.isOn()) {
            if (dc == -1) {
                dc = 0;
                pid.printValues(Serial);
                Serial.print(F("start "));
                Serial.println(data.getSetPointRPM());
            }
            uint32_t t = micros() - motor._startTime;
            if (dc != pid.getDutyCycle()) {
                if (Serial.availableForWrite() > 24) {
                    dc = pid.getDutyCycle();
                    Serial.print(t);
                    Serial.print(',');
                    Serial.print(ui_data.getRpm());
                    Serial.print(',');
                    Serial.print(dc);
                    Serial.print('\n');
                }
            }
            else if (ui_data.getRpm() == data.getSetPointRPM() || t > 2 * 1000000) {
                // Serial.println(F("end"));
                // motor.stop(MotorStateEnum::OFF);
            }
        }
        else {
            dc = -1;
        }
    }
    #endif
}
