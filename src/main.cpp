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
#include "DebugBuffer.h"
#include "helpers.h"
#include "adc.h"

#if HAVE_COMPILED_ON_DATE
    const char __compile_date__[] PROGMEM = { __DATE__ " " __TIME__ };
#endif

ConfigData data;
UIConfigData ui_data;
Encoder knob(PIN_ROTARY_ENC_CLK, PIN_ROTARY_ENC_DT);
InterruptPushButton<PIN_BUTTON1_PORT, _BV(PIN_BUTTON1_BIT)> button1(PIN_BUTTON1, PRESSED_WHEN_LOW | ENABLE_INTERNAL_PULLUP);
InterruptPushButton<PIN_BUTTON2_PORT, _BV(PIN_BUTTON2_BIT)> button2(PIN_BUTTON2, PRESSED_WHEN_LOW | ENABLE_INTERNAL_PULLUP);

#if (ADAFRUIT_SSD1306_FIXED_SIZE == 0) || (ADAFRUIT_SSD1306_FIXED_WIDTH != SCREEN_WIDTH || ADAFRUIT_SSD1306_FIXED_HEIGHT != SCREEN_HEIGHT)
#    error invalid settings
#endif

#if HAVE_CURRENT_LIMIT
#    define CURRENT_LIMIT_MENU(x) x
#else
#    define CURRENT_LIMIT_MENU()
#endif

#define MENU_ITEMS_STRING \
    "Speed\0"             \
    "Mode\0"              \
    "LED Brightness\0"    \
    CURRENT_LIMIT_MENU("Current Limit\0") \
    "Max PWM\0"           \
    "Stall Time\0"        \
    "Brake\0"             \
    "Motor RPM/Volt\0"    \
    "Info\0"              \
    "Restore Defaults\0"  \
    "Exit & Save\0"       \
    "\0"

static const char menuItemsString[] PROGMEM = { MENU_ITEMS_STRING };

Adafruit_SSD1306 display(OLED_ADDRESS, OLED_RESET_PIN);
Menu menu;

void read_eeprom()
{
    EEPROMData eeprom_data;
    EEPROM.begin();
    EEPROM.get(0, eeprom_data);
    if (eeprom_data.magic == EEPROM_MAGIC) {
        data = eeprom_data;
        pid.setPidValues(eeprom_data.Kp, eeprom_data.Ki, eeprom_data.Kd);
    }
    EEPROM.end();
}

void write_eeprom(const __FlashStringHelper *message)
{
    EEPROMData eeprom_data, eeprom_data_current;

    eeprom_data.magic = EEPROM_MAGIC;
    eeprom_data = data;
    pid.getPidValues(eeprom_data.Kp, eeprom_data.Ki, eeprom_data.Kd);

    EEPROM.get(0, eeprom_data_current);
    if (eeprom_data != eeprom_data_current) {
        EEPROM.put(0, eeprom_data);
        display_message(message, DISPLAY_SAVED_TIMEOUT);
    }
    else {
        ui_data.refreshDisplay();
    }
}

void set_version(char *buffer)
{
    sprintf_P(buffer, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

// setup and clear display
void setup_display()
{
    constexpr uint16_t displayBootTimeDelay = 750;

    // the display needs some time to power up...
    // fade in the LED meanwhile and blink the current limit indicator LED
    #if HAVE_LED_FADING
        auto start = millis();
        uint16_t time;
        do {
            time = millis() - start;
            data.setLedBrightness();
            if ((time / (displayBootTimeDelay / 10)) % 2) {
                setCurrentLimitLedOn();
            }
            else {
                setCurrentLimitLedOff();
            }
        } while(time < displayBootTimeDelay);
    #else
        for(uint8_t i = 0; i < 4; i++) {
            digitalWrite(PIN_CURRENT_LIMIT_LED_PINNO, i % 2);
            delay(displayBootTimeDelay / 10);
        }
    #endif

    if (display.begin(OLED_ADDRESS, OLED_ADDRESS)) {
        Wire.setClock(400000);
        display.setTextColor(WHITE);
        display.cp437(true);

        display.clearDisplay();
        display.display();
        ui_data.refreshDisplay();
    }
}

void display_rpm()
{
    display.print(RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral));
    display.println(_F(_rpm));
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

void display_current_limit()
{
    display.fillCircle(SCREEN_WIDTH - FONT_HEIGHT * 1 - 3, FONT_HEIGHT + 1, FONT_HEIGHT + 1, WHITE);
    display.setTextSize(2);
    display.setTextColor(INVERSE);
    display.setCursor(SCREEN_WIDTH - FONT_HEIGHT * 2, 2);
    display.print('C');
    display.setTextColor(WHITE);
}

void display_set_point_and_voltage()
{
    char buf[8];
    int len = sprintf_P(buf, PSTR("%u"), data.getSetPointRPM());
    display.setCursor(SCREEN_WIDTH - (FONT_WIDTH * len), 0);
    display.print(buf);
    #if HAVE_VOLTAGE_DETECTION
        auto U = adc.getVoltage_V();
        display.setCursor(SCREEN_WIDTH - ((U >= 10) ? 5 * FONT_WIDTH : FONT_WIDTH * 4), FONT_HEIGHT);
        display.println(U, 2);
    #endif
}

void refresh_display()
{
    if (!menu.isOpen() && (millis() > ui_data.refresh_timer))  {
        ui_data.refresh_timer = millis() + DISPLAY_REFRESH_TIME;
        ui_data.refresh_counter++;

        #if DEBUG_RPM_SIGNAL
            if (millis() - rpm_sense.getLastSignalMillis() < 1000) {
                Serial.print(rpm_sense.getTimerIntegralTicksFloat() / TIMER1_TICKS_PER_US, 2);
                Serial.print(' ');
                Serial.print(RPM_SENSE_TICKS_TO_HZ(rpm_sense.getTimerIntegralTicksFloat()), 2);
                Serial.print(' ');
                Serial.print(ui_data.display_pulse_length_integral);
                Serial.print(' ');
                Serial.print(RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral));
                Serial.print(' ');
                Serial.print(RPM_SENSE_US_TO_HZ(ui_data.display_pulse_length_integral * 100.0) * (1 / 100.0), 2);
                Serial.print(' ');
                Serial.println(ui_data.display_duty_cycle_integral);
            }
        #endif

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (motor.isVelocityMode()) {
            if (motor.isOn()) {
                if (data.pid_config == PidConfigEnum::OFF) {
                    display_set_point_and_voltage();
                    display.setTextSize(2);
                    display.setCursor(0, 5);
                }
                display_rpm();
                if (data.pid_config != PidConfigEnum::OFF) {
                    // char buffer[64];
                    pid.printValues(display);
                    // display.print(buffer);
                    //display.println(buffer);
                    // display.print('<');
                    switch(data.pid_config) {
                        case PidConfigEnum::KP:
                            display.print(F("<Kp>"));
                            break;
                        case PidConfigEnum::KI:
                            display.print(F("<Ki>"));
                            break;
                        case PidConfigEnum::KD:
                            display.print(F("<Kp>"));
                            break;
                        case PidConfigEnum::OMUL:
                            display.print(F("<out-mul>"));
                            break;
                        case PidConfigEnum::DTMUL:
                            display.print(F("<dt-mul>"));
                            break;
                        case PidConfigEnum::SAVE:
                            display.print(F("<Save>"));
                            break;
                        case PidConfigEnum::RESTORE:
                            display.print(F("<Restore>"));
                            break;
                        default:
                            break;
                    }
                }
                else {
                    display_duty_cycle_bar(ui_data.display_duty_cycle_integral);
                }
            }
            else {
                display.setTextSize(1);
                #if 0
                    display.printf_P(PSTR("Set point velocity\n%u rpm\n"), data.getSetPointRPM());
                #else
                    display.println(F("Set point velocity"));
                    display.print(data.getSetPointRPM());
                    display.println(_F(_rpm));
                #endif
            }
        }
        else {
            if (motor.isOn()) {
                display.setTextSize(2);
                display.setCursor(0, 5);
                display_rpm();

                display.setTextSize(1);
                char buffer[16];
                PrintBuffer buf(buffer, sizeof(buf));
                buf.print(adc.getCurrent_A(), 2);
                buf.print('A');
                display.setCursor(SCREEN_WIDTH - (strlen(buffer) * FONT_WIDTH), 0);
                display.print(buffer);

                display_duty_cycle_bar(pid.duty_cycle);
            }
            else {
                display.setTextSize(1);
                #if HAVE_PRINTF_FLT
                    display.printf_P(PSTR("Set point PWM %.1f%%\n"), data.getSetPointDutyCycle() * 100 / (float)MAX_DUTY_CYCLE);
                #else
                    display.println(F("Set point PWM"));
                    display.print(data.getSetPointDutyCycle() * 100 / (float)MAX_DUTY_CYCLE, 1);
                    display.println('%');
                #endif
            }
        }

        constexpr uint8_t y = (SCREEN_HEIGHT - (FONT_HEIGHT * 2)) + 2;
        display.setTextSize(2);
        if (motor.getState() == MotorStateEnum::ERROR) {
            display.setCursor((SCREEN_WIDTH - (5 * FONT_WIDTH * 2)) / 2, y);
            display.print(_F(ERROR));
        }
        else if (motor.getState() == MotorStateEnum::STALLED) {
            display.setCursor((SCREEN_WIDTH - (7 * FONT_WIDTH * 2)) / 2, y);
            display.print(_F(STALLED));
        }
        else if (motor.getState() == MotorStateEnum::OFF) {
            display.setCursor((SCREEN_WIDTH - (3 * FONT_WIDTH * 2)) / 2, y);
            display.print(_F(OFF));
        }

        #if HAVE_CURRENT_LIMIT
            if (ui_data.display_current_limit_timer) {
                // display for 2 seconds
                if (millis() - ui_data.display_current_limit_timer > 2000) {
                    ATOMIC_BLOCK(ATOMIC_FORCEON) {
                        ui_data.display_current_limit_timer = 0;
                    }
                }
                else {
                    display_current_limit();
                }
            }
        #endif

        display.display();
    }
}

static void led_brightness_str(char *message, uint8_t size)
{
    if (data.led_brightness < LED_MIN_PWM) {
        strcpy_P(message, _T(OFF));
    }
    else {
        #if HAVE_PRINTF_FLT
            sprintf_P(message, PSTR("%.1f%%"), (data.led_brightness * 100) / static_cast<float>(LED_MAX_PWM));
        #else
            PrintBuffer buf(message, size);
            buf.print((data.led_brightness * 100) / static_cast<float>(LED_MAX_PWM), 1);
            buf.print('%');
            // snprintf_P(message, size, PSTR("%u%%"), (data.led_brightness * 100) / LED_MAX_PWM);
        #endif
    }
}

#if HAVE_CURRENT_LIMIT

static void current_limit_str(char *message, uint8_t size)
{
    if (current_limit.isDisabled()) {
        strcpy_P(message, _T(DISABLED));
    }
    else {
        #if HAVE_PRINTF_FLT
            sprintf_P(message, PSTR("%.1fA"), CURRENT_LIMIT_DAC_TO_CURRENT(current_limit.getLimit()));
        #else
            PrintBuffer buf(message, size);
            buf.print(CURRENT_LIMIT_DAC_TO_CURRENT(current_limit.getLimit()), 1);
            buf.print('A');
        #endif
    }
}

#endif

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

        pid.printValues(display);
        // display.println(message);

        #if HAVE_CURRENT_LIMIT
            if (!current_limit.isDisabled()) {
                current_limit_str(message, sizeof(message));
                display.printf_P(PSTR("Limit %s, "), message);
            }
        #endif
        led_brightness_str(message, sizeof(message));
        #if HAVE_LED_POWER
            display.printf_P(PSTR("LED %s %umW\n"), message, LED_POWER_mW(data.led_brightness));
        #else
            display.printf_P(PSTR("LED %s\n"), message);
        #endif

        #if HAVE_VOLTAGE_DETECTION && !HAVE_CURRENT_DETECTION
            #if HAVE_PRINTF_FLT
                display.printf_P(PSTR("Input %.2fV\n"), adc.getVoltage_V());
            #else
                display.print(F("Input "));
                display.print(adc.getVoltage_V(), 2);
                display.println('V');
            #endif
        #elif HAVE_CURRENT_DETECTION && HAVE_VOLTAGE_DETECTION
            #if HAVE_PRINTF_FLT
                display.printf_P(PSTR("Input %.2fV %.3fA\n"), adc.getVoltage_V(), adc.getCurrent_A());
            #else
                display.print(F("Input "));
                display.print(adc.getVoltage_V(), 2);
                display.print(F("V "));
                display.print(adc.getCurrent_A(), 3);
                display.println('A');
            #endif
        #else
            #if HAVE_PRINTF_FLT
                display.printf_P(PSTR("Input %.2fV\n"), adc.getVoltage_V());
            #else
                display.print(F("Input "));
                display.print(adc.getVoltage_V(), 2);
                display.println('V');
            #endif
        #endif

        // #if HAVE_CURRENT_DETECTION && HAVE_VOLTAGE_DETECTION
        //     display.print(F("V "));
        //     display.print(getCurrent() / 1000.0, 3);
        //     display.println('A');
        // #else
        //     display.println('V');
        // #endif

        ui_data.refresh_timer = millis() + 500;
    }
    else {
        // display menu
        menu.displayTitle();
        switch(menu.getPosition()) {
            case MenuEnum::MENU_SPEED:
                if (motor.isDutyCycleMode()) {
                    #if HAVE_PRINTF_FLT
                        sprintf_P(message, PSTR("%.1f%%"), (pid.duty_cycle * 100) / static_cast<float>(MAX_DUTY_CYCLE));
                    #else
                        PrintBuffer buf(message, sizeof(message));
                        buf.print(pid.duty_cycle * 100 / (float)MAX_DUTY_CYCLE, 1);
                        buf.print('%');
                    #endif
                }
                else {
                    sprintf_P(message, PSTR("%u rpm"), data.getSetPointRPM());
                }
                break;
            case MenuEnum::MENU_MODE:
                strcpy_P(message, motor.isVelocityMode() ? PSTR("Velocity") : PSTR("PWM"));
                break;
            case MenuEnum::MENU_LED:
                led_brightness_str(message, sizeof(message));
                break;
            #if HAVE_CURRENT_LIMIT
                case MenuEnum::MENU_CURRENT:
                    current_limit_str(message, sizeof(message));
                    break;
            #endif
            case MenuEnum::MENU_PWM:
                sprintf_P(message, PSTR("%u%%"), motor.getMaxDutyCycle() * 100 / MAX_DUTY_CYCLE);
                break;
            case MenuEnum::MENU_BRAKE:
                strcpy_P(message, motor.isBrakeEnabled() ? _T(ENABLED) : _T(DISABLED));
                break;
            case MenuEnum::MENU_STALL:
                sprintf_P(message, PSTR("%u ms"), motor.getMaxStallTime());
                break;
            case MenuEnum::MENU_MOTOR: {
                    auto rpmV = data.getRpmPerVolt();
                    if (rpmV) {
                        sprintf_P(message, PSTR("%u rpm/V"), rpmV);
                        message[10] = 0;
                    }
                    else {
                        strcpy_P(message, _T(DISABLED));
                    }
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
        if (data.pid_config == PidConfigEnum::OFF) {
            data.changeSetPoint(value);
            motor.updateMotorSpeed();
            return true;
        }
        else {
            if (data.pid_config == PidConfigEnum::SAVE) {
                write_eeprom();
                data.pid_config = PidConfigEnum::OFF;
                return false;
            }
            else {
                pid.resetPidValues();
                pid.reset();
            }
        }
        if (data.pid_config != PidConfigEnum::OFF) {
            pid.updatePidValue(static_cast<uint8_t>(data.pid_config) - 1, value);
            pid.reset();
        }
        return true;
    }
    return false;
}

void rotary_button_released(Button& btn, uint16_t duration)
{
    if (motor.isOn()) {
        // motor running
        if (motor.isVelocityMode()) {
            data.pid_config = static_cast<PidConfigEnum>((static_cast<uint8_t>(data.pid_config) + 1) % static_cast<uint8_t>(PidConfigEnum::MAX));
            ui_data.refreshDisplay();
        }
    }
    else if (menu.isOpen()) {
        // inside menu
        if (duration > MENU_LONG_PRESS_MILLIS) {
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
                        menu.close();
                        data = ConfigData();
                        pid.resetPidValues();
                        write_eeprom(F("RESTORED"));
                        read_eeprom();
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
        if (motor.getState() != MotorStateEnum::ON) {
            knob.write(0);
            motor.start();
        }
        else if (motor.isOn()) {
            motor.stop(MotorStateEnum::OFF);
            refresh_display();
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
        int16_t new_value;
        switch(menu.getPosition()) {
            case MenuEnum::MENU_SPEED:
                if (!update_motor_settings(value)) {
                    return;
                }
                break;
            case MenuEnum::MENU_LED:
                new_value = data.led_brightness + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED);
                data.led_brightness = std::clamp<int16_t>(new_value, LED_MIN_PWM - 1, LED_MAX_PWM);
                data.setLedBrightnessNoDelay();
                break;
            #if HAVE_CURRENT_LIMIT
                case MenuEnum::MENU_CURRENT:
                    new_value = current_limit.getLimit() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED);
                    if (new_value >= CURRENT_LIMIT_MAX && value > 0) {
                        current_limit.setLimit(CURRENT_LIMIT_DISABLED);
                    }
                    else {
                        current_limit.setLimit(std::clamp<int16_t>(new_value, CURRENT_LIMIT_MIN, CURRENT_LIMIT_MAX));
                    }
                    current_limit.updateLimit();
                    break;
            #endif
            case MenuEnum::MENU_PWM:
                motor.setMaxDutyCycle(std::clamp<int16_t>(motor.getMaxDutyCycle() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE));
                break;
            case MenuEnum::MENU_STALL:
                motor.setMaxStallTime(std::clamp<int16_t>(motor.getMaxStallTime() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), STALL_TIME_MIN, STALL_TIME_MAX));
                break;
            case MenuEnum::MENU_MOTOR:
                data.setRpmPerVolt(std::clamp<int16_t>(data.getRpmPerVolt() + KNOB_GET_VALUE(value, KNOB_VALUE_SPEED), 0, 32500));
                break;
            case MenuEnum::MENU_MODE:
                motor.toggleMode();
                break;
            case MenuEnum::MENU_BRAKE:
                motor.enableBrake(!motor.isBrakeEnabled());
                break;
            default:
                break;
        }
        menu.resetTimer();
        switch(menu.getPosition()) {
            case MenuEnum::MENU_MODE:
            case MenuEnum::MENU_BRAKE:
                if (value != 0) {
                    knob.write(0);
                }
                break;
            default:
                break;
        }
    }
    else {
        // main screen
        if (update_motor_settings(value)) {
            ui_data.refreshDisplay();
        }
    }
}

void setup()
{
    Serial.begin(115200);

    motor.begin();
    #if HAVE_LED_FADING
        setupLedPwm();
    #endif
    current_limit.begin();

    pinMode(PIN_LED_DIMMER, OUTPUT);

    #if HAVE_VOLTAGE_DETECTION
        pinMode(PIN_VOLTAGE, INPUT);
    #endif
    #if HAVE_CURRENT_DETECTION
        pinMode(PIN_CURRENT, INPUT);
    #endif
    #if HAVE_VOLTAGE_DETECTION || HAVE_CURRENT_DETECTION
        // it takes a while after setup to get the first values
        // these are collected during the display initializtation, which takes 750ms (setup_display())
        adc.setup();
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
    display_message(message, DISPLAY_BOOT_VERSION_TIMEOUT, 1);

    current_limit.enable(true);

    knob.setAcceleration(KNOB_ACCELERATION);
}

void readStringUntil(char *begin, char *end)
{
    char ch;
    while(Serial.available() && (begin < end) && (ch = Serial.read()) != '\n') {
        *begin++ = ch;
    }
    *begin = 0;
}

void update_pid_cfg()
{
    #if 1
    //TODO test code
    // 1060byte
    // w1.0,0.001,0.0001,0.001,0.00000000001
    static const char *sep =  ",\r\n";
    char buf[128];
    readStringUntil(buf, buf + sizeof(buf) - 1);
    char *ptr = strtok(buf, sep);
    auto outPtr = &pid.Kp;
    auto lastPtr = &pid.dtMultiplier;
    while(ptr && outPtr <= lastPtr) {
        *outPtr = atof(ptr);
        ptr = strtok(nullptr, sep);
    }
    #else
    // 1814byte
    // w1.0,0.001,0.0001,0.001,0.00000000001
    static const char *sep =  ",\r\n";
    String str = Serial.readStringUntil('\n');
    char *ptr = strtok(str.begin(), sep);
    if (ptr) {
        pid.Kp = atof(ptr);
        if ((ptr = strtok(nullptr, ",\r\n"))) {
            pid.Ki = atof(ptr);
            if ((ptr = strtok(nullptr, ",\r\n"))) {
                pid.Kd = atof(ptr);
                if ((ptr = strtok(nullptr, ",\r\n"))) {
                    pid.outputMultiplier = atof(ptr);
                    if (ptr) {
                        pid.dtMultiplier = atof(ptr);
                    }
                }
            }
        }
    }
    #endif
    pid.reset();
    pid.printValues(Serial);
    // Serial.print(buf);
    //print_pid_cfg_serial();
}

void process_ui_events()
{
    PinChangesEnum tmpFlag;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        tmpFlag = pinChangedFlag;
        pinChangedFlag = PinChangedType::NONE;
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

    #if DEBUG_INPUTS
        static uint8_t oldValue;
        bool a = digitalRead(PIN_RPM_SIGNAL);
        bool b = digitalRead(PIN_BUTTON1);
        bool c = digitalRead(PIN_BUTTON2);
        uint8_t value = (a << 3) | (b << 2) | c;
        if (oldValue != value) {
            oldValue = value;
            Serial.printf_P(PSTR("gpio: rpm=%u btns=%u/%u\n"), a, b, c);
        }
    #endif

#if HAVE_SERIAL_COMMANDS
    if (Serial.available()) {
        uint8_t ch;
        switch(ch = Serial.read()) {
            #if HAVE_SERIAL_HELP
                case 'h':
                case '?':
                    Serial.print(F(
                        "h, ?                       Display help\n"
                        "i                          Open info menu\n"
                        "b                          Toggle brake\n"
                        #if HAVE_SERIAL_MOTOR_CONTROL
                        "d                          Set motor to DC (duty cycle)\n"
                        "r                          Set motor to PID controlled\n"
                        "+, *                       Increase DC or RPM\n"
                        "-, /                       Decrease DC or RPM\n"
                        #endif
                        "l                          Toggle LED brightness\n"
                        #if HAVE_VOLTAGE_DETECTION
                        "v                          Display voltage\n"
                        #endif
                        #if HAVE_CURRENT_DETECTION
                        "V                          Display current\n"
                        #endif
                    ));
                    break;
            #endif
#if 1
            case 'i':
                menu.open();
                menu.setPosition(MenuEnum::MENU_INFO);
                menu.enter();
                break;
            #if HAVE_LED_POWER
                case 'l': {
                        if (data.led_brightness < LED_MIN_PWM) {
                            data.led_brightness = LED_MIN_PWM;
                        }
                        else if (data.led_brightness == LED_MAX_PWM) {
                            data.led_brightness = LED_MIN_PWM - 1;
                        }
                        else {
                            data.led_brightness = std::min(data.led_brightness + ((data.led_brightness >= 230) ? 1 : (data.led_brightness >= 200) ? 4 : 16), LED_MAX_PWM);
                        }
                        Serial.printf_P(PSTR("led %u pwm %umW\n"), data.led_brightness, LED_POWER_mW(data.led_brightness));
                    }
                    break;
            #endif
#endif
#if 0
            case 'I': {
                uint16_t minRpm = ~0;
                uint16_t maxRpm = 0;
                uint32_t sumRpm = 0;
                uint8_t counter = 0;
                auto mode = motor.getMode();
                motor.setMode(ControlModeEnum::DUTY_CYCLE);
                motor.setDutyCycle(VELOCITY_START_DUTY_CYCLE);
                motor.start();
                auto avg = data.rpm_sense_average;
                for(uint8_t i = 20; i < 200; i += 10) {
                    if (!motor.isOn()) {
                        break;
                    }
                    motor.setSpeed(i);
                    data.rpm_sense_average = 64;
                    delay(2500);
                    uint16_t rpm = RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral);
                    auto U = getVoltage();
                    Serial.printf_P(PSTR("%u %u %u "), i, U, rpm);
                    auto effU = ((U * i) / 255) - 0.68/*switching and cable losses*/;
                    auto rpmPerVolt = rpm / effU;
                    minRpm = std::min(rpmPerVolt, minRpm);
                    maxRpm = std::max(rpmPerVolt, maxRpm);
                    sumRpm += rpmPerVolt;
                    counter++;
                    Serial.println(rpmPerVolt);
                }
                motor.stop();
                motor.setMode(mode);
                data.rpm_sense_average = avg;
                Serial.printf_P(PSTR("rpm/V avg=%u median=%u min=%u max=%u points=%u\n"), (unsigned)(sumRpm / counter), (unsigned)((minRpm + maxRpm) / 2), minRpm, maxRpm, counter);
            } break;
#endif
#if 0
            #if HAVE_VOLTAGE_DETECTION
                case 'v':
                    Serial.print(F("Voltage (V) "));
                    Serial_flush_input();
                    for(uint8_t i = 0; i < 100; i++) {
                        if (i % 10 == 0) {
                            Serial.printf_P(PSTR("%03u: "), i);
                        }
                        Serial.print(adc.getVoltage_V(), 3);
                        Serial.print(i % 10 == 9 ? '\n' : ' ');
                        delay(125);
                        if (Serial.available()) {
                            Serial_flush_input();
                            break;
                        }
                    }
                    Serial.println();
                    break;
            #endif
            #if HAVE_CURRENT_DETECTION
                case 'V':
                    Serial.print(F("Current (A) "));
                    Serial_flush_input();
                    for(uint8_t i = 0; i < 250; i++) {
                        if (i % 10 == 0) {
                            Serial.printf_P(PSTR("%03u: "), i);
                        }
                        Serial.print(adc.getCurrent_A(), 3);
                        Serial.print(i % 10 == 9 ? '\n' : ' ');
                        delay(50);
                        if (Serial.available()) {
                            Serial_flush_input();
                            break;
                        }
                    }
                    Serial.println();
                    break;
            #endif
#endif
#if 0
            case 'a':
                data.rpm_sense_average++;
                Serial.print(F("rpms_avg="));
                Serial.println(data.rpm_sense_average);
                break;
            case 's':
                data.rpm_sense_average--;
                Serial.print(F("rpms_avg="));
                Serial.println(data.rpm_sense_average);
                break;
            case 'e':
                // print_pid_cfg_serial();
                pid.printValues(Serial);
                break;
            case 'w':
                update_pid_cfg();
                write_eeprom();
                break;
            case 'S':
                start_stop_button_pressed(*(Button *)(nullptr));
                motor.dump(Serial);
                break;
#endif
            #if 1 || HAVE_SERIAL_MOTOR_CONTROL
                case 'b':
                    if (isBrakeOn()) {
                        if (motor.getState() == MotorStateEnum::BRAKING) {
                            motor.stop(MotorStateEnum::OFF);
                        }
                        else {
                            motor.setBrake(false);
                        }
                    }
                    else if (motor.isOff()) {
                        cli();
                        rpm_sense._lastSignalMillis = millis() + 30000;
                        sei();
                        if (motor.getState() != MotorStateEnum::BRAKING) {
                            motor.stop(MotorStateEnum::BRAKING);
                        }
                        else {
                            motor.setBrake(true);
                            delay(1000);
                        }
                    }
                    break;
            #endif
            #if HAVE_SERIAL_MOTOR_CONTROL
                case 'd':
                    motor.setMode(ControlModeEnum::DUTY_CYCLE);
                    motor.dump(Serial);
                    break;
                case 'r':
                    motor.setMode(ControlModeEnum::PID);
                    motor.dump(Serial);
                    break;
            #endif
            #if HAVE_SERIAL_MOTOR_CONTROL
                case '+':
                case '*':
                    data.changeSetPoint(POTI_RANGE / (ch == '*' ? 10 : 128));
                    motor.updateMotorSpeed();
                    motor.dump(Serial);
                    break;
                case '-':
                case '/': {
                    data.changeSetPoint(POTI_RANGE / (ch == '/' ? -10 : -128));
                    motor.updateMotorSpeed();
                    motor.dump(Serial);
                }
                break;
            #endif
        }
    }
#endif

    #if CURRENT_LIMIT_LED_IDLE_INDICATOR
        if (motor.getState() == MotorStateEnum::OFF) {
            #if ((FLASH_INTERVAL / FLASH_ON_TIME) > FLASH_COUNTER_MASK)
                #error increase on-time or decrease interval
            #endif
            #if (FLASH_COUNTER_MASK < (FLASH_NUM_TIMES * 4))
                #error the LED must not flash continuously
            #endif
            EVERY_N_MILLIS(FLASH_ON_TIME) {
                // start with wait period
                static uint8_t counter = FLASH_NUM_TIMES * 2;
                #if (FLASH_WAIT_PERIOD / FLASH_ON_TIME) >= FLASH_COUNTER_MASK
                    counter++;
                    #if FLASH_COUNTER_MASK
                        counter &= FLASH_COUNTER_MASK;
                    #endif
                #else
                    if (++counter > (FLASH_WAIT_PERIOD / FLASH_ON_TIME)) {
                        counter = 0;
                    }
                #endif
                if (counter < FLASH_NUM_TIMES * 2) {
                    if (!(counter & 0x01)) {
                        setCurrentLimitLedOn();
                    }
                    else if (counter & 0x01) {
                        setCurrentLimitLedOff();
                    }
                }
            }
        }
    #endif


    // Serial.println(RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral));
    // delay(100);

    process_ui_events();
    data.setLedBrightness();
    motor.loop();
    menu.loop();
    refresh_display();
}
