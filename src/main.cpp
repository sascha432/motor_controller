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

#if HAVE_CURRENT_LIMIT
#    define CURRENT_LIMIT_MENU(x) x
#else
#    define CURRENT_LIMIT_MENU(x)
#endif

#if HAVE_RPM_PER_VOLT
#    define RPM_PER_VOLT_MENU(x) x
#else
#    define RPM_PER_VOLT_MENU(x)
#endif

#if HAVE_LED
#    define LED_MENU(x) x
#else
#    define LED_MENU(x)
#endif

#define MENU_ITEMS_STRING \
    "Speed\0"             \
    "Mode\0"              \
    LED_MENU("LED Brightness\0" ) \
    CURRENT_LIMIT_MENU("Current Limit\0") \
    "Max PWM\0"           \
    "Stall Time\0"        \
    "Brake\0"             \
    RPM_PER_VOLT_MENU("Motor RPM/Volt\0") \
    "Info\0"              \
    "Restore Defaults\0"  \
    "Exit & Save\0"       \
    "\0"

static const char menuItemsString[] PROGMEM = { MENU_ITEMS_STRING };

Adafruit_SSD1306 display(OLED_ADDRESS, OLED_RESET_PIN);
Menu menu;

void UIConfigData::loop()
{
    auto millis = millis16();
    if (millis - _updateUITimer >= Timeouts::Display::kRefresh) {
        _updateUITimer = millis;
        updateRpmAndDutyCycle();
    }
}

void UIConfigData::updateRpmAndDutyCycle()
{
    auto events = rpm_sense.getEvents();
    if (events._count) {
        _displayRpm = (static_cast<uint32_t>(_displayRpm) + events._ticks) / (events._count + 1);
    }
    _dutyCycleAvg = ((static_cast<uint16_t>(_dutyCycleAvg) << 2) + getMotorPWM_timer()) / 5;
}

void read_eeprom()
{
    EEPROMData eeprom_data;
    EEPROM.begin();
    EEPROM.get(0, eeprom_data);
    if (eeprom_data.magic == EEPROM_MAGIC) {
        data = eeprom_data;
        pid.setPidValues(eeprom_data.pid_settings);
    }
    EEPROM.end();
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

bool setup_display_begin()
{

    if (display.begin(OLED_ADDRESS, OLED_ADDRESS)) {
        return true;
    }
    // Wire.setClock(100000);
    // if (display.begin(OLED_ADDRESS, OLED_ADDRESS)) {
    //     return true;
    // }
    return false;
}

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
    while((time = (millis16() - start)) < Timeouts::kBootDelay);

    if (setup_display_begin()) {
        setup_display_init();
    }
    else {
        delay(500);
        restart_device();
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

void refresh_display()
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

#if HAVE_LED

static void _ledBrightness_str(char *message, uint8_t size)
{
    if (data.getLedBrightness() < LED_MIN_PWM) {
        strcpy_P(message, PSTR("OFF"));
    }
    else {
        PrintBuffer buf(message, size);
        buf.print(data.getLedBrightessPercent(), 1);
        buf.print('%');
    }
}

#endif

#if HAVE_CURRENT_LIMIT

static void current_limit_str(char *message, uint8_t size)
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
                            strcpy_P(message, _T(DISABLED));
                        }
                    }
                    break;
            #endif
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

void rotary_button_released(Button& btn, uint16_t duration)
{
    bool longPress = duration > Timeouts::UI::kLongPress;
    if (motor.isOn() || (longPress && !menu.isOpen())) {
        // motor running
        if (longPress) { // long press?
            data.toggleDisplayMotorStatus();
            ui_data.refreshDisplay();
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
                if (!update_motor_settings(value)) {
                    return;
                }
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
        if (update_motor_settings(value)) {
            ui_data.refreshDisplay();
        }
    }
}

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
    Wire.setClock(400000);
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

void readStringUntil(char *begin, char *end, uint16_t timeout = 1000)
{
    char ch = 0;
    uint32_t start = millis();
    while((begin < end) && ((millis() - start) < timeout) && (ch != '\n')) {
        if (Serial.available()) {
            ch = Serial.read();
            *begin++ = ch;
        }
    }
    *begin = 0;
}

void update_pid_cfg()
{
    // w1.1,0.5,0.05,0.015
    // w2,0,0,0.015
    static const char *sep =  ",\r\n";
    char buf[64];
    readStringUntil(buf, buf + sizeof(buf) - 1);
    char *ptr = strtok(buf, sep);
    auto outPtr = pid._settings.begin();
    while(ptr && outPtr < pid._settings.end()) {
        *outPtr++ = atof(ptr);
        ptr = strtok(nullptr, sep);
    }
    pid.reset();
    pid.printValues(Serial);
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

void serial_commands() {
#if HAVE_SERIAL_COMMANDS
    if (Serial.available()) {
        char ch = Serial.read();
        Serial.print(ch);
        switch(ch) {
            #if 0
                case 'R':
                    restart_device();
                    break;
            #endif
            #if 0
                case 'i':
                    menu.open();
                    menu.setPosition(MenuEnum::MENU_INFO);
                    menu.enter();
                    break;
            #endif
            #if 0
                #if HAVE_LED_POWER
                    case 'l': {
                            if (data._ledBrightness < LED_MIN_PWM) {
                                data._ledBrightness = LED_MIN_PWM;
                            }
                            else if (data._ledBrightness == LED_MAX_PWM) {
                                data._ledBrightness = LED_MIN_PWM - 1;
                            }
                            else {
                                data._ledBrightness = std::min(data._ledBrightness + ((data._ledBrightness >= 230) ? 1 : (data._ledBrightness >= 200) ? 4 : 16), LED_MAX_PWM);
                            }
                            Serial.printf_P(PSTR("led %u pwm %umW\n"), data._ledBrightness, LED_POWER_mW(data._ledBrightness));
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
                    motor.setMode(ControlModeEnum::PWM);
                    motor.setDutyCycle(32);
                    motor.start();
                    auto avg = 64;
                    for(uint8_t i = 20; i < 200; i += 10) {
                        if (!motor.isOn()) {
                            break;
                        }
                        motor.setSpeed(i);
                        data.rpm_sense_average = 64;
                        delay(2500);
                        uint16_t rpm = RPM_SENSE_TICKS_TO_RPM(ui_data._displayRpm);
                        auto U = adc.getVoltage_V();
                        Serial.printf_P(PSTR("%u %u %u "), i, U, rpm);
                        auto effU = ((U * i) / 255) - 0.68/*switching and cable losses*/;
                        auto rpmPerVolt = rpm / effU;
                        minRpm = std::min<float>(rpmPerVolt, minRpm);
                        maxRpm = std::max<float>(rpmPerVolt, maxRpm);
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
            #if HAVE_VOLTAGE_DETECTION && HAVE_CURRENT_DETECTION
                case 'A': {
                        Serial.print(F("ADC "));
                        Serial_flush_input();
                        float sum = 0;
                        uint16_t count = 0;
                        for(uint8_t i = 0; i < 100; i++) {
                            float U = adc.getVoltage_V();
                            sum += U;
                            count++;
                            Serial.print(U, 3);
                            Serial.print(' ');
                            Serial.print(sum / count, 3);
                            Serial.print(' ');
                            Serial.print(adc.getVoltageAvg_V(), 3);
                            Serial.print(' ');
                            Serial.print(adc.getCurrent_A(), 3);
                            Serial.print(' ');
                            Serial.print(adc.getPower_W(), 3);
                            Serial.print(' ');
                            Serial.print(adc.getADCAvg(0));
                            Serial.print(' ');
                            Serial.print(adc.getADCAvg(1));
                            Serial.println();
                            delay(250);
                            if (Serial.available()) {
                                Serial_flush_input();
                                break;
                            }
                        }
                        Serial.println();
                    } break;
            #endif
            #endif
            #if 1
                case 'e':
                    pid.printValues(Serial);
                    break;
                case 'w':
                    update_pid_cfg();
                    write_eeprom();
                    break;
                case 'S':
                    start_stop_button_pressed(*(Button *)(nullptr));
                    break;
                #endif
            #if 0
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
                        rpm_sense.setLastSignalMillis(millis() + 30000);
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
                    if (motor.isOff()) {
                        motor.setMode(ControlModeEnum::PWM);
                        Serial.println(F("PWM"));
                    }
                    break;
                case 'r':
                    if (motor.isOff()) {
                        motor.setMode(ControlModeEnum::PID);
                        Serial.println(F("PID"));
                    }
                    break;
            #endif
            #if HAVE_SERIAL_MOTOR_CONTROL
                case '+':
                case '*':
                    data.changeSetPoint(ch == '*' ? 1 : 10);
                    motor.dump(Serial);
                    break;
                case '-':
                case '/': {
                    data.changeSetPoint(ch == '/' ? -1 : -10);
                    motor.dump(Serial);
                }
                break;
            #endif
        }
    }
#endif
}

void loop() {

    serial_commands();
    process_ui_events();
    data.loop();
    motor.loop();
    menu.loop();
    ui_data.loop();
    refresh_display();

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
