 /**
 * Author: sascha_lammers@gmx.de
 */

// DC motor controller with RPM feedback

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Adafruit_SSD1306.h>
#include <Button.h>
#include <ButtonEventCallback.h>
#include <PushButton.h>
#include <Bounce2.h>
#include <Encoder.h>
#include "main.h"
#include "rpm_sensing.h"
#include "interrupts.h"
#include "pid_control.h"
#include "motor.h"
#include "current_limit.h"
#include "timer.h"
#include "DebugBuffer.h"
#include "helpers.h"

#if HAVE_COMPILED_ON_DATE
const char __compile_date__[] PROGMEM = { __DATE__ " " __TIME__ };
#endif

Data_t data;
UIData_t ui_data;
Encoder knob(PIN_ROTARY_ENC_CLK, PIN_ROTARY_ENC_DT);
PushButton button1(PIN_BUTTON1, PRESSED_WHEN_LOW|ENABLE_INTERNAL_PULLUP);
PushButton button2(PIN_BUTTON2, PRESSED_WHEN_LOW|ENABLE_INTERNAL_PULLUP);
#if (ADAFRUIT_SSD1306_FIXED_SIZE == 0) || (ADAFRUIT_SSD1306_FIXED_WIDTH != SCREEN_WIDTH || ADAFRUIT_SSD1306_FIXED_HEIGHT != SCREEN_HEIGHT)
#error invalid settings
#endif
Adafruit_SSD1306 display(OLED_ADDRESS, OLED_RESET_PIN);
Menu menu(MenuEnum::MENU_COUNT, display);

void read_eeprom()
{
    EEPROMData_t eeprom_data;
    EEPROM.begin();
    EEPROM.get(0, eeprom_data);
    if (eeprom_data.magic == EEPROM_MAGIC) {
        data.copyFrom(eeprom_data);
        pid.setPidValues(eeprom_data.Kp, eeprom_data.Ki, eeprom_data.Kd);
    }
    // only called once in setup
    // else {
    //     data = Data_t();
    //     pid.resetPidValues();
    // }
    EEPROM.end();
}

void write_eeprom()
{
    EEPROMData_t eeprom_data, eeprom_data_current;

    eeprom_data.magic = EEPROM_MAGIC;
    data.copyTo(eeprom_data);
    pid.getPidValues(eeprom_data.Kp, eeprom_data.Kd, eeprom_data.Ki);

    EEPROM.begin();
    EEPROM.get(0, eeprom_data_current);
    bool changed = memcmp(&eeprom_data, &eeprom_data_current, sizeof(eeprom_data)) != 0;
    if (changed) {
        EEPROM.put(0, eeprom_data);
    }
    EEPROM.end();

    if (changed) {
        display_message(_F(SAVED), DISPLAY_SAVED_TIMEOUT);
    }
    else {
        ui_data.refreshDisplay();
    }
}

void set_version(char *buffer, uint8_t size)
{
    snprintf_P(buffer, size, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}


// setup and clear display
void setup_display() {

    constexpr uint16_t displayBootTimeDelay = 750;

    // the display needs some time to power up...
    // fade in the LED meanwhile and blink the current limit indicator LED
#if HAVE_LED_FADING
    unsigned long start = millis();
    uint16_t time;
    do {
        time = millis() - start;
        data.setLedBrightness();
        digitalWrite(PIN_CURRENT_LIMIT_LED, (time / (displayBootTimeDelay / 10)) % 2);
    } while(time < displayBootTimeDelay);
#else
    for(uint8_t i = 0; i < 4; i++) {
        digitalWrite(PIN_CURRENT_LIMIT_LED, i % 2);
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
    if (ui_data.display_pulse_length_integral) {
        display.print(RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral));
    } else {
        display.print(0);
    }
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
    auto U = getVoltage();
    display.setCursor(SCREEN_WIDTH - ((U >= 10) ? 5 * FONT_WIDTH : FONT_WIDTH * 4), FONT_HEIGHT);
    display.println(U, 2);
}

void refresh_display()
{
    if (millis() > ui_data.refresh_timer)  {
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

        if (menu.isOpen()) {
            if (menu.getPosition() == MenuEnum::MENU_INFO) {
                menu_display_value();
            }
            else {
                menu.close();
                write_eeprom();
            }
            return;
        }

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
                    //char buffer[64];
                    pid.printValues(display);
                    //display.println(buffer);
                    display.print('<');
                    switch(data.pid_config) {
                        case PidConfigEnum::KP:
                            display.print(F("Kp"));
                            break;
                        case PidConfigEnum::KI:
                            display.print(F("Ki"));
                            break;
                        case PidConfigEnum::KD:
                            display.print(F("Kp"));
                            break;
                        case PidConfigEnum::OMUL:
                            display.print(F("out-mul"));
                            break;
                        case PidConfigEnum::DTMUL:
                            display.print(F("dt-mul"));
                            break;
                        case PidConfigEnum::SAVE:
                            display.print(F("Save"));
                            break;
                        case PidConfigEnum::RESTORE:
                            display.print(F("Restore"));
                            break;
                        default:
                            break;
                    }
                    display.print('>');
                }
                else {
                    display_duty_cycle_bar(ui_data.display_duty_cycle_integral);
                }
            }
            else {
                display.setTextSize(1);
                display.println(F("Set point velocity"));
                display.print(data.getSetPointRPM());
                display.println(_F(_rpm));
            }
        }
        else {
            if (motor.isOn()) {
                display.setTextSize(2);
                display.setCursor(0, 5);
                display_rpm();
                display_duty_cycle_bar(pid.duty_cycle);
            }
            else {
                display.setTextSize(1);
                display.println(F("Set point PWM"));
                display.print(data.getSetPointDutyCycle() * 100 / (float)MAX_DUTY_CYCLE, 1);
                display.println('%');
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
        uint32_t time;
        // display for 2 seconds
        if ((time = get_time_diff(ui_data.display_current_limit_timer, millis())) > 2000) {
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
        snprintf_P(message, size, PSTR("%u%%"), data.led_brightness * 100 / 255);
    }
}

#if HAVE_CURRENT_LIMIT

static void current_limit_str(char *message, uint8_t size)
{
    if (current_limit.isDisabled()) {
        strcpy_P(message, _T(DISABLED));
    }
    else {
        PrintBuffer buf(message, size);
        buf.print(CURRENT_LIMIT_DAC_TO_CURRENT(current_limit.getLimit()), 1);
        buf.print('A');
    }
}

#endif

#if HAVE_VOLTAGE_DETECTION

    float getVoltage()
    {
        #define NUM_READS 5
        #define ADC_VALUES 1024
        #if NUM_READS * ADC_VALUES > 0xffff
        #error 16bit overflow
        #endif

        uint16_t value = analogRead(PIN_VOLTAGE);
        for(uint8_t i = 1; i < NUM_READS; i++) {
            value += analogRead(PIN_VOLTAGE);
            delay(1);
        }
        return value * (VOLTAGE_DETECTION_DIVIDER / (ADC_VALUES * NUM_READS / MCU_VOLTAGE));
    }

#endif

void menu_display_value()
{
    char message[16];

    display.clearDisplay();
    if (menu.getPosition() == MenuEnum::MENU_INFO) {
        display.setTextSize(1);
        display.setCursor(0, 0);

        set_version(message, sizeof(message));
        display.println(message);

        pid.printValues(display);
        //display.println(message);

#if HAVE_CURRENT_LIMIT
        if (!current_limit.isDisabled()) {
            display.print(F("Limit "));
            current_limit_str(message, sizeof(message));
            display.print(message);
            display.print(F(", "));
        }
#endif
        display.print(F("LED "));
        led_brightness_str(message, sizeof(message));
        display.println(message);

#if HAVE_VOLTAGE_DETECTION

        display.print(F("Input "));
        display.print(getVoltage(), 2);
        display.println('V');
#endif

        ui_data.refresh_timer = millis() + 500;
    }
    else {
        menu.displayTitle();
        switch(menu.getPosition()) {
            case MenuEnum::MENU_SPEED:
                if (motor.isDutyCycleMode()) {
                    PrintBuffer buf(message, sizeof(message));
                    buf.print(pid.duty_cycle * 100 / (float)MAX_DUTY_CYCLE, 1);
                    buf.print('%');
                }
                else {
                    snprintf_P(message, sizeof(message), PSTR("%u rpm"), data.getSetPointRPM());
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
                snprintf_P(message, sizeof(message), PSTR("%u%%"), motor.getMaxDutyCycle() * 100 / MAX_DUTY_CYCLE);
                break;
            case MenuEnum::MENU_BRAKE:
                strcpy_P(message, motor.isBrakeEnabled() ? _T(ENABLED) : _T(DISABLED));
                break;
            case MenuEnum::MENU_STALL:
                snprintf_P(message, sizeof(message), PSTR("%u ms"), motor.getMaxStallTime());
                break;
            default:
                break;
        }
        const uint8_t y = 12;
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

void knob_released(Button& btn, uint16_t duration)
{
    if (motor.isOn()) {
        if (motor.isVelocityMode()) {
            data.pid_config = static_cast<PidConfigEnum>((static_cast<uint8_t>(data.pid_config) + 1) % static_cast<uint8_t>(PidConfigEnum::MAX));
            ui_data.refreshDisplay();
        }
    }
    else if (menu.isOpen()) {
        if (duration > 250 || menu.getPosition() == MenuEnum::MENU_EXIT) {
            knob.write(0);
            ui_data.refreshDisplay();
        }
        else if (menu.isActive()) {
            menu.exit();
            knob.write(menu.getPositionInt() * KNOB_MENU_MULTIPLIER);
            ui_data.menuResetAutoCloseTimer();
        }
        else {
            menu.enter();
            knob.write(0);
            menu_display_value();
        }
    }
    else {
        menu.open();
        knob.write(menu.getPositionInt() * KNOB_MENU_MULTIPLIER);
        ui_data.menuResetAutoCloseTimer();
    }
}

void start_stop_button_pressed(Button& btn)
{
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
}

void read_knob()
{
    if (ui_data.readKnobValue()) {
        if (menu.isMainMenuActive()) {
            if (menu.setPosition(knob.read() / KNOB_MENU_MULTIPLIER)) {
                ui_data.menuResetAutoCloseTimer();
            }
        }
        else {
            int16_t value = knob.readAndReset() * KNOB_INVERTED;
            int16_t new_value;
 #if KNOB_ACCELERATION
            int16_t acceleration = value * 256 / KNOB_ACCELERATION;
            if (acceleration < 0) {
                acceleration = -acceleration;
            }
            if (acceleration > 384) { // make sure it wont get 0
                acceleration -= 128;
            }
            value = (value * acceleration) / 256;
#endif
            // Serial.print(F("knob "));
            // Serial.println(value);
            if (value != 0) {
               if (menu.isActive()) {
                    switch(menu.getPosition()) {
                        case MenuEnum::MENU_SPEED:
                            if (!update_motor_settings(value)) {
                                return;
                            }
                            break;
                        case MenuEnum::MENU_LED:
                            new_value = data.led_brightness + (value * LED_MENU_SPEED);
                            data.led_brightness = max(0, min(LED_MAX_PWM, new_value));
                            data.setLedBrightnessNoDelay();
                            break;
#if HAVE_CURRENT_LIMIT
                        case MenuEnum::MENU_CURRENT:
                            new_value = current_limit.getLimit() + value;
                            if (new_value >= CURRENT_LIMIT_MAX && value > 0) {
                                current_limit.setLimit(CURRENT_LIMIT_DISABLED);
                            }
                            else {
                                current_limit.setLimit(new_value);
                            }
                            current_limit.updateLimit();
                            break;
#endif
                        case MenuEnum::MENU_PWM:
                            motor.setMaxDutyCycle(motor.getMaxDutyCycle() + value);
                            break;
                        case MenuEnum::MENU_STALL:
                            motor.setMaxStallTime(motor.getMaxStallTime() + (value * 5));
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
                    ui_data.menuResetAutoCloseTimer();
                    menu_display_value();
                    switch(menu.getPosition()) {
                        case MenuEnum::MENU_MODE:
                        case MenuEnum::MENU_BRAKE:
                            if (value != 0) {
                                delay(250);
                                knob.write(0);
                            }
                            break;
                        default:
                            break;
                    }
                }
                else {
                    if (update_motor_settings(value)) {
                        ui_data.refreshDisplay();
                    }
                }
            }
        }
    }
}

void setup() {

    Serial.begin(115200);

    motor.begin();
    current_limit.begin();

    pinMode(PIN_LED_DIMMER, OUTPUT);

    pinMode(PIN_ROTARY_ENC_DT, INPUT);
    pinMode(PIN_ROTARY_ENC_CLK, INPUT);

#if HAVE_DEBUG_RPM_SIGNAL_OUT
    digitalWrite(PIN_RPM_SIGNAL_DEBUG_OUT, LOW);
    pinMode(PIN_RPM_SIGNAL_DEBUG_OUT, OUTPUT);
#endif

#if HAVE_VOLTAGE_DETECTION
    pinMode(PIN_VOLTAGE, INPUT);
#endif

    button1.onPress(start_stop_button_pressed);
    button2.onRelease(knob_released);

    read_eeprom();

    setup_display();
    // disable current limit signal LED
    digitalWrite(PIN_CURRENT_LIMIT_LED, LOW);

    menu.add(MenuEnum::MENU_SPEED, PSTR("Speed"));
    menu.add(MenuEnum::MENU_MODE, PSTR("Mode"));
    menu.add(MenuEnum::MENU_LED, PSTR("LED Brightness"));
#if HAVE_CURRENT_LIMIT
    menu.add(MenuEnum::MENU_CURRENT, PSTR("Current Limit"));
#endif
    menu.add(MenuEnum::MENU_PWM, PSTR("Max PWM"));
    menu.add(MenuEnum::MENU_STALL, PSTR("Stall Time"));
    menu.add(MenuEnum::MENU_BRAKE, PSTR("Brake"));
    menu.add(MenuEnum::MENU_INFO, PSTR("Info"));
    menu.add(MenuEnum::MENU_EXIT, PSTR("Exit & Save"));

    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        TCCR1A = 0;
        TCCR1B = TIMER1_PRESCALER_BV;
        TCCR2B = (TCCR2B & 0b11111000) | TIMER2_PRESCALER_BV;

        rpm_sense.begin();

#if HAVE_INTERRUPTS
        setup_interrupts();
#endif
    }

    char message[16];
    set_version(message, sizeof(message));
    Serial.println(message);
#if HAVE_COMPILED_ON_DATE
    Serial.print(F("Compiled on "));
    Serial.println(FPSTR(__compile_date__));
#endif
    display_message(message, DISPLAY_BOOT_VERSION_TIMEOUT, 1);

    current_limit.enable(true);
}

void update_pid_cfg()
{
    //w1.0,0.001,0.0001,0.001,0.00000000001
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
    pid.reset();
    pid.printValues(Serial);
    //print_pid_cfg_serial();
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

        Serial.print(F("gpio: rpm="));
        Serial.print((int)a);
        Serial.print(F(" btns="));
        Serial.print((int)b);
        Serial.print('/');
        Serial.println((int)c);
    }
#endif

#if HAVE_SERIAL_COMMANDS
    if (Serial.available()) {
        uint8_t ch;
        switch(ch = Serial.read()) {
            case 'v':
                Serial.print("Voltage ");
                for(uint8_t i = 0; i < 10; i++) {
                    Serial.println(i);
                    Serial.print(getVoltage(), 3);
                    Serial.print(' ');
                    delay(100);
                }
                Serial.println();
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
#if 0
            case 'b':
                if (digitalRead(PIN_BRAKE)) {
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
                    }
                }
                break;
#endif
            case 'd':
                motor.setMode(ControlModeEnum::DUTY_CYCLE);
                motor.dump(Serial);
                break;
            case 'r':
                motor.setMode(ControlModeEnum::PID);
                motor.dump(Serial);
                break;
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
        }
    }
#endif

    read_knob();
    button1.update();
    button2.update();
    data.setLedBrightness();
    motor.loop();
    refresh_display();
}
