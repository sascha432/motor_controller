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
#include "pid_control.h"
#include "timer.h"
#include "DebugBuffer.h"
#include "helpers.h"
#include "menu.h"

#define _D_T(name, value)       static const char _text_##name[] PROGMEM = { value }
#define _T(name)                _text_##name
#define _F(name)                FPSTR(_T(name))

_D_T(OFF, "OFF");
_D_T(DISABLED, "DISABLED");
_D_T(ENABLED, "ENABLED");
_D_T(ERROR, "ERROR");
_D_T(STALLED, "STALLED");
_D_T(BREAKING, "BREAKING");
_D_T(SAVED, "SAVED");
_D_T(_rpm, " rpm");

Data_t data;
UIData_t ui_data;
Encoder knob(PIN_ROTARY_ENC_CLK, PIN_ROTARY_ENC_DT);
PushButton button1(PIN_BUTTON1, PRESSED_WHEN_LOW|ENABLE_INTERNAL_PULLUP);
PushButton button2(PIN_BUTTON2, PRESSED_WHEN_LOW|ENABLE_INTERNAL_PULLUP);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
Menu menu(MenuEnum::MENU_COUNT, display);

void read_eeprom() {
    EEPROMData_t eeprom_data;

    EEPROM.begin();
    EEPROM.get(0, eeprom_data);
    if (eeprom_data.magic == EEPROM_MAGIC) {
        data.copyFrom(eeprom_data);
        pid.setPidValues(eeprom_data.Kp, eeprom_data.Ki, eeprom_data.Kd);
        // surprisingly this was not increasing code size cause of the extra call, but reduced it by 26 byte. gcc -O3
        // pid.Kp = eeprom_data.Kp;
        // pid.Ki = eeprom_data.Ki;
        // pid.Kd = eeprom_data.Kd;
    }
    EEPROM.end();
}

void write_eeprom() {
    EEPROMData_t eeprom_data, eeprom_data_current;

    eeprom_data.magic = EEPROM_MAGIC;
    data.copyTo(eeprom_data);
    pid.getPidValues(eeprom_data.Kp, eeprom_data.Kd, eeprom_data.Ki);

    EEPROM.begin();
    EEPROM.get(0, eeprom_data_current);
    bool changed = memcmp(&eeprom_data, &eeprom_data_current, sizeof(eeprom_data)) != 0;
    if (changed) {
        EEPROM.put(0, eeprom_data);
        Serial.println(F("saved"));
    }
    EEPROM.end();

    if (changed) {
        char message[16];
        strcpy_P(message, _T(SAVED));
        display_message(message, DISPLAY_SAVED_TIMEOUT);
    }
    else {
        ui_data.refreshDisplay();
    }
}


#if PIN_CURRENT_LIMIT_INDICATOR

void pciSetup(byte pin) {
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));
    PCIFR |= bit(digitalPinToPCICRbit(pin));
    PCICR |= bit(digitalPinToPCICRbit(pin));
}

volatile uint32_t current_limit_tripped = 0;
volatile uint8_t *current_limit_pin;
uint8_t current_limit_bit;

ISR(PCINT0_vect) {
    if (*current_limit_pin & current_limit_bit) {
        current_limit_tripped = millis();
    }
}

#endif

void set_version(char *buffer, uint8_t size) {
    snprintf_P(buffer, size, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}


// set PWM duty cycle with enabling/disabling the brake and turning the motor signal led on/off
void set_motor_speed(int speed) {
    if (data.brake_enaged) {
        debug_printf_P(PSTR("speed %u - BREAKING\n"), speed);
        return;
    }
    if (data.motor_state != MotorStateEnum::ON && speed != 0) {
        // motor state must be ON or speed cannot be set to non zero
        debug_printf_P(PSTR("speed %u - OFF\n"), speed);
        return;
    }
    if (speed != 0) {
        digitalWrite(PIN_BRAKE, LOW);
    }
#if DEBUG_MOTOR_SPEED
    Serial.print(F("speed="));
    Serial.println(speed);
#endif
#if 0
    if (speed != 0) {
        if (digitalRead(PIN_BRAKE)) {
            motor_stop(ERROR);
            return;
        }
    }
#endif
    if (speed > data.max_pwm) {
        speed = data.max_pwm;
    }
    analogWrite(PIN_MOTOR_PWM, speed);
    if (speed == 0) {
        digitalWrite(PIN_MOTOR_PWM, LOW);
        if (data.brake_enabled) {
            digitalWrite(PIN_BRAKE, HIGH);
            data.brake_enaged = true;
        }
    }
}


// setup and clear display
void setup_display() {

    // the display needs some time to power up...
    // fade in the LED meanwhile
#if HAVE_LED_FADING
    unsigned long end = millis() + 750;
    while(millis() < end) {
        data.setLedBrightness();
    }
#else
        delay(750);
#endif

    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        display.setTextColor(WHITE);
        display.cp437(true);

        display.clearDisplay();
        display.display();
        ui_data.refreshDisplay();
    }
    else {
        // ui_data.disableRefreshDisplay();
        Serial.println(F("SSD1306 "));
        Serial.println(_F(ERROR));
    }
}

void display_rpm() {
    if (ui_data.display_pulse_length_integral) {
        display.print(RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral));
    } else {
        display.print(0);
    }
    display.println(_F(_rpm));
}

void display_duty_cycle_bar(uint8_t dc) {
    uint8_t width = (dc * (SCREEN_WIDTH - 4)) / MAX_DUTY_CYCLE;
    display.drawRect(0, SCREEN_HEIGHT - 11, SCREEN_WIDTH, 11, WHITE);
    display.fillRect(2, SCREEN_HEIGHT - 9, width, 7, WHITE);
    if (data.max_pwm < MAX_DUTY_CYCLE) {
        uint8_t max_pwm_pos = ((data.max_pwm * (SCREEN_WIDTH - 4)) / MAX_DUTY_CYCLE) + 2;
        if (max_pwm_pos < (SCREEN_WIDTH - 2)) {
            display.drawFastVLine(max_pwm_pos, SCREEN_HEIGHT - 10, 9, INVERSE);
        }
    }
}

void refresh_display() {
    if (millis() > ui_data.refresh_timer)  {
        ui_data.refresh_timer = millis() + DISPLAY_REFRESH_TIME;
        ui_data.refresh_counter++;

        if (menu.isOpen()) {
            menu.close();
            write_eeprom();
            return;
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (data.control_mode == ControlModeEnum::PID) {
            if (data.motor_state == MotorStateEnum::ON) {
                if (data.pid_config == PidConfigEnum::OFF) {
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
                display.print(POTI_TO_RPM(data.set_point_input));
                display.println(_F(_rpm));
            }
        }
        else {
            if (data.motor_state == MotorStateEnum::ON) {
                display.setTextSize(2);
                display.setCursor(0, 5);
                display_rpm();
                display_duty_cycle_bar(pid.duty_cycle);
            }
            else {
                display.setTextSize(1);
                display.println(F("Set point PWM"));
                display.print(POTI_TO_DUTY_CYCLE(data.set_point_input) * 100.0 / MAX_DUTY_CYCLE, 1);
                display.println('%');
            }
        }

#if PIN_CURRENT_LIMIT_INDICATOR

        if (current_limit_tripped) {
            if (get_time_diff(current_limit_tripped, millis()) > 500) {   // display for 500ms
                current_limit_tripped = 0;
            }
            else {
                display.fillCircle(SCREEN_WIDTH - FONT_HEIGHT * 1 - 3, FONT_HEIGHT + 1, FONT_HEIGHT + 1, WHITE);
                display.setTextSize(2);
                display.setTextColor(INVERSE);
                display.setCursor(SCREEN_WIDTH - FONT_HEIGHT * 2, 2);
                display.print('C');
                display.setTextColor(WHITE);
            }
        }

#endif

        constexpr uint8_t y = (SCREEN_HEIGHT - (FONT_HEIGHT * 2)) + 2;
        display.setTextSize(2);
        if (data.motor_state == MotorStateEnum::ERROR) {
            display.setCursor((SCREEN_WIDTH - (5 * FONT_WIDTH * 2)) / 2, y);
            display.print(_F(ERROR));
        }
        else if (data.motor_state == MotorStateEnum::STALLED) {
            display.setCursor((SCREEN_WIDTH - (7 * FONT_WIDTH * 2)) / 2, y);
            display.print(_F(STALLED));
        }
        else if (data.motor_state == MotorStateEnum::OFF) {
            display.setCursor((SCREEN_WIDTH - (3 * FONT_WIDTH * 2)) / 2, y);
            display.print(_F(OFF));
        }
        display.display();
    }
}

void display_message(char *message, uint16_t time, uint8_t size) {
    const uint8_t y = (SCREEN_HEIGHT / 2) - (size * FONT_HEIGHT) + 3;
    const uint8_t x = (SCREEN_WIDTH / 2) - (strlen(message) * size * (FONT_WIDTH / 2));
    display.clearDisplay();
    display.setCursor(x, y);
    display.setTextSize(size);
    display.print(message);
    display.display();
    ui_data.refresh_timer = millis() + time;
}

// set RPM pulse length from poti value
void calc_rpm_pulse_length() {
    pid.set_point_rpm_pulse_length = RPM_SENSE_RPM_TO_US(POTI_TO_RPM(data.set_point_input));
}

// set duty cycle from poti value
void set_duty_cycle() {
    pid.duty_cycle = POTI_TO_DUTY_CYCLE(data.set_point_input);
}

// set duty cycle or RPM pulse length
void update_motor_speed() {
    if (data.control_mode == ControlModeEnum::DUTY_CYCLE) {
        set_duty_cycle();
        if (data.motor_state == MotorStateEnum::ON) {
            set_motor_speed(pid.duty_cycle);
        }
    }
    else {
        calc_rpm_pulse_length();
    }
}

static void led_brightness_str(char *message, uint8_t size) {
    if (data.led_brightness < LED_MIN_PWM) {
        strcpy_P(message, _T(OFF));
    }
    else {
        snprintf_P(message, size, PSTR("%u%%"), data.led_brightness * 100 / 255);
    }
}

#if HAVE_CURRENT_LIMIT

static void current_limit_str(char *message, uint8_t size) {
    if (data.current_limit == CURRENT_LIMIT_DISABLED) {
        strcpy_P(message, _T(DISABLED));
    }
    else {
        PrintBuffer buf(message, size);
        buf.print(CURRENT_LIMIT_DAC_TO_CURRENT(data.current_limit), 1);
        buf.print('A');
    }
}

#endif

//26820
void menu_display_value() {
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
        if (data.current_limit != CURRENT_LIMIT_DISABLED) {
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
        uint16_t value = analogRead(PIN_VOLTAGE);
        for(uint8_t i = 1; i < 10; i++) {
            value += analogRead(PIN_VOLTAGE);
            delay(5);
        }
        // 10 times the ADC reading / 1024 * 5V
        float voltage = value / (1024 * 10 / MCU_VOLTAGE);
        // apply voltage divider
        voltage *= VOLTAGE_DETECTION_DIVIDER;
        display.print(F("Input "));
        display.print(voltage, 2);
        display.println('V');
#endif

        ui_data.menuResetAutoCloseTimer();
    }
    else {
        menu.displayTitle();
        switch(menu.getPosition()) {
            case MenuEnum::MENU_SPEED:
                if (data.control_mode == ControlModeEnum::DUTY_CYCLE) {
                    PrintBuffer buf(message, sizeof(message));
                    buf.print(pid.duty_cycle * 100 / (float)MAX_DUTY_CYCLE, 1);
                    buf.print('%');
                }
                else {
                    snprintf_P(message, sizeof(message), PSTR("%u rpm"), POTI_TO_RPM(data.set_point_input));
                }
                break;
            case MenuEnum::MENU_MODE:
                strcpy_P(message, data.control_mode == ControlModeEnum::PID ? PSTR("Velocity") : PSTR("PWM"));
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
                snprintf_P(message, sizeof(message), PSTR("%u%%"), data.max_pwm * 100 / MAX_DUTY_CYCLE);
                break;
            case MenuEnum::MENU_BRAKE:
                strcpy_P(message, data.brake_enabled ? _T(ENABLED) : _T(DISABLED));
                break;
            case MenuEnum::MENU_STALL:
                snprintf_P(message, sizeof(message), PSTR("%u ms"), data.max_stall_time);
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

bool update_motor_settings(int16_t value) {
    if (value) {
        if (data.pid_config == PidConfigEnum::OFF) {
            if (value < 0) {
                if (data.set_point_input + value > POTI_MIN) {
                    data.set_point_input += value;
                }
                else {
                    data.set_point_input = POTI_MIN;
                }
            }
            else {
                if (data.set_point_input + value < POTI_MAX) {
                    data.set_point_input += value;
                }
                else {
                    data.set_point_input = POTI_MAX;
                }
            }
            update_motor_speed();
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

void knob_released(Button& btn, uint16_t duration) {
    if (data.motor_state == MotorStateEnum::ON) {
        if (data.control_mode == ControlModeEnum::PID) {
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

void start_stop_button_pressed(Button& btn) {
    if (menu.isClosed()) {
        if (data.motor_state != MotorStateEnum::ON && !data.brake_enaged) {
            knob.write(0);
            motor_start();
        }
        else if (data.motor_state == MotorStateEnum::ON) {
            motor_stop(MotorStateEnum::OFF);
            refresh_display();
        }
    }
}

void read_knob() {
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
                            new_value = data.current_limit + value;
                            if (data.current_limit >= CURRENT_LIMIT_MAX && value > 0) {
                                data.current_limit = CURRENT_LIMIT_DISABLED;
                            }
                            else {
                                data.current_limit = max(CURRENT_LIMIT_MIN, min(CURRENT_LIMIT_MAX, new_value));
                            }
                            set_current_limit();
                            break;
#endif
                        case MenuEnum::MENU_PWM:
                            new_value = data.max_pwm + value;
                            data.max_pwm = max(MIN_DUTY_CYCLE, min(MAX_DUTY_CYCLE, new_value));
                            break;
                        case MenuEnum::MENU_STALL:
                            new_value = data.max_stall_time + (value * 5);
                            data.max_stall_time = max(STALL_TIME_MIN, min(STALL_TIME_MAX, new_value));
                            break;
                        case MenuEnum::MENU_MODE:
                            if (data.control_mode == ControlModeEnum::PID) {
                                data.setControlMode(ControlModeEnum::DUTY_CYCLE);
                            } else {
                                data.setControlMode(ControlModeEnum::PID);
                            }
                            break;
                        case MenuEnum::MENU_BRAKE:
                            data.brake_enabled = !data.brake_enabled;
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

#if HAVE_CURRENT_LIMIT

void set_current_limit() {
    // for calibration:
    // PWM DAC 0-255 = 0-135mV with a 1000K/27K voltage divider
    // 0.003R shunt, 1mV = 0.333A, ~0.5A per step...
    // LM393 input offset voltage 1-2mV
    analogWrite(PIN_CURRENT_LIMIT, data.current_limit);
    if (data.current_limit == CURRENT_LIMIT_DISABLED) {
        pinMode(PIN_CURRENT_LIMIT_OVERRIDE, OUTPUT);
        digitalWrite(PIN_CURRENT_LIMIT_OVERRIDE, HIGH);     // set comparator vref to 5V >1000A
    }
    else {
        digitalWrite(PIN_CURRENT_LIMIT_OVERRIDE, LOW);
        pinMode(PIN_CURRENT_LIMIT_OVERRIDE, INPUT);         // floating = use current limit
    }
}

#endif

void setup() {

    Serial.begin(115200);

    digitalWrite(PIN_BRAKE, LOW);
    digitalWrite(PIN_MOTOR_PWM, LOW);
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    pinMode(PIN_BRAKE, OUTPUT);

    pinMode(PIN_CURRENT_LIMIT, OUTPUT);
    pinMode(PIN_LED_DIMMER, OUTPUT);

	pinMode(PIN_RPM_SIGNAL, INPUT);

    pinMode(PIN_ROTARY_ENC_DT, INPUT);
    pinMode(PIN_ROTARY_ENC_CLK, INPUT);

#if HAVE_VOLTAGE_DETECTION
    pinMode(PIN_VOLTAGE, INPUT);
#endif

#if PIN_CURRENT_LIMIT_INDICATOR
    current_limit_pin = portInputRegister(digitalPinToPort(PIN_CURRENT_LIMIT_INDICATOR));
    current_limit_bit = digitalPinToBitMask(PIN_CURRENT_LIMIT_INDICATOR);
    pciSetup(PIN_CURRENT_LIMIT_INDICATOR);
#endif

    button1.onPress(start_stop_button_pressed);
    button2.onRelease(knob_released);

    read_eeprom();

#if HAVE_CURRENT_LIMIT
    set_current_limit();
#endif

    setup_display();
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

    // RPM sensing
    data.setControlMode(data.control_mode);
    init_capture_timer();

    // setup timer2 PWM
    TCCR2B = (TCCR2B & 0b11111000) | TIMER2_PRESCALER;

    char message[16];
    set_version(message, sizeof(message));
    Serial.println(message);
    display_message(message, DISPLAY_BOOT_VERSION_TIMEOUT, 1);
}

// stop motor
void motor_stop(MotorStateEnum state) {
#if DEBUG_MOTOR_SPEED
    if (data.motor_state != state) {
        Serial.print(F("motor_stop="));
        Serial.println(state);
    }
#endif
    cli();
    set_motor_speed(0);
    data.motor_state = state;
    sei();
    refresh_display();

    char message[16];
    switch (state) {
    case MotorStateEnum::ERROR:
        strcpy_P(message, _T(ERROR));
        break;
    case MotorStateEnum::STALLED:
        strcpy_P(message, _T(STALLED));
        break;
    case MotorStateEnum::BREAKING:
    default:
        strcpy_P(message, _T(BREAKING));
        break;
    }
    display_message(message, DISPLAY_MENU_TIMEOUT / 2);
    data.pid_config = PidConfigEnum::OFF;
}

// start motor
void motor_start() {
    cli();
    ui_data = UIData_t();
    data.motor_state = MotorStateEnum::ON;
    reset_capture_timer();
    pid.reset();
    if (data.control_mode == ControlModeEnum::DUTY_CYCLE) {
        update_motor_speed();
    }
    else {
        calc_rpm_pulse_length();
        set_motor_speed(VELOCITY_START_DUTY_CYCLE);
    }
    sei();
    refresh_display();
}

void update_pid_cfg() {
    String str = Serial.readStringUntil('\n');
    char *a, *b, *c;
    a = strtok(str.begin(), ",\r\n");
    if (a) {
        b = strtok(nullptr, ",\r\n");
        pid.Kp = atof(a);
        if (b) {
            c = strtok(nullptr, ",\r\n");
            pid.Ki = atof(b);
            if (c) {
                pid.Kd = atof(c);
            }
        }
    }
    pid.reset();
    pid.printValues(Serial);
    //print_pid_cfg_serial();
}

void loop() {

#if DEBUG_INPUTS

    Serial.print(F("gpio: rpm="));
    Serial.print(digitalRead(PIN_RPM_SIGNAL));
    Serial.print(F(" btn1="));
    Serial.print(digitalRead(PIN_BUTTON1));
    Serial.print(F(" btn2="));
    Serial.println(digitalRead(PIN_BUTTON2));

#endif

#if HAVE_SERIAL_COMMANDS
    if (Serial.available()) {
        uint8_t ch;
        switch(ch = Serial.read()) {
#if DEBUG
            // case 'k':
            //     extern void clear_variance();
            //     clear_variance();
            //     break;
            // case 'K':
            //     extern void display_variance();
            //     display_variance();
            //     break;
            // case 'U':
            //     debug_update_rate += 250;
            //     Serial_printf_P(PSTR("Update rate: %u\n"), debug_update_rate);
            //     break;
            // case 'u':
            //     debug_update_rate = max(250, debug_update_rate - 250);
            //     Serial_printf_P(PSTR("Update rate: %u\n"), debug_update_rate);
            //     break;
            // case 'f':
            //     rpm_sense_dump_measurement();
            //     rpm_sense_reset_measurement();
            //     break;
#endif
#if 0
            case 'l': // display DAC output voltage / current limit
                {
                    Serial.print(F("DAC max. V "));
                    Serial.println(CURRENT_LIMIT_DAC_VOLTAGE, 6);
                    for(uint8_t i = CURRENT_LIMIT_MIN; i <= CURRENT_LIMIT_MAX; i++) {
                        Serial.print(i);
                        Serial.print('=');
                        Serial.print(CURRENT_LIMIT_DAC_TO_CURRENT(i));
                        Serial.print('/');
                        Serial.println(CURRENT_LIMIT_DAC_TO_mV(i));
                    }
                }
                break;
#endif
#if 0
            case 'D':
                Serial.println(F("reinitializing display"));
                setup_display();
                break;
#endif
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
                break;
            case 'd':
                data.setControlMode(ControlModeEnum::DUTY_CYCLE);
                break;
            case 'r':
                data.setControlMode(ControlModeEnum::PID);
                break;
            case 't':
                current_limit_tripped = millis();
                break;
            case '+':
            case '*':
                data.set_point_input += POTI_RANGE / (ch == '*' ? 10 : 128);
                if (data.set_point_input > POTI_MAX)  {
                    data.set_point_input = POTI_MAX;
                }
                else if (data.set_point_input < POTI_MIN) {
                    data.set_point_input = POTI_MIN;
                }
                update_motor_speed();
                break;
            case '-':
            case '/': {
                uint16_t decrement = POTI_RANGE / (ch == '/' ? 10 : 128);
                if (data.set_point_input >= decrement)  {
                    data.set_point_input -= decrement;
                }
                else if (data.set_point_input < POTI_MIN) {
                    data.set_point_input = POTI_MIN;
                }
                update_motor_speed();
            }
            break;
#if DEBUG_PID_CONTROLLER
            case 'W': {
                    data.set_point_input = POTI_MIN;
                    pid.set_point_rpm_pulse_length = RPM_SENSE_RPM_TO_US(pid_test_set_points[0]);
                    pid_test_duration = 5000;

                    update_pid_cfg();

                    motor_stop(MotorStateEnum::OFF);
                    Serial.println(F("PID test"));
                    delay(PID_TEST_WAIT);

                    data.setControlMode(PID);
                    // data.input_mode = VALUE;

                    pid_test_timer_start = millis();
                    pid_test_timer_end = pid_test_timer_start + pid_test_duration;
                    pid_test_counter = 0;
                    pid_test_micros_offset = micros() % 10000;
                    motor_start();
                }
                break;
            case 'F':
                pid_enable_serial_output = !pid_enable_serial_output;
                break;

            case '4':
                pid.Kp += pid_tune_increment;
                break;
            case '1':
                pid.Kp -= pid_tune_increment;
                break;
            case '5':
                pid.Ki += pid_tune_increment;
                break;
            case '2':
                pid.Ki -= pid_tune_increment;
                break;
            case '6':
                pid.Kd += pid_tune_increment;
                break;
            case '3':
                pid.Kd -= pid_tune_increment;
                break;

            case '7':
                if (pid_tune_increment > 10) {
                    pid_tune_increment = 10;
                }
                pid_tune_increment /= 10.0;
                if (pid_tune_increment < 0.00001) {
                    pid_tune_increment = 0.00001;
                }
                break;
            case '8':
                pid_tune_increment = 0;
                break;
            case '9':
                if (pid_tune_increment < 0.00001) {
                    pid_tune_increment = 0.00001;
                }
                pid_tune_increment *= 10.0;
                if (pid_tune_increment > 10) {
                    pid_tune_increment = 10;
                }
                break;
            case ',':
                rpm_sense_average_count--;
                Serial.println(rpm_sense_average_count);
                break;
            case '.':
                rpm_sense_average_count++;
                Serial.println(rpm_sense_average_count);
                break;
#endif
#if DEBUG_RPM_SIGNAL
            case ' ':
                dump_capture_timer_values();
                break;
#endif
        }
    }
#endif

    read_knob();

    button1.update();
    button2.update();

    data.setLedBrightness();

    // check if the RPM signal has not been updated for a given period of time
    if (millis() > capture_timer_last_signal_millis() + data.max_stall_time) {
        if (data.brake_enaged) {
            digitalWrite(PIN_BRAKE, LOW);
            data.brake_enaged = false;
            if (menu.isClosed()) {
                ui_data.refreshDisplay();
            }
        }
        if (data.motor_state == MotorStateEnum::ON && !IS_PID_TUNING) {
            // debug_printf_P(PSTR("STALL %lu > %lu\n"), millis(), capture_timer_last_signal_millis() + MOTOR_MAX_STALL_TIME);
            motor_stop(MotorStateEnum::STALLED);
        }
    }

    refresh_display();

#if DEBUG_PID_CONTROLLER
    if (pid_test_timer_start) {
        if (millis() > pid_test_timer_end) {
            data.set_point_input = 0;
            pid_test_timer_start = 0;
            motor_stop(MotorStateEnum::OFF);
            Serial.println(F("---END---\n\n"));
        }
    }
#endif
}
