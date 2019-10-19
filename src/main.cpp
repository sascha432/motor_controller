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

Data_t data = { PID, OFF, false, 0, 0/*LED*/, 115/*~20A*/, 1000 };
UIData_t ui_data;
Encoder knob(PIN_ROTARY_ENC_CLK, PIN_ROTARY_ENC_DT);
PushButton button1(PIN_BUTTON1, PRESSED_WHEN_LOW|ENABLE_INTERNAL_PULLUP);
PushButton button2(PIN_BUTTON2, PRESSED_WHEN_LOW|ENABLE_INTERNAL_PULLUP);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET_PIN);
Menu menu(MENU_COUNT, display);

void read_eeprom() {
    EEPROMData_t eeprom_data;

    EEPROM.begin();
    EEPROM.get(0, eeprom_data);
    if (eeprom_data.magic == EEPROM_MAGIC) {
        data.control_mode = eeprom_data.control_mode;
        data.set_point_input = eeprom_data.set_point_input;
        data.led_brightness = eeprom_data.led_brightness;
        data.current_limit = eeprom_data.current_limit;
        data.max_stall_time = eeprom_data.max_stall_time;
        pid.Kp = eeprom_data.Kp;
        pid.Kd = eeprom_data.Kd;
        pid.Ki = eeprom_data.Ki;
    }
    EEPROM.end();
}

void write_eeprom() {
    EEPROMData_t eeprom_data, eeprom_data_current;

    eeprom_data.magic = EEPROM_MAGIC;
    eeprom_data.control_mode = data.control_mode;
    eeprom_data.set_point_input = data.set_point_input;
    eeprom_data.led_brightness = data.led_brightness;
    eeprom_data.current_limit = data.current_limit;
    eeprom_data.max_stall_time = data.max_stall_time;
    eeprom_data.Kp = pid.Kp;
    eeprom_data.Kd = pid.Kd;
    eeprom_data.Ki = pid.Ki;

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
        strcpy_P(message, PSTR("SAVED"));
        display_message(message, DISPLAY_SAVED_TIMEOUT);
    }
    else {
        ui_data.refresh_timer = 0;
    }
}

void set_version(char *buffer, uint8_t size) {
    snprintf_P(buffer, size, PSTR("Version %u.%u.%u"), VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH);
}

void set_control_mode(ControlModeEnum_t mode) {
    if (data.motor_state != ON) {
        data.control_mode = mode;
        if (mode == PID) {
            capture_timer_set_callback(update_pid_controller);
        }
        else {
            capture_timer_set_callback(update_duty_cycle);
        }
    }
}

// set PWM duty cycle with enabling/disabling the brake and turning the motor signal led on/off
void set_motor_speed(int speed) {
    if (data.brake_enaged) {
        debug_printf_P(PSTR("speed %u - BRAKING\n"), speed);
        return;
    }
    if (data.motor_state != ON && speed != 0) {
        // motor state must be ON or speed cannot be set to non zero
        debug_printf_P(PSTR("speed %u - OFF\n"), speed);
        return;
    }
    if (speed != 0) {
        digitalWrite(PIN_BRAKE, LOW);
    }
    analogWrite(PIN_MOTOR_PWM, speed);
    if (speed == 0) {
        digitalWrite(PIN_MOTOR_PWM, LOW);
        digitalWrite(PIN_BRAKE, HIGH);
        data.brake_enaged = true;
    }
}

// setup and clear display
void setup_display() {
    if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        display.setTextColor(WHITE);
        display.cp437(true);

        display.clearDisplay();
        display.display();
        ui_data.refresh_timer = 0;
    }
    else {
        ui_data.refresh_timer = ~0; // disables refresh_display()
        debug_printf_P(PSTR("SSD1306 error\n"));
    }
}

void display_rpm() {
    if (ui_data.display_pulse_length_integral) {
        display.print(RPM_SENSE_US_TO_RPM(ui_data.display_pulse_length_integral));
    } else {
        display.print(0);
    }
    display.print(F(" rpm"));
}

void display_duty_cycle_bar(uint8_t dc) {
    uint8_t width = dc * (SCREEN_WIDTH - 4) / MAX_DUTY_CYCLE;
    display.drawRect(0, SCREEN_HEIGHT - 11, SCREEN_WIDTH, 11, WHITE);
    display.fillRect(2, SCREEN_HEIGHT - 9, width, 7, WHITE);
}

void refresh_display() {
    if (millis() > ui_data.refresh_timer)  {
        ui_data.refresh_timer = millis() + 100;
        ui_data.refresh_counter++;

        if (menu.isOpen()) {
            menu.close();
            write_eeprom();
            return;
        }

        display.clearDisplay();
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (data.control_mode == PID) {
            if (data.motor_state == ON) {
                display.setTextSize(2);
                display.setCursor(0, 5);
                display_rpm();
                display_duty_cycle_bar(ui_data.display_duty_cycle_integral);
            }
            else {
                display.setTextSize(1);
                display.println(F("Set point velocity"));
                display.print(POTI_TO_RPM(data.set_point_input));
                display.println(F(" rpm"));
            }
        }
        else {
            if (data.motor_state == ON) {
                display.setTextSize(2);
                display.setCursor(0, 5);
                display_rpm();
                display_duty_cycle_bar(pid.duty_cycle);
            }
            else {
                display.setTextSize(1);
                display.println(F("Set point PWM"));
                display.print(pid.duty_cycle * 100.0 / MAX_DUTY_CYCLE, 1);
                display.println('%');
            }
        }

        const uint8_t y = (SCREEN_HEIGHT - (FONT_HEIGHT * 2)) + 2;
        display.setTextSize(2);
        if (data.motor_state == ERROR) {
            display.setCursor((SCREEN_WIDTH - (5 * FONT_WIDTH * 2)) / 2, y);
            display.print(F("ERROR"));
        }
        else if (data.motor_state == STALLED) {
            display.setCursor((SCREEN_WIDTH - (7 * FONT_WIDTH * 2)) / 2, y);
            display.print(F("STALLED"));
        }
        else if (data.motor_state == OFF) {
            display.setCursor((SCREEN_WIDTH - (3 * FONT_WIDTH * 2)) / 2, y);
            display.print(F("OFF"));
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
    if (data.control_mode == DUTY_CYCLE) {
        set_duty_cycle();
        if (data.motor_state == ON) {
            set_motor_speed(pid.duty_cycle);
        }
    }
    else {
        calc_rpm_pulse_length();
    }
}

void menu_reset_timer() {
    ui_data.refresh_timer = millis() + DISPLAY_MENU_TIMEOUT;
}

void menu_display_value() {
    char message[16];

    display.clearDisplay();
    if (menu.getPosition() == MENU_INFO) {
        display.setTextSize(1);
        display.setCursor(0, 0);

        set_version(message, sizeof(message));
        display.println(message);

        PrintBuffer print_buf(message, sizeof(message));
        display.print(F("PID "));
        print_buf.printTrimmedFloat(pid.Kp);
        display.print(message);
        display.print(' ');
        print_buf.printTrimmedFloat(pid.Ki);
        display.print(message);
        display.print(' ');
        print_buf.printTrimmedFloat(pid.Kd);
        display.println(message);

#if HAVE_VOLTAGE_DETECTION
        uint16_t value = analogRead(PIN_VOLTAGE);
        for(uint8_t i = 1; i < 10; i++) {
            value += analogRead(PIN_VOLTAGE);
            delay(5);
        }
        // 10 times the ADC reading / 1024 * 5V
        float voltage = value / (1024 * 10 / MCU_VOLTAGE);
        // resistor divider
        voltage *= VOLTAGE_DETECTION_DIVIDER;
        display.print(F("Input "));
        display.print(voltage, 2);
        display.println('V');
#endif

        menu_reset_timer();
    }
    else {
        menu.displayTitle();
        switch(menu.getPosition()) {
            case MENU_SPEED:
                if (data.control_mode == DUTY_CYCLE) {
                    uint16_t value = pid.duty_cycle * 1000UL / MAX_DUTY_CYCLE;
                    snprintf_P(message, sizeof(message), PSTR("%u.%u%%"), value / 10, value % 10);
                }
                else {
                    snprintf_P(message, sizeof(message), PSTR("%u rpm"), POTI_TO_RPM(data.set_point_input));
                }
                break;
            case MENU_MODE:
                strcpy_P(message, data.control_mode == PID ? PSTR("Velocity") : PSTR("PWM"));
                break;
            case MENU_LED:
                if (data.led_brightness < LED_MIN_PWM) {
                    strcpy_P(message, PSTR("OFF"));
                }
                else {
                    snprintf_P(message, sizeof(message), PSTR("%u%%"), data.led_brightness * 100 / 255);
                }
                break;
            case MENU_CURRENT:
                if (data.current_limit == CURRENT_LIMIT_DISABLED) {
                    strcpy_P(message, PSTR("DISABLED"));
                }
                else {
                    uint16_t fp = CURRENT_LIMIT_DAC_TO_CURRENT(data.current_limit * 10);
                    snprintf_P(message, sizeof(message), PSTR("%u.%uA"), fp / 10, fp % 10);
                }
                break;
            case MENU_STALL:
                snprintf_P(message, sizeof(message), PSTR("%u ms"), data.max_stall_time);
                break;
        }
        const uint8_t y = 12;
        const uint8_t x = (SCREEN_WIDTH / 2) - (strlen(message) * FONT_WIDTH);
        display.setCursor(x, y);
        display.setTextSize(2);
        display.print(message);
    }
    display.display();
}

bool update_speed() {
    auto value = -knob.readAndReset();
    int32_t change = value * (abs(value / KNOB_ACCELERATION) + 1);
    if (change < 0) {
        if (data.set_point_input + change > POTI_MIN) {
            data.set_point_input += change;
        }
        else {
            data.set_point_input = POTI_MIN;
        }
        update_motor_speed();
    }
    else if (change > 0) {
        if (data.set_point_input + change < POTI_MAX) {
            data.set_point_input += change;
        }
        else {
            data.set_point_input = POTI_MAX;
        }
        update_motor_speed();
    }
    return change != 0;
}

void knob_released(Button& btn, uint16_t duration) {
    if (data.motor_state == ON) {
        // no menu while running
    }
    else if (menu.isOpen()) {
        if (duration > 250 || menu.getPosition() == MENU_EXIT) {
            knob.write(0);
            ui_data.refresh_timer = 0;
        }
        else if (menu.isActive()) {
            menu.exit();
            knob.write(menu.getPosition() * KNOW_MENU_MULTIPLIER + KNOB_MENU_CENTER);
            menu_reset_timer();
        }
        else {
            menu.enter();
            knob.write(0);
            menu_display_value();
        }
    }
    else {
        menu.open();
        knob.write(menu.getPosition() * KNOW_MENU_MULTIPLIER + KNOB_MENU_CENTER);
        menu_reset_timer();
    }
}

void start_stop_button_pressed(Button& btn) {
    if (!menu.isOpen()) {
        if (data.motor_state != ON && !data.brake_enaged) {
            knob.readAndReset();
            motor_start();
            refresh_display();
        }
        else if (data.motor_state == ON) {
            motor_stop(OFF);
            refresh_display();
        }
    }
}

void read_knob() {
    if (millis() <= ui_data.knob_read_timer) {
        return;
    }
    ui_data.knob_read_timer = millis() + 150;

    if (menu.isActive()) {
        int16_t value;
        switch(menu.getPosition()) {
            case MENU_SPEED:
                if (update_speed()) {
                    menu_reset_timer();
                }
                break;
            case MENU_LED:
                value = knob.readAndReset();
                if (value != 0) {
                    int16_t new_value = data.led_brightness - value;
                    data.led_brightness = max(0, min(255, new_value));
                    set_led_brightness();
                    menu_reset_timer();
                }
                break;
            case MENU_CURRENT:
                value = -knob.readAndReset();
                if (value != 0) {
                    int16_t new_value = data.current_limit + value;
                    if (data.current_limit >= CURRENT_LIMIT_MAX && value > 0) {
                        data.current_limit = CURRENT_LIMIT_DISABLED;
                    }
                    else {
                        data.current_limit = max(CURRENT_LIMIT_MIN, min(CURRENT_LIMIT_MAX, new_value));
                    }
                    set_current_limit();
                    menu_reset_timer();
                }
                break;
            case MENU_STALL:
                value = -knob.readAndReset();
                if (value != 0) {
                    int16_t new_value = data.max_stall_time + (value * 5);
                    data.max_stall_time = max(STALL_TIME_MIN, min(STALL_TIME_MAX, new_value));
                    menu_reset_timer();
                }
                break;
            case MENU_MODE:
                value = abs(knob.read());
                if (value >= 4) {
                    knob.write(0);
                    if (data.control_mode == PID) {
                        set_control_mode(DUTY_CYCLE);
                    } else {
                        set_control_mode(PID);
                    }
                    menu_reset_timer();
                }
                break;
        }
        menu_display_value();
    }
    else if (menu.isOpen()) {
        menu.setPosition(knob.read() / KNOW_MENU_MULTIPLIER);
        menu_reset_timer();
    }
    else {
        update_speed();
    }
}

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

void set_led_brightness() {
    if (data.led_brightness < LED_MIN_PWM) {
        analogWrite(PIN_LED_DIMMER, 0);
    } else {
        analogWrite(PIN_LED_DIMMER, data.led_brightness);
    }
}

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

    button1.onPress(start_stop_button_pressed);
    button2.onRelease(knob_released);

    setup_display();
    menu.add(MENU_SPEED, PSTR("Speed"));
    menu.add(MENU_MODE, PSTR("Mode"));
    menu.add(MENU_LED, PSTR("LED Brightness"));
    menu.add(MENU_CURRENT, PSTR("Current Limit"));
    menu.add(MENU_STALL, PSTR("Stall Time"));
    menu.add(MENU_INFO, PSTR("Info"));
    menu.add(MENU_EXIT, PSTR("Exit & Save"));

    read_eeprom();

    set_current_limit();
    set_led_brightness();

    // RPM sensing
    set_control_mode(data.control_mode);
    init_capture_timer();

    // setup timer2 PWM
    TCCR2B = (TCCR2B & 0b11111000) | TIMER2_PRESCALER;

    char message[16];
    set_version(message, sizeof(message));
    display_message(message, DISPLAY_BOOT_VERSION_TIMEOUT, 1);
}

// stop motor
void motor_stop(MotorStateEnum_t state) {
    cli();
    set_motor_speed(0);
    data.motor_state = state;
    sei();
    refresh_display();

    char message[16];
    switch (state) {
    case ERROR:
        strcpy_P(message, PSTR("ERROR!"));
        break;
    case STALLED:
        strcpy_P(message, PSTR("STALLED!"));
        break;
    default:
        strcpy_P(message, PSTR("BRAKING"));
        break;
    }
    display_message(message, DISPLAY_MENU_TIMEOUT / 2);
}

// start motor
void motor_start() {
    cli();
    memset(&ui_data, 0, sizeof(ui_data));
    update_motor_speed();
    data.motor_state = ON;
    reset_capture_timer();
    reset_pid();
    set_motor_speed(pid.duty_cycle);
    sei();
    refresh_display();
}

void update_duty_cycle() {
    ui_data.display_pulse_length_integral = (ui_data.display_pulse_length_integral * DISPLAY_RPM_MULTIPLIER + capture_timer_get_micros()) / (DISPLAY_RPM_MULTIPLIER + 1);
}

void print_pid_cfg() {
    char message[16];
    PrintBuffer buf(message, sizeof(message));
    Serial.print('w');
    buf.printTrimmedFloat(pid.Kp);
    Serial.print(message);
    Serial.print(',');
    buf.printTrimmedFloat(pid.Ki);
    Serial.print(message);
    Serial.print(',');
    buf.printTrimmedFloat(pid.Kd);
    Serial.println(message);
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
    reset_pid();
    print_pid_cfg();
}

void loop() {

    if (Serial.available()) {
        uint8_t ch;
        switch(ch = Serial.read()) {
            // case '?':
            // case 'h':
            // case 'H':
            //     Serial.print(F(
            //         "Keys:\n"
            //         "-----\n"
            //         "r = PID control\n"
            //         "d = duty cycle control\n"
            //         "e = print PID controller configuration\n"
            //         "w = configure PID controller\n"
            //         "W = run PID controller test\n"
            //         "F = toggle output of PID controller\n"
            //         "+/- in/decrease motor speed\n"
            //         "*,/ increase motor speed rapidly\n"
            //         "S = start/stop button\n"
            //         "./, in/decrease rpm sense averaging (RPMs)\n"
            //         "f = measure RPM sensing\n"
            //         "U/u in/decrease debug update rate\n"
            //         "7 = Ti /= 10       8 = Ti = 0          9 = Ti *= 10\n"
            //         "4 = Kp++           5 = Ki++            6 = Kd++\n"
            //         "1 = Kp--           2 = Ki--            3 = Kd--\n"
            //         "y = dt--           Y = dt--\n"
            //         // "1 = duty cycle or RPM\n"
            //         // "2 = soft start Limit\n"
            //     ));
            //     break;
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
            case 'e':
                print_pid_cfg();
                break;
            case 'w':
                update_pid_cfg();
                write_eeprom();
                break;
            case 'S':
                start_stop_button_pressed(*(Button *)(nullptr));
                break;
            case 'd':
                set_control_mode(DUTY_CYCLE);
                break;
            case 'r':
                set_control_mode(PID);
                break;
            case '+':
            case '*':
                data.set_point_input += POTI_RANGE / (ch == '*' ? 10 : 128);
                if (data.set_point_input > POTI_MAX)  {
                    data.set_point_input = POTI_MAX;
                }
                if (data.set_point_input < POTI_MIN) {
                    data.set_point_input = POTI_MIN;
                }
                update_motor_speed();
                break;
            case '-':
            case '/':
                uint16_t decrement = POTI_RANGE / (ch == '/' ? 10 : 128);
                if (data.set_point_input >= decrement)  {
                    data.set_point_input -= decrement;
                }
                if (data.set_point_input < POTI_MIN) {
                    data.set_point_input = POTI_MIN;
                }
                update_motor_speed();
                break;
#if DEBUG_PID_CONTROLLER
            case 'W': {
                    data.set_point_input = POTI_MIN;
                    set_point_rpm_pulse_length = RPM_SENSE_RPM_TO_US(pid_test_set_points[0]);
                    pid_test_duration = 5000;

                    update_pid_cfg();

                    motor_stop(OFF);
                    Serial.println(F("PID test"));
                    delay(PID_TEST_WAIT);

                    set_control_mode(PID);
                    data.input_mode = VALUE;

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
                Kp += pid_tune_increment;
                break;
            case '1':
                Kp -= pid_tune_increment;
                break;
            case '5':
                Ki += pid_tune_increment;
                break;
            case '2':
                Ki -= pid_tune_increment;
                break;
            case '6':
                Kd += pid_tune_increment;
                break;
            case '3':
                Kd -= pid_tune_increment;
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

    read_knob();

    button1.update();
    button2.update();

    // check if the RPM signal has not been updated for a given period of time
    if (millis() > capture_timer_last_signal_millis() + data.max_stall_time) {
        if (data.brake_enaged) {
            digitalWrite(PIN_BRAKE, LOW);
            data.brake_enaged = false;
            if (!menu.isOpen()) {
                ui_data.refresh_timer = 0;
            }
        }
        if (data.motor_state == ON && !IS_PID_TUNING) {
            // debug_printf_P(PSTR("STALL %lu > %lu\n"), millis(), capture_timer_last_signal_millis() + MOTOR_MAX_STALL_TIME);
            motor_stop(STALLED);
        }
    }

    refresh_display();

#if DEBUG_PID_CONTROLLER
    if (pid_test_timer_start) {
        if (millis() > pid_test_timer_end) {
            data.set_point_input = 0;
            pid_test_timer_start = 0;
            motor_stop(OFF);
            Serial.println(F("---END---\n\n"));
        }
    }
#endif
}
