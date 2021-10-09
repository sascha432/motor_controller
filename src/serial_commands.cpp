/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"
#include "pid_control.h"
#include "motor.h"

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

void serial_commands()
{
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
                            Serial.printf_P(PSTR("led %u pwm %umW pwm=%u\n"), data.getLedBrightness(), LED_POWER_mW(data.getLedBrightness()), data.getLedBrightness());
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
