/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

#define FPSTR(str)                              reinterpret_cast<const __FlashStringHelper *>(str)

#define FONT_WIDTH                              6
#define FONT_HEIGHT                             8

class Menu {
public:

    Menu(uint8_t count, Adafruit_SSD1306 &display);
    ~Menu();

    void display();
    void displayTitle();

    void add(uint8_t index, PGM_P name);

    bool isOpen() const {
        return _state != 0;
    }

    bool isActive() const {
        return _state == 2;
    }

    void open();

    void close() {
        _state = 0;
    }

    void up();
    void down();
    void setPosition(uint8_t position);

    uint8_t getPosition() const {
        return _position;
    }

    uint8_t getSize() const {
        return _size;
    }

    void enter() {
        _state = 2;
    }
    void exit();

private:
    void _displayItem(uint8_t y, uint8_t index);

private:
    uint8_t _position;
    uint8_t _size;
    uint8_t _state;
    PGM_P *_items;
    Adafruit_SSD1306 &_display;
};
