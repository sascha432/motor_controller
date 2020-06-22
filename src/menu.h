/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include "progmem_strings.h"

#define FONT_WIDTH                              6
#define FONT_HEIGHT                             8

enum class MenuEnum : uint8_t {
    MENU_SPEED = 0,
    MENU_MODE,
    MENU_LED,
#if HAVE_CURRENT_LIMIT
    MENU_CURRENT,
#endif
    MENU_PWM,
    MENU_STALL,
    MENU_BRAKE,
    MENU_INFO,
    MENU_EXIT,
    MENU_COUNT,
};

class Menu {
public:

    Menu(uint8_t count, Adafruit_SSD1306 &display);
    inline Menu(MenuEnum count, Adafruit_SSD1306 &display) : Menu(static_cast<uint8_t>(count), display) {}
    ~Menu();

    void display();
    void displayTitle();

    void add(uint8_t index, PGM_P name);
    inline void add(MenuEnum index, PGM_P name) {
        add(static_cast<uint8_t>(index), name);
    }

    bool isOpen() const;
    bool isClosed() const {
        return !isOpen();
    }
    bool isActive() const;
    bool isMainMenuActive() const;
    void open();
    void close();

    void up();
    void down();
    bool setPosition(uint8_t position);
    inline bool setPosition(MenuEnum position) {
        return setPosition(static_cast<uint8_t>(position));
    }

    uint8_t getPositionInt() const {
        return _position;
    }
    inline MenuEnum getPosition() const {
        return static_cast<MenuEnum>(_position);
    }

    inline uint8_t getSize() const {
        return _size;
    }

    void enter();
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
