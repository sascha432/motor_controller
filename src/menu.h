/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "progmem_strings.h"
#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define FONT_WIDTH  6
#define FONT_HEIGHT 8

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
    MENU_MOTOR,
    MENU_INFO,
    MENU_RESTORE,
    MENU_EXIT,
    MENU_COUNT
};

class Menu {
public:
    static constexpr size_t kMenuSize =  static_cast<size_t>(MenuEnum::MENU_COUNT);

public:
    // Menu(Adafruit_SSD1306 &display);
    Menu();

    void display();
    void displayTitle();

    void add(PGM_P items);
    void add(uint8_t index, PGM_P name);
    void add(MenuEnum index, PGM_P name);

    bool isOpen() const;
    bool isClosed() const;
    bool isActive() const;
    bool isMainMenuActive() const;
    void open();
    void close();

    void up();
    void down();
    bool setPosition(uint8_t position);
    bool setPosition(MenuEnum position);
    uint8_t getPositionInt() const;
    MenuEnum getPosition() const;

    constexpr uint8_t size() const {
        return kMenuSize;
    }

    void enter();
    void exit();

private:
    void _displayItem(uint8_t y, uint8_t index);

    constexpr Adafruit_SSD1306 &__display() {
        return ::display;
    }
    // Adafruit_SSD1306 &__display() {
    //     return _display;
    // }

private:
    // Adafruit_SSD1306 &_display;
    PGM_P _items[kMenuSize];
    uint8_t _position;
    uint8_t _state;
};

inline Menu::Menu() : _position(0), _state(0)
// inline Menu::Menu(Adafruit_SSD1306 &display) : _display(display), _position(0), _state(0)
{
}

inline void Menu::add(PGM_P items)
{
    auto ptr = &_items[0];
    auto startPtr = items;
    while(pgm_read_byte(startPtr)) {
        *ptr++ = startPtr;
        // Serial.println(FPSTR(startPtr));
        while(pgm_read_byte(startPtr)) {
            startPtr++;
        }
        startPtr++;
    }
}

inline void Menu::add(MenuEnum index, PGM_P name)
{
    add(static_cast<uint8_t>(index), name);
}

inline bool Menu::setPosition(MenuEnum position)
{
    return setPosition(static_cast<uint8_t>(position));
}

inline uint8_t Menu::getPositionInt() const
{
    return _position;
}

inline MenuEnum Menu::getPosition() const
{
    return static_cast<MenuEnum>(_position);
}

inline void Menu::displayTitle()
{
    __display().setTextSize(1);
    _displayItem(0, _position);
}

inline void Menu::add(uint8_t index, PGM_P name)
{
    _items[index] = name;
}

inline bool Menu::isOpen() const
{
    return _state != 0;
}

inline bool Menu::isClosed() const
{
    return !isOpen();
}

inline bool Menu::isActive() const
{
    return _state == 2;
}

inline bool Menu::isMainMenuActive() const
{
    return _state == 1;
}

inline void Menu::close()
{
    _state = 0;
}

inline void Menu::open()
{
    _state = 1;
    display();
}

inline void Menu::up()
{
    _position = (_position - 1) % size();
    display();
}

inline void Menu::down()
{
    _position = (_position + 1) % size();
    display();
}

inline bool Menu::setPosition(uint8_t position)
{
    position = position % size();
    if (_position != position) {
        _position = position;
        display();
        return true;
    }
    return false;
}

inline void Menu::enter()
{
    _state = 2;
}

inline void Menu::exit()
{
    _state = 1;
    display();
}

inline void Menu::_displayItem(uint8_t y, uint8_t index)
{
    auto item = _items[index % size()];
    __display().setCursor((SCREEN_WIDTH / 2) - (strlen_P(item) * (FONT_WIDTH / 2)), y);
    __display().print(FPSTR(item));
}
