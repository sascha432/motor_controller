/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Adafruit_SSD1306.h>
#include <Arduino.h>

#define FONT_WIDTH  6
#define FONT_HEIGHT 8

enum class MenuEnum : uint8_t {
    MENU_SPEED = 0,
    MENU_MODE,
#if HAVE_LED
    MENU_LED,
#endif
#if HAVE_CURRENT_LIMIT
    MENU_CURRENT,
#endif
    MENU_PWM,
    MENU_STALL,
    MENU_BRAKE,
#if HAVE_RPM_PER_VOLT
    MENU_MOTOR,
#endif
    MENU_INFO,
    MENU_RESTORE,
    MENU_EXIT,
    MENU_COUNT
};

class Menu {
public:
    static constexpr size_t kMenuSize =  static_cast<size_t>(MenuEnum::MENU_COUNT);
    static constexpr uint8_t kTimeoutShift = 4;

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
    void closeNoRefresh();

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
    void resetTimer();
    void setTimeout(uint32_t millis);
    uint32_t getTimeout() const;
    void loop();

    // value will be adjusted according to the speed
    // speed < 256 = speed is a multiplier
    // speed > 256 = speed is a divider (max 32767)
    // negative values invert the direction
    // min >= 0 is the minimum value to return
    int16_t getKnobValue(int16_t value, int16_t speed, int16_t min = 1) const;

private:
    void _displayItem(uint8_t y, uint8_t index);

    constexpr Adafruit_SSD1306 &__display() {
        return ::display;
    }
    // Adafruit_SSD1306 &__display() {
    //     return _display;
    // }

private:
    enum class MenuState : uint8_t {
        CLOSED,
        MAIN,
        SUB_MENU,
    };

    // Adafruit_SSD1306 &_display;
    PGM_P _items[kMenuSize];
    uint32_t _timer;
    uint16_t _timeout; // timeout in milliseconds >> kTimeoutShift
    uint8_t _position;
    MenuState _state;
};

inline Menu::Menu() :
    _timer(0),
    _timeout(0),
    _position(0),
    _state(MenuState::CLOSED)
{
}

inline void Menu::add(PGM_P items)
{
    // add all menu items from a string, terminated by too NUL bytes
    auto ptr = &_items[0];
    auto startPtr = items;
    while(pgm_read_byte(startPtr)) {
        *ptr++ = startPtr;
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
    return _state != MenuState::CLOSED;
}

inline bool Menu::isClosed() const
{
    return !isOpen();
}

inline bool Menu::isActive() const
{
    return _state == MenuState::SUB_MENU;
}

inline bool Menu::isMainMenuActive() const
{
    return _state == MenuState::MAIN;
}

inline void Menu::close()
{
    closeNoRefresh();
    refresh_display();
    ui_data.setRefreshTimeoutOnce(Timeouts::Menu::kClose);
}

inline void Menu::closeNoRefresh()
{
    _state = MenuState::CLOSED;
}

inline void Menu::open()
{
    knob.setAcceleration(0);
    resetTimer();
    setTimeout(Timeouts::Menu::kAutoExit);
    _state = MenuState::MAIN;
}

inline void Menu::up()
{
    _position = (_position - 1) % size();
}

inline void Menu::down()
{
    _position = (_position + 1) % size();
}

inline bool Menu::setPosition(uint8_t position)
{
    resetTimer();
    position = position % size();
    if (_position != position) {
        _position = position;
        return true;
    }
    return false;
}

inline void Menu::enter()
{
    switch(getPosition()) {
        case MenuEnum::MENU_BRAKE:
        case MenuEnum::MENU_MODE:
            knob.setAcceleration(0);
            break;
        case MenuEnum::MENU_STALL:
    #if HAVE_RPM_PER_VOLT
        case MenuEnum::MENU_MOTOR:
    #endif
            knob.setAcceleration(255);
            break;
        default:
            knob.setAcceleration(KNOB_ACCELERATION);
            break;
    }
    _state = MenuState::SUB_MENU;
    resetTimer();
}

inline void Menu::exit()
{
    knob.setAcceleration(0);
    _state = MenuState::MAIN;
    resetTimer();
}

inline void Menu::resetTimer()
{
    _timer = millis();
}

inline void Menu::setTimeout(uint32_t millis)
{
    _timeout = millis >> kTimeoutShift;
}

inline uint32_t Menu::getTimeout() const
{
    return _timeout << kTimeoutShift;
}

inline void Menu::loop()
{
    if (getPosition() != MenuEnum::MENU_INFO && (
        (isClosed() && _timeout) ||
        (isOpen() && (millis() - _timer >= getTimeout()))
    )) {
        knob.setAcceleration(KNOB_ACCELERATION);
        _state = MenuState::CLOSED;
        _timeout = 0;
        // checks for changes and updates the display
        write_eeprom();
    }
    else if (isOpen()) {
        display();
    }
}

inline void Menu::_displayItem(uint8_t y, uint8_t index)
{
    auto item = _items[index % size()];
    __display().setCursor((SCREEN_WIDTH / 2) - (strlen_P(item) * (FONT_WIDTH / 2)), y);
    __display().print(FPSTR(item));
}

inline int16_t Menu::getKnobValue(int16_t value, int16_t speed, int16_t min) const
{
    int16_t retval = (static_cast<int32_t>(value) << 8) / speed;
    // do we have a minimum return value?
    if (min > 0) {
        if (retval < 0) {
            // negative value, invert min and check if the return value is greater
            min = -min;
            if (retval >= min)  {
                return min;
            }
        }
        else {
            // positive value, check if it is smaller than min
            if (retval <= min) {
                return min;
            }
        }
    }
    return retval;
}

