/**
 * Author: sascha_lammers@gmx.de
 */

#include "menu.h"

Menu::Menu(uint8_t count, Adafruit_SSD1306 &display) : _position(0), _size(count), _state(0), _display(display) {
    _items = new PGM_P[count];
}

Menu::~Menu() {
    delete _items;
}

void Menu::display() {
    _display.clearDisplay();
    _display.setTextSize(1);
    _displayItem(0, _position + _size - 1);
    _displayItem(12, _position);
    _displayItem(24, _position + 1);
    // highlighjt
    _display.fillRoundRect(5, 10, SCREEN_WIDTH - 15, 12, 4, INVERSE);

    // scrollbar
    _display.drawRoundRect(SCREEN_WIDTH - 8, 0, 8, SCREEN_HEIGHT, 3, WHITE);
    uint8_t y = 2 + (_position * (SCREEN_HEIGHT - 5) / _size);
    _display.fillRoundRect(SCREEN_WIDTH - 7, y, 6, 6, 3, WHITE);

    _display.display();
}

void Menu::displayTitle() {
    _display.setTextSize(1);
    _displayItem(0, _position);
}

void Menu::add(uint8_t index, PGM_P name) {
    _items[index] = name;
}


void Menu::open() {
    _state = 1;
    display();
}

void Menu::up() {
    _position = (_position - 1) % _size;
    display();
}

void Menu::down() {
    _position = (_position + 1) % _size;
    display();
}

void Menu::setPosition(uint8_t position) {
    _position = position % _size;
    display();
}


void Menu::exit() {
    _state = 1;
    display();
}

void Menu::_displayItem(uint8_t y, uint8_t index) {
    auto item = _items[index % _size];
    _display.setCursor((SCREEN_WIDTH / 2) - (strlen_P(item) * (FONT_WIDTH / 2)), y);
    _display.print(FPSTR(item));
}
