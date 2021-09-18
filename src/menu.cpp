/**
 * Author: sascha_lammers@gmx.de
 */

#include "main.h"

void Menu::display()
{
    if (isActive()) {
        menu_display_submenu();
        return;
    }

    __display().clearDisplay();
    __display().setTextSize(1);
    _displayItem(0, _position + size() - 1);
    _displayItem(12, _position);
    _displayItem(24, _position + 1);
    // highlight
    __display().fillRoundRect(5, 10, SCREEN_WIDTH - 15, 12, 4, SSD1306_INVERSE);

    // scrollbar
    __display().drawRoundRect(SCREEN_WIDTH - 8, 0, 8, SCREEN_HEIGHT, 3, WHITE);
    uint8_t y = 2 + (_position * (SCREEN_HEIGHT - 6) / size());
    __display().fillRoundRect(SCREEN_WIDTH - 7, y, 6, 6, 3, WHITE);

    __display().display();
}
