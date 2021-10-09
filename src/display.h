/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

void setup_display();
void display_refresh();
void display_message(const char *message, uint16_t time, uint8_t size = 2, size_t len = ~0U);
void display_message(const __FlashStringHelper *message, uint16_t time, uint8_t size = 2);