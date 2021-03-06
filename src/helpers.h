/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>
#include <stdio.h>
#include <float.h>
#include <HardwareSerial.h>
#include "helpers.h"

#if DEBUG
extern uint8_t _debug_level;
void debug_print_millis();

#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL   _D_INFO
#endif
#define _D_ALWAYS     0
#define _D_ERROR      1
#define _D_WARNING    2
#define _D_NOTICE     3
#define _D_INFO       5
#define _D_DEBUG      10
#define _D(level, ...)          { if (_debug_level >= level) { __VA_ARGS__; }; }
#define debug_printf_P(...)     { debug_print_millis(); Serial_printf_P(__VA_ARGS__); }
#define debug_printf(...)       { debug_print_millis(); Serial_printf(__VA_ARGS__); }
#else
#define DEBUG_LEVEL   0
#define _D(...) ;
#define debug_printf_P(...)
#define debug_printf(...)
#endif

template <class T>
class unique_ptr {
public:
    typedef T* T_ptr_t;

    constexpr unique_ptr() : _ptr(nullptr) {
    }
    constexpr unique_ptr(nullptr_t) : _ptr(nullptr) {
    }
    explicit unique_ptr(T_ptr_t ptr) : _ptr(ptr) {
    }
    ~unique_ptr() {
        _delete();
    }
    inline unique_ptr &operator=(nullptr_t) {
        reset(nullptr);
        return *this;
    }
    inline T_ptr_t operator *() const {
        return _ptr;
    }
    inline T_ptr_t operator ->() const {
        return _ptr;
    }
    inline operator bool() const {
        return _ptr != nullptr;
    }
    inline T_ptr_t get() const {
        return _ptr;
    }
    inline void reset(nullptr_t = nullptr) {
        _delete();
    }
    void reset(T_ptr_t ptr) {
        _delete();
        _ptr = ptr;
    }
    T_ptr_t release() {
        auto ptr = _ptr;
        _ptr = nullptr;
        return ptr;
    }
    void swap(T_ptr_t &ptr) {
        auto tmp_ptr = ptr;
        ptr = _ptr;
        _ptr = tmp_ptr;
    }
private:
    void _delete() {
        if (_ptr) {
            delete _ptr;
            _ptr = nullptr;
        }
    }
    T_ptr_t _ptr;
};

class PrintBuffer : public Print {
public:
    PrintBuffer(char *buffer, uint8_t size) : _buffer(buffer), _size(size) {
        clear();
    }

    void clear() {
        _pos = 0;
        *_buffer = 0;
    }

    virtual size_t write(uint8_t data) {
        if (_pos < _size) {
            _buffer[_pos++] = (char)data;
            _buffer[_pos] = 0;
            return 1;
        }
        return 0;
    }

    void printTrimmed(float f) {
        print(f, 6);
        trimFloat();
    }

    void clarPrintTrimmed(float f) {
        clear();
        printTrimmed(f);
    }

    void trimFloat() {
        // trims all zeros or 1 digit followed by at least 3 zeros
        // char *end = _buffer + _pos;
        // uint8_t zero_counter = 0;
        // while(--end > _buffer + 1) {
        //     if (*end == '0' && *(end - 1) != '.') {
        //         if (zero_counter) {
        //             zero_counter++;
        //         } else {
        //             *end = 0;
        //             _pos--;
        //         }
        //     }
        //     else {
        //         if (!zero_counter) { // got 1-9, start counting zeros
        //             zero_counter++;
        //         }
        //         else if (zero_counter >= 3) { // got 1-9 again
        //             _pos -= zero_counter;
        //             _buffer[_pos + 1] = 0;
        //             break;
        //         }
        //         else {
        //             break;
        //         }
        //     }
        // }
        // char *end = _buffer + _pos;
        // while(--end > _buffer + 1) {
        //     if (*end == '0' && *(end - 1) != '.') {
        //         *end = 0;
        //         _pos--;
        //     }
        //     else {
        //         break;
        //     }
        // }
    }

    char *getBuffer() {
        return _buffer;
    }

private:
    char *_buffer;
    uint8_t _pos;
    uint8_t _size;
};

uint8_t *get_signature(uint8_t *sig);
unique_ptr<uint8_t> get_mcu_type(char *&mcu, uint8_t *&sig, uint8_t *&fuses);

int Serial_printf(const char *format, ...);
int Serial_printf_P(PGM_P format, ...);
int Serial_print_bin(uint32_t value, uint8_t bits);
bool Serial_readLine(String &input, bool allowEmpty);
int Serial_print_float(double value, uint8_t max_precision = FLT_DIG, uint8_t max_decimals = 8);
int count_decimals(double value, uint8_t max_precision = FLT_DIG, uint8_t max_decimals = 8);

typedef unsigned long ulong;

#define STR(s)                  _STR(s)
#define _STR(s)                 #s
