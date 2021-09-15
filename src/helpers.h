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

#if HAVE_GCC_OPTIMIZE_O3
#    pragma GCC optimize("O3")
#endif

namespace std {

    template <typename _Ta, typename _Tpred>
    const _Ta& clamp(const _Ta& value, const _Ta& minValue, const _Ta& maxValue, _Tpred pred) {
        if (pred(maxValue, value)) {
            return maxValue;
        }
        if (pred(value, minValue)) {
            return minValue;
        }
        return value;
    }

    template <typename _Ta>
    const _Ta& clamp(const _Ta& value, const _Ta& minValue, const _Ta& maxValue) {
        if (maxValue < value) {
            return maxValue;
        }
        if (value < minValue) {
            return minValue;
        }
        return value;
    }

    #undef min
    #undef max

    template<typename _Ta>
    constexpr const _Ta &min(const _Ta &a, const _Ta &b)
    {
        return (a < b) ? a : b;
    }

    template<typename _Ta>
    constexpr const _Ta &min(const _Ta &a, const _Ta &b, const _Ta &c)
    {
        return min<_Ta>(min<_Ta>(a, b), c);
    }

    template<typename _Ta>
    constexpr const _Ta &min(const _Ta &a, const _Ta &b, const _Ta &c, const _Ta &d)
    {
        return min<_Ta>(min<_Ta>(min<_Ta>(a, b), c), d);
    }

    template<typename _Ta>
    constexpr const _Ta &max(const _Ta &a, const _Ta &b)
    {
        return (a < b) ? b : a;
    }

    template<typename _Ta>
    constexpr const _Ta &max(const _Ta &a, const _Ta &b, const _Ta &c)
    {
        return max<_Ta>(max<_Ta>(a, b), c);
    }

    template<typename _Ta>
    constexpr const _Ta &max(const _Ta &a, const _Ta &b, const _Ta &c, const _Ta &d)
    {
        return max<_Ta>(max<_Ta>(max<_Ta>(a, b), c), d);
    }

    template< typename T > class unique_ptr
    {
    public:
        using pointer = T*;
        unique_ptr() noexcept : ptr(nullptr) {}
        unique_ptr(pointer p) : ptr(p) {}
        pointer operator->() const noexcept { return ptr; }
        T& operator[](decltype(sizeof(0)) i) const { return ptr[i]; }
        void reset(pointer p = pointer()) noexcept
        {
            delete ptr;
            ptr = p;
        }
        T& operator*() const { return *ptr; }
    private:
        pointer ptr;
    };

}

#pragma GCC optimize("Os")

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
std::unique_ptr<uint8_t> get_mcu_type(char *&mcu, uint8_t *&sig, uint8_t *&fuses);

// int count_decimals(double value, uint8_t max_precision = FLT_DIG, uint8_t max_decimals = 8);

typedef unsigned long ulong;

#define STR(s)                  _STR(s)
#define _STR(s)                 #s

/*
// it is required to add the declarations below to the Print class
// framework-arduino-avr\cores\arduino\Print.h
//
class Print {
...

private:
    friend void __debug_printf(const char *format, ...);
    typedef int (* vsnprint_t)(char *, size_t, const char *, va_list ap);
    size_t __printf(vsnprint_t func, const char *format, va_list arg);

public:
    size_t printf(const char *format, ...);
    size_t printf_P(PGM_P format, ...);
};

*/

template <typename T>
inline void swap(T &a, T &b) {
    T c = a;
    a = b;
    b = c;
};


bool Serial_readLine(String &input, bool allowEmpty);
size_t Serial_flush_input();

inline size_t Serial_flush_input() {
    size_t n = 0;
    while(Serial.available()) {
        Serial.read();
        n++;
    }
    return n;
}

#ifndef _STRINGIFY
#define _STRINGIFY(...)                         ___STRINGIFY(__VA_ARGS__)
#endif
#define ___STRINGIFY(...)                       #__VA_ARGS__

#ifndef FPSTR
#define FPSTR(str)                              reinterpret_cast<const __FlashStringHelper *>(PSTR(str))
#endif

template<typename ..._Args>
static inline constexpr const size_t size_of(_Args&&...args) {
    return sizeof...(args);
};
