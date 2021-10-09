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

    template<typename _Type>
    struct progmem_ptr_base {
        progmem_ptr_base(const _Type *ptr) : _ptr(ptr) {}
        progmem_ptr_base(_Type *ptr) : _ptr(ptr) {}

        _Type operator &() const {
            return _ptr;
        }

        progmem_ptr_base &operator++() {
            _ptr++;
            return *this;
        }
        progmem_ptr_base operator++(int) {
            auto tmp = _ptr;
            tmp++;
            return tmp;
        }

        progmem_ptr_base &operator--() {
            _ptr--;
            return *this;
        }
        progmem_ptr_base operator--(int) {
            auto tmp = _ptr;
            tmp--;
            return tmp;
        }

    protected:
        const _Type *_ptr;
    };

    template<typename _Type = uint8_t>
    struct progmem_ptr_byte : progmem_ptr_base<_Type> {
        using base = progmem_ptr_base<_Type>;
        using base::base;
        using base::_ptr;
        using base::operator++;
        using base::operator--;
        using base::operator&;

        operator _Type() const {
            return pgm_read_byte(_ptr);
        }
        _Type operator *() const {
            return static_cast<_Type>(*this);
        }
    };

    template<typename _Type = uint16_t>
    struct progmem_ptr_word : progmem_ptr_base<_Type> {
        using base = progmem_ptr_base<_Type>;
        using base::base;
        using base::_ptr;
        using base::operator++;
        using base::operator--;
        using base::operator&;

        operator _Type() const {
            return pgm_read_word(_ptr);
        }
        _Type operator *() const {
            return static_cast<_Type>(*this);
        }
    };

    template<typename _Type = uint32_t>
    struct progmem_ptr_dword : progmem_ptr_base<_Type> {
        using base = progmem_ptr_base<_Type>;
        using base::base;
        using base::_ptr;
        using base::operator++;
        using base::operator--;
        using base::operator&;

        operator _Type() const {
            return pgm_read_dword(_ptr);
        }
        _Type operator *() const {
            return static_cast<_Type>(*this);
        }
    };

    template<typename _Type = float>
    struct progmem_ptr_float : progmem_ptr_base<_Type> {
        using base = progmem_ptr_base<_Type>;
        using base::base;
        using base::_ptr;
        using base::operator++;
        using base::operator--;
        using base::operator&;

        operator _Type() const {
            return pgm_read_float(_ptr);
        }
        _Type operator *() const {
            return static_cast<_Type>(*this);
        }
    };

    template<typename _Type = void *>
    struct progmem_ptr_pointer : progmem_ptr_base<_Type> {
        using base = progmem_ptr_base<_Type>;
        using base::base;
        using base::_ptr;
        using base::operator++;
        using base::operator--;
        using base::operator&;

        operator _Type() const {
            return pgm_read_ptr(_ptr);
        }
        _Type operator *() const {
            return static_cast<_Type>(*this);
        }
    };

    template<typename _Type, typename _ProgmemPtrType, const size_t _Size>
    struct progmem_array {
        using type = _ProgmemPtrType;

        progmem_array(const _Type *begin) : _begin(begin) {}

        constexpr size_t size() const {
            return _Size;
        }

        constexpr type data() const {
            return begin();
        }

        constexpr type begin() const {
            return _begin;
        }

        constexpr type end() const {
            return _begin + size();
        }

        constexpr type at(size_t n) const {
            if (n < size()) {
                return _begin + n;
            }
            return nullptr;
        }

        constexpr type operator[](size_t n) const {
            return _begin + n;
        }

    private:
        const _Type *_begin;
    };

}

inline uint16_t millis16()
{
    return static_cast<uint16_t>(millis());
}

inline uint16_t micros16()
{
    return static_cast<uint16_t>(micros());
}

inline void readStringUntil(char *begin, char *end, uint16_t timeout = 1000)
{
    char ch = 0;
    uint32_t start = millis();
    while((begin < end) && ((millis() - start) < timeout) && (ch != '\n')) {
        if (Serial.available()) {
            ch = Serial.read();
            *begin++ = ch;
        }
    }
    *begin = 0;
}

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

    void printTrimmed(float f, uint8_t precision = 6) {
        if (f == 0) {
            print(0);
            return;
        }
        print(f, precision);
        _trimFloat();
    }

    void clearPrintTrimmed(float f, uint8_t precision = 6) {
        clear();
        printTrimmed(f, precision);
    }

    char *getBuffer() const {
        return _buffer;
    }

    size_t length() const {
        return _pos;
    }

private:
    void _trimFloat() {
        if (_pos < 2) {
            return;
        }
        auto ptr = _buffer + _pos - 1;
        while(ptr > _buffer && *ptr == '0') {
            ptr--;
            _pos--;
        }
        if (ptr == _buffer) {
            return;
        }
        if (*ptr == '.') {
            ptr += 2;
            _pos++;
            *ptr = 0;
        }
        else if (isdigit(*ptr)) {
            ptr++;
            // _pos++;
            *ptr = 0;
        }
    }

private:
    char *_buffer;
    uint8_t _pos;
    uint8_t _size;
};

#if 0
uint8_t *get_signature(uint8_t *sig);
std::unique_ptr<uint8_t> get_mcu_type(char *&mcu, uint8_t *&sig, uint8_t *&fuses);
#endif

// int count_decimals(double value, uint8_t max_precision = FLT_DIG, uint8_t max_decimals = 8);

// typedef unsigned long ulong;

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
#define FPSTR(str)                              reinterpret_cast<const __FlashStringHelper *>(str)
#endif

#ifndef F
#define F(str)                                  FPSTR(PSTR(str))
#endif

template<typename ..._Args>
static inline constexpr const size_t size_of(_Args&&...args) {
    return sizeof...(args);
};

