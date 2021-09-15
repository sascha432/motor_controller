/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

// HAVE_UINT24 0 use internal class for uint24/int24
// HAVE_UINT24 1 use __uint24/__int24
// HAVE_UINT24 -1 use uint32_t/int32_t
#ifndef HAVE_UINT24
#    if defined(__AVR__)
#        define HAVE_UINT24 1
#    elif defined(ESP8266)
#        define HAVE_UINT24 0
#    elif defined(ESP32)
#        define HAVE_UINT24 -1
#    endif
#endif

#if defined(__INTELLISENSE__)
#    if HAVE_UINT24 != 0
#        warning Intellisense detected, disabling HAVE_UINT24
#    endif
#    undef HAVE_UINT24
#    define HAVE_UINT24 0
#endif

#if HAVE_UINT24 == 1

using uint24_t = __uint24;
using int24_t = __int24;

#elif HAVE_UINT24 == -1

using uint24_t = uint32_t;
using int24_t = int32_t;

#else

#include <stddef.h>
#include <stdint.h>

template<typename _BaseType, size_t _Bits>
struct __custom_int {
    using type = __custom_int<_BaseType, _Bits>;
    using base_type = _BaseType;
    static constexpr size_t kBits = _Bits;
    __custom_int() {
    }
    __custom_int(const volatile type &value) : _value(value._value) {
    }
    __custom_int(const type &value) : _value(value._value) {
    }
    template<typename _Ta>
    __custom_int(_Ta value) : _value(value) {
    }
    operator base_type() const {
        return _value;
    }
    operator base_type() {
        return _value;
    }
    __custom_int &operator--() {
        _value--;
        return *this;
    }
    __custom_int &operator++() {
        _value++;
        return *this;
    }
    __custom_int operator--(int) {
        auto tmp = *this;
        tmp._value--;
        return tmp;
    }
    __custom_int operator++(int) {
        auto tmp = *this;
        tmp._value++;
        return tmp;
    }
    template<typename _Ta>
    __custom_int &operator+=(const _Ta &value) {
        _value += value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator-=(const _Ta &value) {
        _value -= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator/=(const _Ta &value) {
        _value /= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator*=(const _Ta &value) {
        _value *= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator%=(const _Ta &value) {
        _value %= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator&=(const _Ta &value) {
        _value &= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator|=(const _Ta &value) {
        _value |= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator^=(const _Ta &value) {
        _value ^= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator<<=(const _Ta &value) {
        _value <<= value;
        return *this;
    }
    template<typename _Ta>
    __custom_int &operator>>=(const _Ta &value) {
        _value >>= value;
        return *this;
    }
    struct __attribute__((packed)) {
        base_type _value: kBits;
    };
};

using uint24_t = __custom_int<uint32_t, 24>;
using int24_t = __custom_int<int32_t, 24>;

#endif
