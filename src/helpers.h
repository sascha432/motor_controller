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


// from FastLED lib8tion.h

/// Return the current seconds since boot in a 16-bit value.  Used as part of the
/// "every N time-periods" mechanism
inline uint16_t seconds16()
{
    uint32_t ms = millis();
    uint16_t s16;
    s16 = ms / 1000;
    return s16;
}

/// Return the current minutes since boot in a 16-bit value.  Used as part of the
/// "every N time-periods" mechanism
inline uint16_t minutes16()
{
    uint32_t ms = millis();
    uint16_t m16;
    m16 = (ms / (60000L)) & 0xFFFF;
    return m16;
}

/// Return the current hours since boot in an 8-bit value.  Used as part of the
/// "every N time-periods" mechanism
inline uint8_t hours8()
{
    uint32_t ms = millis();
    uint8_t h8;
    h8 = (ms / (3600000L)) & 0xFF;
    return h8;
}


/// Helper routine to divide a 32-bit value by 1024, returning
/// only the low 16 bits. You'd think this would be just
///   result = (in32 >> 10) & 0xFFFF;
/// and on ARM, that's what you want and all is well.
/// But on AVR that code turns into a loop that executes
/// a four-byte shift ten times: 40 shifts in all, plus loop
/// overhead. This routine gets exactly the same result with
/// just six shifts (vs 40), and no loop overhead.
/// Used to convert millis to 'binary seconds' aka bseconds:
/// one bsecond == 1024 millis.
inline uint16_t div1024_32_16( uint32_t in32)
{
    uint16_t out16;
#if defined(__AVR__)
    asm volatile (
        "  lsr %D[in]  \n\t"
        "  ror %C[in]  \n\t"
        "  ror %B[in]  \n\t"
        "  lsr %D[in]  \n\t"
        "  ror %C[in]  \n\t"
        "  ror %B[in]  \n\t"
        "  mov %B[out],%C[in] \n\t"
        "  mov %A[out],%B[in] \n\t"
        : [in] "+r" (in32),
        [out] "=r" (out16)
    );
#else
    out16 = (in32 >> 10) & 0xFFFF;
#endif
    return out16;
}

/// bseconds16 returns the current time-since-boot in
/// "binary seconds", which are actually 1024/1000 of a
/// second long.
inline uint16_t bseconds16()
{
    uint32_t ms = millis();
    uint16_t s16;
    s16 = div1024_32_16(ms);
    return s16;
}


// Classes to implement "Every N Milliseconds", "Every N Seconds",
// "Every N Minutes", "Every N Hours", and "Every N BSeconds".
#if 1
#define INSTANTIATE_EVERY_N_TIME_PERIODS(NAME,TIMETYPE,TIMEGETTER) \
class NAME {    \
public: \
    TIMETYPE mPrevTrigger;  \
    TIMETYPE mPeriod;   \
    \
    NAME() { reset(); mPeriod = 1; }; \
    NAME(TIMETYPE period) { reset(); setPeriod(period); };    \
    void setPeriod( TIMETYPE period) { mPeriod = period; }; \
    TIMETYPE getTime() { return (TIMETYPE)(TIMEGETTER()); };    \
    TIMETYPE getPeriod() { return mPeriod; };   \
    TIMETYPE getElapsed() { return getTime() - mPrevTrigger; }  \
    TIMETYPE getRemaining() { return mPeriod - getElapsed(); }  \
    TIMETYPE getLastTriggerTime() { return mPrevTrigger; }  \
    bool ready() { \
        bool isReady = (getElapsed() >= mPeriod);   \
        if( isReady ) { reset(); }  \
        return isReady; \
    }   \
    void reset() { mPrevTrigger = getTime(); }; \
    void trigger() { mPrevTrigger = getTime() - mPeriod; }; \
        \
    operator bool() { return ready(); } \
};
INSTANTIATE_EVERY_N_TIME_PERIODS(CEveryNMillis,uint32_t,millis);
INSTANTIATE_EVERY_N_TIME_PERIODS(CEveryNSeconds,uint16_t,seconds16);
INSTANTIATE_EVERY_N_TIME_PERIODS(CEveryNBSeconds,uint16_t,bseconds16);
INSTANTIATE_EVERY_N_TIME_PERIODS(CEveryNMinutes,uint16_t,minutes16);
INSTANTIATE_EVERY_N_TIME_PERIODS(CEveryNHours,uint8_t,hours8);
#else

// Under C++11 rules, we would be allowed to use not-external
// -linkage-type symbols as template arguments,
// e.g., LIB8STATIC seconds16, and we'd be able to use these
// templates as shown below.
// However, under C++03 rules, we cannot do that, and thus we
// have to resort to the preprocessor to 'instantiate' 'templates',
// as handled above.
template<typename timeType,timeType (*timeGetter)()>
class CEveryNTimePeriods {
public:
    timeType mPrevTrigger;
    timeType mPeriod;

    CEveryNTimePeriods() { reset(); mPeriod = 1; };
    CEveryNTimePeriods(timeType period) { reset(); setPeriod(period); };
    void setPeriod( timeType period) { mPeriod = period; };
    timeType getTime() { return (timeType)(timeGetter()); };
    timeType getPeriod() { return mPeriod; };
    timeType getElapsed() { return getTime() - mPrevTrigger; }
    timeType getRemaining() { return mPeriod - getElapsed(); }
    timeType getLastTriggerTime() { return mPrevTrigger; }
    bool ready() {
        bool isReady = (getElapsed() >= mPeriod);
        if( isReady ) { reset(); }
        return isReady;
    }
    void reset() { mPrevTrigger = getTime(); };
    void trigger() { mPrevTrigger = getTime() - mPeriod; };

    operator bool() { return ready(); }
};
typedef CEveryNTimePeriods<uint16_t,seconds16> CEveryNSeconds;
typedef CEveryNTimePeriods<uint16_t,bseconds16> CEveryNBSeconds;
typedef CEveryNTimePeriods<uint32_t,millis> CEveryNMillis;
typedef CEveryNTimePeriods<uint16_t,minutes16> CEveryNMinutes;
typedef CEveryNTimePeriods<uint8_t,hours8> CEveryNHours;
#endif


#define CONCAT_HELPER( x, y ) x##y
#define CONCAT_MACRO( x, y ) CONCAT_HELPER( x, y )
#define EVERY_N_MILLIS(N) EVERY_N_MILLIS_I(CONCAT_MACRO(PER, __COUNTER__ ),N)
#define EVERY_N_MILLIS_I(NAME,N) static CEveryNMillis NAME(N); if( NAME )
#define EVERY_N_SECONDS(N) EVERY_N_SECONDS_I(CONCAT_MACRO(PER, __COUNTER__ ),N)
#define EVERY_N_SECONDS_I(NAME,N) static CEveryNSeconds NAME(N); if( NAME )
#define EVERY_N_BSECONDS(N) EVERY_N_BSECONDS_I(CONCAT_MACRO(PER, __COUNTER__ ),N)
#define EVERY_N_BSECONDS_I(NAME,N) static CEveryNBSeconds NAME(N); if( NAME )
#define EVERY_N_MINUTES(N) EVERY_N_MINUTES_I(CONCAT_MACRO(PER, __COUNTER__ ),N)
#define EVERY_N_MINUTES_I(NAME,N) static CEveryNMinutes NAME(N); if( NAME )
#define EVERY_N_HOURS(N) EVERY_N_HOURS_I(CONCAT_MACRO(PER, __COUNTER__ ),N)
#define EVERY_N_HOURS_I(NAME,N) static CEveryNHours NAME(N); if( NAME )

#define CEveryNMilliseconds CEveryNMillis
#define EVERY_N_MILLISECONDS(N) EVERY_N_MILLIS(N)
#define EVERY_N_MILLISECONDS_I(NAME,N) EVERY_N_MILLIS_I(NAME,N)
