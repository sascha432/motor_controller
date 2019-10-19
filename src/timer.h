/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

uint32_t get_time_diff(uint32_t start, uint32_t end);
#define is_time_diff_greater(start, end, time)          (get_time_diff(start, end) > time)
#define is_millis_diff_greater(start, time)             (get_time_diff(start, millis()) > time)

class MicrosTimer {
public:
    typedef uint32_t timer_t;

public:
    MicrosTimer();

    // returns the time passed since start was called
    // 0 means either that start has not been called yet or the maximum time limit has been expired
    inline __attribute__((always_inline)) timer_t getTime() {
        return getTime(micros());
    }

    // (re)start timer
    inline __attribute__((always_inline)) void start() {
        return start(micros());
    }

    timer_t getTime(timer_t time);
    void start(timer_t time);

    inline bool isValid() {
        return getTime() != 0;
    }

private:
    bool _valid;
    timer_t _start;
};

class MicrosTimerMinMaxMean : public MicrosTimer {
public:
    MicrosTimerMinMaxMean();

    void integrateTime(timer_t time, int multiplier);
    void integrateTime(int multiplier);

    void addTime(timer_t time);
    void addTime();

    inline timer_t getMean() const {
        return _mean >> 8;
    }

    inline void setMin(timer_t min) {
        _min = min;
    }
    inline timer_t getMin() const {
        return _min;
    }

    inline void setMax(timer_t max) {
        _max = max;
    }
    inline timer_t getMax() const {
        return _max;
    }

    void clear();
    void clearMinMax();

    void printMeanTo(Print &print);
    void printMinMeanMaxTo(Print &print);

private:
    timer_t _mean;
    timer_t _integral;
    timer_t _min;
    timer_t _max;
    unsigned int _count;
};
