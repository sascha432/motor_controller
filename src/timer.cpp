/**
 * Author: sascha_lammers@gmx.de
 */

#include "timer.h"
#include "helpers.h"

uint32_t get_time_diff(uint32_t start, uint32_t end)
{
    if (end >= start) {
        return end - start;
    }
    // handle overflow
    return end + ~start + 1;
}

MicrosTimer::MicrosTimer() : _valid(false)
{
}

MicrosTimer::timer_t MicrosTimer::getTime(timer_t time)
{
    if (!_valid) {
        return 0;
    }
    if (time >= _start) {
        time -= _start;
    }
    else { // timer_t overflow
        time += ~_start + 1;
        if (time >= (~0UL >> 1)) {    // most likely micros() skipped overflows and is out of sync
            _valid = false;
            return 0;
        }
    }
    return time;
}

void MicrosTimer::start(timer_t time)
{
    _start = time;
    _valid = true;
}


MicrosTimerMinMaxMean::MicrosTimerMinMaxMean() : MicrosTimer()
{
    clear();
}

void MicrosTimerMinMaxMean::integrateTime(int multiplier)
{
    auto time = getTime();
    if (time) {
        integrateTime(time, multiplier);
    }
}

void MicrosTimerMinMaxMean::integrateTime(timer_t time, int multiplier)
{
    _min = std::min(_min, time);
    _max = std::max(_max, time);
    _mean = ((_mean * multiplier) + (time << 8)) / (multiplier + 1);
}

void MicrosTimerMinMaxMean::addTime(timer_t time)
{
    integrateTime(time, _count);
    _count++;
}

void MicrosTimerMinMaxMean::addTime()
{
    auto time = getTime();
    if (time) {
        addTime(time);
    }
}

void MicrosTimerMinMaxMean::clear()
{
    clearMinMax();
    _mean = 0;
    _count = 0;
}

void MicrosTimerMinMaxMean::clearMinMax() {

    _min = ~0UL;
    _max = 0;
}

void MicrosTimerMinMaxMean::printMeanTo(Print &print)
{
    print.print(getMean());
}

void MicrosTimerMinMaxMean::printMinMeanMaxTo(Print &print)
{
    if (_min == ~0UL) {
        print.print("NA");

    } else {
        print.print(_min);
    }
    print.print('-');
    print.print(getMean());
    print.print('-');
    print.print(_max);
}
