/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include "main.h"
#include "timer.h"

class PidController {
public:

    #if HAVE_PID_CONTROLLER_STATS

    // number of records to buffer
    static constexpr size_t kMaxStatsSize = 8;

    static constexpr uint8_t kShiftTime = 7; // ~8.3 seconds

    enum class StatsState : uint8_t {
        IDLE,
        RUNNING,
        DONE,
    };

    struct Data {
        uint16_t _time;
        uint16_t _rpm;
        uint16_t _rpmAvg;
        uint16_t _setPoint;
        uint8_t _dutyCycle;
    };

    struct Stats {
        uint32_t _start;
        uint32_t _stop;
        uint32_t _dropped;
        // uint32_t _sent;
        // uint32_t _add;
        StatsState _state;
        Data *_data;
        Data *_head;
        Data *_tail;

        Stats() :
            _state(StatsState::IDLE),
            _data(nullptr),
            _head(_data),
            _tail(_data)
        {
        }

        // restart
        void start() {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (!_data) {
                    _data = new Data[kMaxStatsSize];
                    if (!_data) {
                        clear();
                        NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
                            Serial.println(F("alloc PidStats failed"));
                        }
                        return;
                    }
                }
                _start = micros();
                _stop = 0;
                _state = StatsState::RUNNING;
                _dropped = 0;
                // _sent = 0;
                // _add = 0;
                _head = _tail = _data;
            }
        }

        // stop
        void stop() {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                _state = StatsState::DONE;
            }
        }

        // stop and discard
        void clear() {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                _state = StatsState::DONE;
                if (_data) {
                    delete[] _data;
                }
                _data = nullptr;
                _head = _tail = _data;
            }
        }

        bool isAutoStop() const {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                return _stop == 0;
            }
            __builtin_unreachable();
        }

        // bool empty() const {
        //     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        //         return _empty();
        //     }
        //     __builtin_unreachable();
        // }

        // print single record
        bool flushSingle() {
            Data data;
            // uint16_t dropped;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (_empty()) {
                    return false;
                }
                // "time in seconds" "set point rpm" "rpm avg" "rpm" "duty cycle"[ "droppd records"]
                // "0000000000 9999 8888 7777 255[ 12345]\n"
                // make sure that the entire row fits into the serial buffer or wait until enough is free
                if (Serial.availableForWrite() < 32) {
                    return false;
                }
                // dropped = _dropped;
                // _dropped = 0;
                data = _getData();
            }
            NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
                char sep = ',';
                auto time = (static_cast<uint32_t>(data._time) << kShiftTime);
                if (!time || !data._rpmAvg || !data._setPoint || !data._rpm || !data._dutyCycle) {
                    return true;
                }
                Serial.print(time);
                // Serial.print(sep);
                // Serial.print(data._setPoint);
                Serial.print(sep);
                Serial.print(data._rpmAvg);
                Serial.print(sep);
                Serial.print(data._rpm);
                Serial.print(sep);
                Serial.print(data._dutyCycle);
                Serial.print('\n');
            }
            return true;
        }

        // // print stats and continue collecting
        // void flush() {
        //     while(flushSingle()) {
        //     }
        // }

        // add data point
        void add(uint16_t rpm, uint16_t rpmAvg, uint16_t setPoint, uint8_t dutyCycle) {
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                if (_state == StatsState::DONE) {
                    return;
                }
                // _add++;
                if (_state == StatsState::IDLE) {
                    start();
                }
                if (_state != StatsState::RUNNING) {
                    return;
                }
                if (!_hasSpace()) {
                    _dropped++;
                    return;
                }
                uint32_t ms = micros();
                auto &data = *_tail;
                data._time = (ms - _start) >> kShiftTime;
                data._rpm = rpm;
                data._rpmAvg = rpmAvg;
                data._setPoint = setPoint;
                data._dutyCycle = dutyCycle;
                _tail = next(_tail);
            }
        }

        // bool hasSpace() const {
        //     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        //         return _state == StatsState::RUNNING &&  (next(_head) != _tail);
        //     }
        //     __builtin_unreachable();
        // }

        void dumpInfo(Print &output) {
            if (output.availableForWrite() < 32) {
                NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
                    output.flush();
                }
            }
            struct Info {
                long dropped;
                // long sent;
                // long add;
                StatsState state;
                int size;
                int space;
            };
            Info info;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                info.dropped = _dropped;
                // info.sent = _sent;
                // info.add = _add;
                info.state = _state;
                info.size = size();
                info.space = _hasSpace();
            }
            NONATOMIC_BLOCK(NONATOMIC_RESTORESTATE) {
                output.print(F("PID stats "));
                switch(info.state) {
                    case StatsState::IDLE:
                        output.print(F("IDLE"));
                        break;
                    case StatsState::RUNNING:
                        output.print(F("RUN"));
                        break;
                    case StatsState::DONE:
                        output.print(F("STOP"));
                        break;
                }
                output.printf_P(PSTR(": dropped=%ld"), info.dropped); //, info.sent, info.add);
                output.printf_P(PSTR(" space=%d/%d/"), info.space, info.size);
                output.println(kMaxStatsSize);
            }
        }

    private:
        bool _empty() const {
            return !_head || (_head == _tail);
        }

        bool hasSpace() const {
            return _state == StatsState::RUNNING && _head && _hasSpace();
        }

        bool _hasSpace() const {
            return next(_head) != _tail;
        }

        Data *next(Data *ptr) const {
            if (++ptr == &_data[kMaxStatsSize]) {
                return const_cast<Data *>(&_data[0]);
            }
            return ptr;
        }

        size_t size() const {
            size_t count = 0;
            ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
                auto ptr = _head;
                while(ptr != _tail) {
                    count++;
                    ptr = next(ptr);
                }
            }
            return count;
        }

        // get record from data, interrupts must be disabled when being called
        Data _getData() {
            Data tmp;
            memcpy(&tmp, _head, sizeof(tmp));
            _head = next(_head);
            return tmp;
        }

    };

    #endif

public:
    // extra bits for the duty cycle
    static constexpr uint8_t kDutyCycleShift = 7;

public:
    PidController();

    // reset PID controller before starting
    void reset();

    // restore default PID settings
    void resetPidValues();
    // set PID values
    void setPidValues(const PidSettings &pid);
    // get PID values
    const PidSettings &getPidValues() const;
    // update single value of the PID settings
    void updatePidValue(PidConfigEnum pid, int16_t steps);
    // display PID values
    void printValues(Print &buffer) const;
    // display PID settings menu
    void displayPidSettingsMenu(PidConfigEnum highlight) const;

    #if HAVE_PID_CONTROLLER_STATS
        Stats _stats;
    #endif
    PidSettings _settings;

private:
    friend class RpmSense;
    friend class CurrentLimit;

    // this method will turn the motor on when called without checking anything
    void updateTicks(int32_t ticks);
    // get duty cycle of the PID controller
    uint8_t getDutyCycle() const;

private:
    float _integral;
    uint16_t _dutyCycle;
    int32_t _previousError;
    MicrosTimer _lastUpdate;
};

inline PidController::PidController() :
    _integral(0),
    _dutyCycle(0),
    _previousError(0)
    // _setPointRpm(0),
    // _setPointRpmTicks(0)
{
    resetPidValues();
}

inline void PidController::resetPidValues()
{
    _settings = PidSettings();
}

inline void PidController::setPidValues(const PidSettings &pid)
{
    _settings = pid;
}

inline const PidSettings &PidController::getPidValues() const
{
    return _settings;
}

extern PidController pid;
