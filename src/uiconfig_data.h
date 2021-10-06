/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

class UIConfigData {
public:
    static constexpr int16_t kDefaultTimeout = Timeouts::Display::kRefresh;
public:
    UIConfigData();

    // refresh display now
    void refreshDisplay();
    // refresh display every timeout milliseconds
    void setRefreshTimeout(uint16_t timeout) {
        _setRefreshTimeout(timeout);
    }
    // refresh display after timeout millis
    void setRefreshTimeoutOnce(uint16_t timeout) {
        _setRefreshTimeout(-timeout);
    }
    // disable auto refresh
    void disableRefreshDisplay();
    // returns true to refresh the display
    bool doRefreshDisplay() const;

    void updateTimer();

    bool isTimeoutOnce() const;

    // calculate values for UI
    void updateRpmAndDutyCycle();

    void loop();

    // get RPM for UI
    uint16_t getRpm() const;

    // get averaged duty cycle
    uint8_t getDutyCycle() const {
        return _dutyCycleAvg;
    }

private:
    void _setRefreshTimeout(int16_t timeout);

    inline uint16_t _getTimeout() const {
        return static_cast<uint16_t>(_timeout < 0 ? -_timeout : _timeout);
    }

    uint32_t _refreshTimer;
    int16_t _timeout;
    uint8_t _dutyCycleAvg;
    uint16_t _displayRpm;
    uint16_t _updateUITimer;
};

inline UIConfigData::UIConfigData() :
    _refreshTimer(0),
    _timeout(kDefaultTimeout),
    _dutyCycleAvg(0),
    _displayRpm(0)
{
}

inline void UIConfigData::refreshDisplay()
{
    _refreshTimer = 0;
}

inline void UIConfigData::_setRefreshTimeout(int16_t timeout)
{
    _refreshTimer = millis();
    if (!_refreshTimer) {
        _refreshTimer++;
    }
    _timeout = timeout;
}

inline void UIConfigData::disableRefreshDisplay()
{
    _refreshTimer = 1;
    _timeout = 0;
}

inline void UIConfigData::updateTimer()
{
    _refreshTimer = millis();
    if (_timeout < 0) {
        // setRefreshTimeoutOnce was used, reset to default after one refresh
        _timeout = kDefaultTimeout;
    }
}

inline bool UIConfigData::isTimeoutOnce() const
{
    return _timeout < 0;
}

inline bool UIConfigData::doRefreshDisplay() const
{
    return _refreshTimer == 0 ? true : (_timeout ? ((millis() - _refreshTimer) >= _getTimeout()) : false);
}

