/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

#include <Arduino.h>

class UIConfigData {
public:
    static constexpr int16_t kDefaultTimeout = DISPLAY_REFRESH_TIME;
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
    uint16_t getDutyCycle() const {
        return display_duty_cycle_integral;
    }

private:
    void _setRefreshTimeout(int16_t timeout);

    inline uint16_t _getTimeout() const {
        return static_cast<uint16_t>(_timeout < 0 ? -_timeout : _timeout);
    }

    uint32_t _refreshTimer;
    int16_t _timeout;
    float display_duty_cycle_integral;
    float display_pulse_length_integral;
    uint16_t _updateUITimer;
};

inline UIConfigData::UIConfigData() :
    _refreshTimer(0),
    _timeout(kDefaultTimeout),
    display_duty_cycle_integral(0),
    display_pulse_length_integral(RPM_SENSE_RPM_TO_TICKS(1))
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

