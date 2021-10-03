/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

class EEPROMData {
public:
    uint32_t magic;
    ControlModeEnum control_mode;
    uint16_t set_point_rpm;
    uint8_t set_point_pwm;
    uint8_t _ledBrightness;
    uint8_t current_limit;
    uint8_t brake_enabled;
    uint16_t max_stall_time;
    PidSettings pid_settings;
    uint8_t max_pwm;
    uint16_t rpm_per_volt;
    MotorStatusEnum _motorStatus;

    constexpr size_t size() const {
        return sizeof(*this);
    }

    bool operator!=(const EEPROMData &data) const;
    EEPROMData &operator=(const ConfigData &data);
};

inline bool EEPROMData::operator!=(const EEPROMData &data) const
{
    return memcmp(this, &data, size());
}
