/**
 * Author: sascha_lammers@gmx.de
 */

#pragma once

struct PidSettings
{
    float Kp;
    float Ki;
    float Kd;

    PidSettings() :
        Kp(5.0),
        Ki(0.0),
        Kd(0.0)
    {
    }

    PidSettings(float aKp, float aKi, float aKd) :
        Kp(aKp),
        Ki(aKi),
        Kd(aKd)
    {
    }

    float *begin() {
        return &Kp;
    }

    constexpr const float *begin() const {
        return &Kp;
    }

    constexpr const float *end() const {
        return &Kd + 1;
    }

    float &operator[](PidConfigEnum index) {
        switch(index) {
            case PidConfigEnum::KP:
                return Kp;
            case PidConfigEnum::KI:
                return Ki;
            case PidConfigEnum::KD:
                return Kd;
            default:
                break;
        }
        return *begin();
    }

    const float &operator[](PidConfigEnum index) const {
        return const_cast<PidSettings *>(this)->operator[](index);
    }

    float &operator[](uint8_t index) {
        return this->operator[](static_cast<PidConfigEnum>(index));
    }
};
