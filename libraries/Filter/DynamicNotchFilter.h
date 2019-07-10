/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <cmath>
#include <inttypes.h>
#include <AP_Param/AP_Param.h>
#include "NotchFilter.h"


template <class T>
class DynamicNotchFilter {
public:
    // set parameters
    void create(uint8_t harmonics);
    void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB, uint8_t scaled_notches);
    T apply(const T &sample);
    void reset();
    void update(float center_freq_hz, uint8_t scaled_notches);

private:
    NotchFilter<T>*  filters;
    float sample_freq_hz;
    float bandwidth_hz;
    float attenuation_dB;
    uint8_t harmonics;
};

/*
  notch filter enable and filter parameters
 */
class DynamicNotchFilterParams : public NotchFilterParams {
public:
    DynamicNotchFilterParams(void);
    void set_center_freq_hz(uint16_t center_freq_hz) { _center_freq_hz.set(center_freq_hz); }
    uint8_t harmonics(void) const { return _harmonics; }
    uint16_t min_freq_hz(void) const { return _min_freq_hz; }
    uint16_t max_freq_hz(void) const { return _max_freq_hz; }
    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Int8 _harmonics;
    AP_Int16 _min_freq_hz;
    AP_Int16 _max_freq_hz;
};

typedef DynamicNotchFilter<float> DynamicNotchFilterFloat;
typedef DynamicNotchFilter<Vector3f> DynamicNotchFilterVector3f;

