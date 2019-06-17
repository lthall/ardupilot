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

#include "DynamicNotchFilter.h"

/*
  initialise filter
 */
template <class T>
void DynamicNotchFilter<T>::init(float _sample_freq_hz, float _center_freq_hz, float _bandwidth_hz, float _attenuation_dB)
{
    sample_freq_hz = _sample_freq_hz;
    bandwidth_hz = _bandwidth_hz;
    attenuation_dB = _attenuation_dB;
    for (int i=0; i<harmonics+1; i++) {
        filters[i].init(sample_freq_hz, _center_freq_hz * (i+1), bandwidth_hz, attenuation_dB);
    }
}

/*
  initialise filter
 */
template <class T>
void DynamicNotchFilter<T>::create(uint8_t _harmonics)
{
    filters = new NotchFilter<T>[_harmonics+1];
    harmonics = _harmonics;
}

template <class T>
void DynamicNotchFilter<T>::update(float center_freq_hz)
{
    if (filters == nullptr) {
        return;
    }

    for (int i=0; i<harmonics+1; i++) {
        filters[i].init(sample_freq_hz, center_freq_hz * (i+1), bandwidth_hz, attenuation_dB);
    }
}

template <class T>
T DynamicNotchFilter<T>::apply(const T &sample)
{
    if (filters == nullptr) {
        return sample;
    }

    T output = filters[0].apply(sample);
    for (int i=1; i<harmonics+1; i++) {
        output = filters[i].apply(output);
    }
    return output;
}

template <class T>
void DynamicNotchFilter<T>::reset()
{
    if (filters == nullptr) {
        return;
    }

    for (int i=0; i<harmonics+1; i++) {
        filters[i].reset();
    }
}

// table of user settable parameters
const AP_Param::GroupInfo DynamicNotchFilterParams::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable
    // @Description: Enable notch filter
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, DynamicNotchFilterParams, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: BW
    // @DisplayName: Bandwidth
    // @Description: Notch bandwidth in Hz
    // @Range: 5 100
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("BW", 2, DynamicNotchFilterParams, _bandwidth_hz, 20),

    // @Param: ATT
    // @DisplayName: Attenuation
    // @Description: Notch attenuation in dB
    // @Range: 5 30
    // @Units: dB
    // @User: Advanced
    AP_GROUPINFO("ATT", 3, DynamicNotchFilterParams, _attenuation_dB, 15),

    // @Param: HMNCS
    // @DisplayName: Harmonics
    // @Description: Number of harmonic frequencies to add to the filter. This option takes effect on the next reboot.
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("HMNCS", 4, DynamicNotchFilterParams, _harmonics, 1),

    // @Param: MINHZ
    // @DisplayName: Minimum Frequency
    // @Description: Notch minimum`center frequency in Hz
    // @Range: 10 400
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MINHZ", 5, DynamicNotchFilterParams, _min_freq_hz, 80),

    // @Param: MAXHZ
    // @DisplayName: Maximum Frequency
    // @Description: Notch maximum`center frequency in Hz
    // @Range: 200 800
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("MAXHZ", 6, DynamicNotchFilterParams, _max_freq_hz, 400),
    
    AP_GROUPEND
};

/*
  a notch filter with enable and filter parameters - constructor
 */
DynamicNotchFilterParams::DynamicNotchFilterParams(void)
{
    AP_Param::setup_object_defaults(this, var_info);    
}

/* 
   instantiate template classes
 */
template class DynamicNotchFilter<float>;
template class DynamicNotchFilter<Vector3f>;
