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
/*
 * AP_EFI_NWPMU.cpp
 *
 *      Author: Francisco Ferreira
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>

#include <AP_Common/AP_Common.h>

#include <AP_HAL/utility/sparse-endian.h>

#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Math/AP_Math.h>

#include "AP_EFI_NWPMU.h"

#if HAL_WITH_EFI_NMPMU

extern const AP_HAL::HAL& hal;

#define debug_can(level_debug, fmt, args...) do { if ((level_debug) <= AP::can().get_debug_level_driver(_driver_index)) { printf(fmt, ##args); }} while (0)

AP_EFI_NWPMU::AP_EFI_NWPMU(AP_EFI &_frontend) :
    AP_EFI_Backend(_frontend)
{
    debug_can(2, "NWPMU: constructed\n");
}

void AP_EFI_NWPMU::init(uint8_t driver_index, bool enable_filters)
{
    _driver_index = driver_index;

    debug_can(2, "NWPMU: starting init\n");

    if (_initialized) {
        debug_can(1, "NWPMU: already initialized\n");
        return;
    }

    // get CAN manager instance
    AP_HAL::CANManager* can_mgr = hal.can_mgr[driver_index];

    if (can_mgr == nullptr) {
        debug_can(1, "NWPMU: no mgr for this driver\n");
        return;
    }

    if (!can_mgr->is_initialized()) {
        debug_can(1, "NWPMU: mgr not initialized\n");
        return;
    }

    // store pointer to CAN driver
    _can_driver = can_mgr->get_driver();

    if (_can_driver == nullptr) {
        debug_can(1, "NWPMU: no CAN driver\n");
        return;
    }

    snprintf(_thread_name, sizeof(_thread_name), "nwpmu_%u", driver_index);

    // start thread for receiving and sending CAN frames
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_EFI_NWPMU::loop, void), _thread_name, 4096, AP_HAL::Scheduler::PRIORITY_CAN, 0)) {
        debug_can(1, "NWPMU: couldn't create thread\n");
        return;
    }

    _initialized = true;

    debug_can(2, "NWPMU: init done\n");

    return;
}

void AP_EFI_NWPMU::loop()
{
    uavcan::MonotonicTime timeout;
    uavcan::CanFrame empty_frame { (0 | uavcan::CanFrame::FlagEFF), nullptr, 0 };
    const uavcan::CanFrame* select_frames[uavcan::MaxCanIfaces] { };
    select_frames[CAN_IFACE_INDEX] = &empty_frame;

    const uint32_t LOOP_INTERVAL_US = AP::scheduler().get_loop_period_us();
    while (true) {
        uavcan::CanSelectMasks inout_mask;
        uint64_t now = AP_HAL::micros64();

        // always look for received frames
        inout_mask.read = 1 << CAN_IFACE_INDEX;
        timeout = uavcan::MonotonicTime::fromUSec(now + LOOP_INTERVAL_US);

        // wait to receive frame
        uavcan::CanSelectMasks in_mask = inout_mask;
        _can_driver->select(inout_mask, select_frames, timeout);

        if (in_mask.read & inout_mask.read) {
            uavcan::CanFrame frame;
            uavcan::MonotonicTime time;
            uavcan::UtcTime utc_time;
            uavcan::CanIOFlags flags {};

            int16_t res = _can_driver->getIface(CAN_IFACE_INDEX)->receive(frame, time, utc_time, flags);

            if (res == 1) {
                const uint32_t id =  frame.id & uavcan::CanFrame::MaskExtID;

                switch ((NWPMU_ID)id) {
                    case NWPMU_ID::ECU_1:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            struct ecu_1 *data = (struct ecu_1 *)frame.data;
                            internal_state.engine_speed_rpm = data->rpm;
                            internal_state.throttle_position_percent = data->tps * 0.1f;
                            internal_state.cylinder_status[0].ignition_timing_deg = data->ignition_angle * 0.1f;
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for ECU 1");
                        }
                        break;
                    case NWPMU_ID::ECU_2:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            struct ecu_2 *data = (struct ecu_2 *)frame.data;
                            switch ((NWPMU_PRESSURE_TYPE)data->pressure_type) {
                                case NWPMU_PRESSURE_TYPE::kPa:
                                    internal_state.atmospheric_pressure_kpa = data->baro * 0.01f;
                                    internal_state.intake_manifold_pressure_kpa = data->baro * 0.01f;
                                    break;
                                case NWPMU_PRESSURE_TYPE::psi:
                                    internal_state.atmospheric_pressure_kpa = data->baro * 0.0689476f;
                                    internal_state.intake_manifold_pressure_kpa = data->baro * 0.0689476f;
                                    break;
                                default:
                                    debug_can(1, "Unknown pressure type %d", data->pressure_type);
                                    break;
                            }
                            internal_state.cylinder_status[0].lambda_coefficient = data->lambda * 0.01f;
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for ECU 2");
                        }
                        break;
                    case NWPMU_ID::ECU_4:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            struct ecu_4 *data = (struct ecu_4 *)frame.data;
                            // remap the analog input for fuel pressure, 0.5 V == 0 PSI, 4.5V == 100 PSI
                            internal_state.fuel_pressure = linear_interpolate(0, 689.476,
                                                                              data->analog_fuel_pres * 0.001,
                                                                              0.5f,4.5f);
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for ECU 4");
                        }
                        break;
                    case NWPMU_ID::ECU_5:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            struct ecu_5 *data = (struct ecu_5 *)frame.data;
                            switch((NWPMU_TEMPERATURE_TYPE)data->temp_type) {
                                case NWPMU_TEMPERATURE_TYPE::C:
                                    internal_state.coolant_temperature = data->coolant_temp * 0.1f + C_TO_KELVIN;
                                    internal_state.cylinder_status[0].cylinder_head_temperature = data->coolant_temp * 0.1f + C_TO_KELVIN;
                                    break;
                                case NWPMU_TEMPERATURE_TYPE::F:
                                    internal_state.coolant_temperature = ((data->coolant_temp * 0.1f) - 32 * 5/9) + C_TO_KELVIN;
                                    internal_state.cylinder_status[0].cylinder_head_temperature = ((data->coolant_temp * 0.1f) - 32 * 5/9) + C_TO_KELVIN;
                                    break;
                                default:
                                    debug_can(1, "Unknown temperature type for ECU 5\n");
                            }
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for ECU 5");
                        }
                        break;
                    case NWPMU_ID::ECU_6:
                        if (sem.take(1)) {
                            internal_state.last_updated_ms = AP_HAL::millis();
                            struct ecu_6 *data = (struct ecu_6 *)frame.data;
                            debug_can(2, "Volts %f %f\n", data->analog_5 * 0.1f, data->analog_7 *0.1f);
                            if (!_emitted_version && (AP_HAL::millis() > 10000)) { // don't emit a version early in the boot process
                                gcs().send_text(MAV_SEVERITY_INFO, "NWPMU Version: %d.%d.%d",
                                                                   data->firmware_major,
                                                                   data->firmware_minor,
                                                                   data->firmware_build);
                                _emitted_version = true;
                            }
                            sem.give();
                        } else {
                            debug_can(2, "Failed to acquire the lock for ECU 6");
                        }
                        break;
                    case NWPMU_ID::GCU:
                    case NWPMU_ID::ECU_3:
                    case NWPMU_ID::ECU_7:
                    case NWPMU_ID::ECU_8:
                    case NWPMU_ID::ECU_9:
                    case NWPMU_ID::ECU_10:
                    case NWPMU_ID::ECU_11:
                    case NWPMU_ID::ECU_12:
                        break;
                }
            }
        }

    }
}

void AP_EFI_NWPMU::update()
{
    // copy the data to the front end
    copy_to_frontend();

}

AP_EFI_NWPMU *AP_EFI_NWPMU::get_singleton(uint8_t driver_index) {
    if (driver_index >= AP::can().get_num_drivers() ||
        AP::can().get_protocol_type(driver_index) != AP_BoardConfig_CAN::Protocol_Type_NWPMU) {
        return nullptr;
    }
    return static_cast<AP_EFI_NWPMU*>(AP::can().get_driver(driver_index));
}

#endif
#endif // HAL_WITH_UAVCAN
