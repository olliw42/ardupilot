//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_UAVCAN

#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_BoardConfig/AP_BoardConfig_CAN.h>
#include <GCS_MAVLink/GCS.h>

#include "BP_RangeFinder_UC4H.h"

extern const AP_HAL::HAL& hal;

#define debug_rf_uavcan(level, fmt, args...) do { if ((level) <= AP_BoardConfig_CAN::get_can_debug()) { printf(fmt, ##args); }} while (0)


BP_RangeFinder_UC4H::BP_RangeFinder_UC4H(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params) :
    AP_RangeFinder_Backend(_state, _params)
{
    _our_id = UINT32_MAX;
    _initialized = false;
    _send_banner = false;
    _new_distance_received = false;

    state.last_reading_ms = 0;

    _my_sem = hal.util->new_semaphore();
}


bool BP_RangeFinder_UC4H::init()
{
    _our_id = _calc_id(params.orientation.get(), params.address.get());

    if (_our_id == UINT32_MAX) {
        return false;
    }
    return true;
}


//TODO: we could sum up the data, similar to Benewake
void BP_RangeFinder_UC4H::update(void)
{
    _do_send_banner();

    if (!_initialized) {
        return;
    }

    if (!_new_distance_received) {
        return;
    }
    _new_distance_received = false;

    if (_my_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {

        uint32_t now_ms = AP_HAL::millis();

        if (_range_flag != UC4HDISTANCE_RANGE_INVALID) {
            state.distance_cm = (uint16_t)(_range * 100.0f);
            state.last_reading_ms = now_ms;
            update_status();
        } else if ((now_ms - state.last_reading_ms) > 200) {
            set_status(RangeFinder::RangeFinder_NoData);
        }

        _my_sem->give();
    }
}


void BP_RangeFinder_UC4H::send_banner(uint8_t instance)
{
    _instance = instance;
    _send_banner = true;
}


void BP_RangeFinder_UC4H::_do_send_banner(void)
{
    if (_send_banner) {
        _send_banner = false;

        if (!_initialized) {
            gcs().send_text(MAV_SEVERITY_INFO, "RangeFinder %u: UC4H %u %i %i %u notdet", _instance+1,
                    _node_id,
                    _calc_pitch_from_id(_our_id) * 15,
                    _calc_yaw_from_id(_our_id) * 15,
                    _calc_subid_from_id(_our_id)
                    );
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "RangeFinder %u: UC4H %u %i %i %u", _instance+1,
                    _node_id, //BUG: _node id is not yet known!!!
                    _calc_pitch_from_id(_our_id) * 15,
                    _calc_yaw_from_id(_our_id) * 15,
                    _calc_subid_from_id(_our_id)
                    );
        }
    }
}


void BP_RangeFinder_UC4H::handle_uc4hdistance_msg(uint32_t ext_id, int8_t fixed_axis_pitch, int8_t fixed_axis_yaw, uint8_t sensor_sub_id, uint8_t range_flag, float range)
{
    if ((_our_id == UINT32_MAX) || ((ext_id & 0x00FFFFFF) != _our_id)) {
        return;
    }

    if (!_initialized) {
        _initialized = true;
        _node_id = (uint8_t)((ext_id & 0xFF000000) >> 24);
    }

    _range_flag = range_flag;

    switch( range_flag ){
    case UC4HDISTANCE_RANGE_INVALID:
        //skip the data
        break;
    case UC4HDISTANCE_RANGE_VALID:
        _range = range;
        break;
    case UC4HDISTANCE_RANGE_TOOCLOSE:
        _range = 0.02f; //use something very small, so that AP_RangeFinder_Backend's update_status() can do the job
        break;
    case UC4HDISTANCE_RANGE_TOOFAR:
        //_range = 100.00f;  //for the moment use something very large, so that AP_RangeFinder_Backend's update_status() can do the job
        _range = (max_distance_cm() + 50) * 0.01f; //make it larger so that AP_RangeFinder_Backend's update_status() triggers
        break;
    default:
        //skip the data
        _range_flag = UC4HDISTANCE_RANGE_INVALID; //set this so we can handle it alike
        break;
    }

    _new_distance_received = true; //we set this even if the data was bad, bad data is handled by update()
}


void BP_RangeFinder_UC4H::handle_uc4hdistance_msg_sensorproperties(uint32_t ext_id, float range_min, float range_max, float vertical_field_of_view, float horizontal_field_of_view)
{

}


uint32_t BP_RangeFinder_UC4H::_calc_id(int8_t pitch, int8_t yaw, uint8_t sub_id)
{
    return ((uint32_t)pitch & 0x000000FF) + ( ((uint32_t)yaw & 0x000000FF) << 8 ) + ( ((uint32_t)sub_id & 0x000000FF) << 16 );
}

// 0:Forward, 1:Forward-Right, 2:Right, 3:Back-Right, 4:Back, 5:Back-Left, 6:Left, 7:Forward-Left, 24:Up, 25:Down
uint32_t BP_RangeFinder_UC4H::_calc_id(uint8_t orient, uint8_t sub_id)
{
int8_t pitch, yaw; //the angles are in integer, and 15° steps, i.e. 90° = 6, 180° = 12, -90° = -6

    switch( orient ){
    case ROTATION_NONE:     pitch = 0; yaw = 0; break; //Forward
    case ROTATION_YAW_45:   pitch = 0; yaw = 3; break; //Forward-Right
    case ROTATION_YAW_90:   pitch = 0; yaw = 6; break; //Right
    case ROTATION_YAW_135:  pitch = 0; yaw = 9; break; //Back-Right
    case ROTATION_YAW_180:  pitch = 0; yaw = 12; break; //Back
    case ROTATION_YAW_225:  pitch = 0; yaw = -9; break; //Back-Left
    case ROTATION_YAW_270:  pitch = 0; yaw = -6; break; //Left
    case ROTATION_YAW_315:  pitch = 0; yaw = -3; break; //Forward-Left
//    case ROTATION_ROLL_180:                     pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_45:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_90:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_135:             pitch = 0; yaw = 0; break;
//    case ROTATION_PITCH_180:                    pitch = 0; yaw = 12; break;
//    case ROTATION_ROLL_180_YAW_225:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_270:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_YAW_315:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90:                      pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_45:               pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_90:               pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_135:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270:                     pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_YAW_45:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_YAW_90:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_YAW_135:             pitch = 0; yaw = 0; break;
    case ROTATION_PITCH_90:     pitch = 6; yaw = 0; break; //Up
    case ROTATION_PITCH_270:    pitch = -6; yaw = 0; break; //Down
//    case ROTATION_PITCH_180_YAW_90:             pitch = 0; yaw = 0; break;
//    case ROTATION_PITCH_180_YAW_270:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_90:             pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_PITCH_90:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_PITCH_90:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_180:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_PITCH_180:           pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_270:            pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_180_PITCH_270:           pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_270_PITCH_270:           pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_180_YAW_90:     pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_YAW_270:              pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_68_YAW_293:     pitch = 0; yaw = 0; break;
//    case ROTATION_PITCH_315:                    pitch = 0; yaw = 0; break;
//    case ROTATION_ROLL_90_PITCH_315:            pitch = 0; yaw = 0; break;
//    case ROTATION_MAX:                          pitch = 0; yaw = 0; break;
//    case ROTATION_CUSTOM:                       pitch = 0; yaw = 0; break;
    default:
        return UINT32_MAX; //don't accept
    };

    return _calc_id(pitch, yaw, sub_id);
}


int8_t BP_RangeFinder_UC4H::_calc_pitch_from_id(uint32_t id)
{
    return (int8_t)(id & 0x000000FF);
}

int8_t BP_RangeFinder_UC4H::_calc_yaw_from_id(uint32_t id)
{
    return (int8_t)((id >> 8) & 0x000000FF);
}

uint8_t BP_RangeFinder_UC4H::_calc_subid_from_id(uint32_t id)
{
    return (uint8_t)((id >> 16) & 0x000000FF);
}

#endif

