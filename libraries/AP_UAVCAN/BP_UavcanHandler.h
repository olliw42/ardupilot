//******************************************************
//OW
// (c) olliw, www.olliw.eu, GPL3
//******************************************************
// Template class to handle backend calls via their frontend singleton

#pragma once


#define BP_UAVCAN_HANDLER_MAX_INSTANCES  16

template <class T>
class BP_Uavcan_Handler
{
public:
    bool add(T* uavcan_backend)
    {
        if (!uavcan_backend || (_num_uavcan_instances >= BP_UAVCAN_HANDLER_MAX_INSTANCES)) return false;
        _uavcan_handler[_num_uavcan_instances++] = uavcan_backend;
        return true;
    };

    uint8_t num(void){ return _num_uavcan_instances; }
    bool is_ok(uint8_t i){ return ((i < _num_uavcan_instances) && (_uavcan_handler[i] != nullptr)); }
    T* call(uint8_t i){ return _uavcan_handler[i]; } //no check for i in range since this MUTS be called only in safe cases !!!

private:
    uint8_t _num_uavcan_instances;
    T* _uavcan_handler[BP_UAVCAN_HANDLER_MAX_INSTANCES];
};

#define FOREACH_UAVCAN_HANDLER(H,x)  for(uint8_t i=0; i<H.num(); i++){ if (H.is_ok(i)) H.call(i)->x; }
