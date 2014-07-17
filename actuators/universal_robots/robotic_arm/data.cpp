#include "data.h"
#include <iostream>

namespace snark { namespace ur { namespace robotic_arm { 

const char* robotmode_str( robotmode::mode mode )
{
    switch( mode )
    {
        case robotmode::running:
            return "running_mode";
        case robotmode::freedrive:
            return "freedrive_mode";
        case robotmode::ready:
            return "ready_mode";
        case robotmode::initializing:
            return "initializing_mode";
        case robotmode::security_stopped:
            return "security_stopped_mode";
        case robotmode::estopped:
            return "estopped_mode";
        case robotmode::fatal_error:
            return "fatal_error_mode";
        case robotmode::no_power:
            return "no_power_mode";
        case robotmode::not_connected:
            return "not_connected_mode";
        case robotmode::shutdown:
            return "shutdown_mode";
        case robotmode::safeguard_stop:
            return "safeguard_stop_mode";
        default:
            std::cerr << "unknown robot mode: " << int(mode) << std::endl;
            COMMA_THROW( comma::exception, "unknown robot mode" );
            return "";
    }
}

const char* jointmode_str( jointmode::mode mode )
{
    switch( mode )
    {
        case jointmode::power_off:
            return "power_off_mode";
        case jointmode::error:
            return "error_mode";
        case jointmode::freedrive:
            return "freedrive_mode";
        case jointmode::calibration:
            return "calibration_mode";
        case jointmode::stopped:
            return "stopped_mode";
        case jointmode::running:
            return "running_mode";
        case jointmode::initializing:
            return "initializing_mode";
        case jointmode::idle:
            return "idle_mode";
        default:
            return "other_mode"; // some other mode not converted to string
    }
}

} } } // namespace snark { namespace ur { namespace robotic_arm { 
