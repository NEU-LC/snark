#include "data.h"
#include <iostream>

namespace snark { namespace ur { namespace robotic_arm { 

const char* robotmode_str( robotmode::mode mode )
{
    switch( mode )
    {
        case robotmode::running:
            return "RN"; 
        case robotmode::freedrive:
            return "FD";
        case robotmode::ready:
            return "RD";
        case robotmode::initializing:
            return "IN";
        case robotmode::security_stopped:
            return "SS";
        case robotmode::estopped:
            return "ES";
        case robotmode::fatal_error:
            return "FE";
        case robotmode::no_power:
            return "NP";
        case robotmode::not_connected:
            return "NC";
        case robotmode::shutdown:
            return "SH";
        case robotmode::safeguard_stop:
            return "SG";
        default:
            std::cerr << "unknown robot mode: " << int(mode) << std::endl;
            COMMA_THROW( comma::exception, "unknown robot mode" );
            return "UN";
    }
}

const char* jointmode_str( jointmode::mode mode )
{
    switch( mode )
    {
        case jointmode::power_off:
            return "PO";
        case jointmode::error:
            return "ER";
        case jointmode::freedrive:
            return "FD";
        case jointmode::calibration:
            return "CL";
        case jointmode::stopped:
            return "SS";
        case jointmode::running:
            return "RN";
        case jointmode::initializing:
            return "IN";
        case jointmode::idle:
            return "ID";
        default:
            return "OT"; // some other mode not converted to string
    }
}

} } } // namespace snark { namespace ur { namespace robotic_arm { 
