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

void fixed_status::get_angles(boost::array< plane_angle_t, 6 >& angles)
{
    angles[0] = this->positions[0]() * radian;
    angles[1] = this->positions[1]() * radian;
    angles[2] = this->positions[2]() * radian;
    angles[3] = this->positions[3]() * radian;
    angles[4] = this->positions[4]() * radian;
    angles[5] = this->positions[5]() * radian;
}


} } } // namespace snark { namespace ur { namespace robotic_arm { 
