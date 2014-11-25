// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "data.h"
#include <iostream>

namespace snark { namespace ur { 

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

robotmode::mode get_robotmode(const std::string& mode)
{
    if( mode == "RN" ) { return robotmode::running; }
    else if( mode == "FD" ) { return robotmode::freedrive; }
    else if( mode == "RD" ) { return robotmode::ready; }
    else if( mode == "IN" ) { return robotmode::initializing; }
    else if( mode == "SS" ) { return robotmode::security_stopped; }
    else if( mode == "FE" ) { return robotmode::fatal_error; }
    else if( mode == "NP" ) { return robotmode::no_power; }
    else if( mode == "NC" ) { return robotmode::not_connected; }
    else if( mode == "SH" ) { return robotmode::shutdown; }
    else if( mode == "SG" ) { return robotmode::safeguard_stop; }
    else { COMMA_THROW( comma::exception, "unknown robot mode given: " << mode ); }

}

jointmode::mode get_jointmode(const std::string& mode)
{
    if( mode == "PO" ) { return jointmode::power_off; }
    else if( mode == "ER" ) { return jointmode::error; }
    else if( mode == "FD" ) { return jointmode::freedrive; }
    else if( mode == "CL" ) { return jointmode::calibration; }
    else if( mode == "SS" ) { return jointmode::stopped; }
    else if( mode == "RN" ) { return jointmode::running; }
    else if( mode == "IN" ) { return jointmode::initializing; }
    else if( mode == "ID" ) { return jointmode::idle; }
    else if( mode == "OT" ) { return jointmode::other; }
    else { COMMA_THROW( comma::exception, "unknown joint mode given: " << mode ); }
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

void status_t::set( boost::posix_time::ptime timestamp, const packet_t& packet )
{
    this->timestamp = timestamp;
    this->position.coordinates = Eigen::Vector3d( packet.translation.x(), packet.translation.y(), packet.translation.z() );
    this->position.orientation = Eigen::Vector3d( packet.rotation.x(), packet.rotation.y(), packet.rotation.z() );    
    this->robot_mode = static_cast< robotmode::mode >( packet.robot_mode() );
    this->length = packet.length();
    this->time_since_boot = packet.time_since_boot();    
    for( int i = 0; i < joints_num; ++i) 
    { 
        this->joint_angles[i] = packet.positions[i]() * radian; 
        this->velocities[i] = packet.velocities[i]();
        this->currents[i] = packet.currents[i]();
        this->forces[i] = packet.forces[i]();
        this->temperatures[i] = packet.temperatures[i]();
        this->joint_modes[i] = static_cast< jointmode::mode >( packet.joint_modes[i]() );
    }
}

} } // namespace snark { namespace ur { 
