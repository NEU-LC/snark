// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2020 Mission Systems Pty Ltd
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the copyright owner nor the names of the contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
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

#include <algorithm>
#include <stdexcept>
#include <vector>
#include <comma/string/split.h>
#include "echodyne.h"

namespace snark { namespace echodyne {

std::vector< status_data_t > from_packet( const MESAK_Status& packet )
{
    std::vector< status_data_t > data;
    data.push_back( status_data_t( packet.data ));
    return data;
}

std::vector< rvmap_data_t > from_packet( const MESAK_Rvmap& packet )
{
    std::vector< rvmap_data_t > data;
    data.push_back( rvmap_data_t( packet.header, packet.data ));
    return data;
}

std::vector< detection_data_t > from_packet( const MESAK_Detection& packet )
{
    std::vector< detection_data_t > data;
    for( auto& d : packet.data )
    {
        data.push_back( detection_data_t( packet.header, d ));
    }
    return data;
}

std::vector< track_data_t > from_packet( const MESAK_Track& packet )
{
    std::vector< track_data_t > data;
    for( auto& d : packet.data )
    {
        data.push_back( track_data_t( packet.header, d ));
    }
    return data;
}

std::vector< meas_data_t > from_packet( const MESAK_Measurement& packet )
{
    std::vector< meas_data_t > data;
    for( auto& d : packet.data )
    {
        data.push_back( meas_data_t( packet.header, d ));
    }
    return data;
}

// User manual section 7.3
const char* eth_speed_to_string( uint32_t eth_speed )
{
    switch( eth_speed )
    {
        case 0: return "1 Gb/s";
        case 1: return "100 Mb/s";
        case 2: return "10 Mb/s";
        default: return "Undefined";
    }
}

const char* mesa_command_status_to_string( mesa_command_status_t status )
{
    switch( status )
    {
        case MESA_OK: return "Success";
        case MESA_IC: return "Invalid Command";
        case MESA_CE: return "Command Execute Error";
        case MESA_IP: return "Invalid Password";
        case MESA_TO: return "Timeout";
        case MESA_NA: return "Unavailable";
        case MESA_UE: return "Unknown Error";
        default:      return "Undefined";
    }
}

// User manual section 7.3
const char* system_state_to_string( uint32_t state )
{
    switch( state )
    {
        case 0:  return "Reset";
        case 1:  return "Init";
        case 2:  return "Idle";
        case 3:  return "Command Executing";
        case 4:  return "Search";
        case 5:  return "SWT";
        case 6:  return "Error";
        case 7:  return "Upgrade";
        case 8:  return "Restart";
        case 9:  return "Interference Detection";
        default: return "Undefined";
    }
}

// User manual section 7.3
const char* time_channel_state_to_string( uint32_t state )
{
    switch( state )
    {
        case 0: return "Idle";
        case 1: return "Waiting";
        case 2: return "Searching";
        case 3: return "No Clear Time Channel";
        case 4: return "Clear Leader";    // this is the desired state for TCM ch0
        case 5: return "Locked Follower"; // this is the desired state for TCM ch1-3
        case 6: return "Lost Track Follower";
        case 7: return "TCM Error";
        default: return "Undefined";
    }
}

// User manual section 6.5.2.3
const char* track_state_to_string( uint32_t state )
{
    switch( state )
    {
        case 0: return "inactive";
        case 1: return "unconfirmed";
        case 2: return "confirmed";
        default: return "Undefined";
    }
}

mesa_data_t mesa_data_from_string( std::string mesa_data )
{
    // be fairly forgiving about what we accept
    std::transform( mesa_data.begin(), mesa_data.end(), mesa_data.begin(),
                  [](unsigned char c){ return std::tolower(c); });
    std::vector< std::string > words = comma::split( mesa_data, "_" );
    if(      mesa_data == "status" )      { return STATUS_DATA; }
    else if( mesa_data == "rvmap" )       { return RVMAP_DATA; }
    else if( mesa_data == "detection" )   { return DETECTION_DATA; }
    else if( mesa_data == "track" )       { return TRACK_DATA; }
    else if( mesa_data == "measurement" ) { return MEAS_DATA; }
    else throw std::invalid_argument( "invalid mesa data type \"" + mesa_data + "\"" );
}

} } // namespace snark { namespace echodyne {
