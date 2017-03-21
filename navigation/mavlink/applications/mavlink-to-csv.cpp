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
// 3. Neither the name of the University of Sydney nor the
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

#include <iostream>
#include <vector>
#include "../../../math/angle.h"
#include "../c_library_v2/common/mavlink.h"

struct attitude_t
{
    unsigned int time_boot_ms;
    double roll;
    double pitch;
    double yaw;
    double rollspeed;
    double pitchspeed;
    double yawspeed;

    attitude_t( const mavlink_attitude_t &m )
        : time_boot_ms( m.time_boot_ms )
        , roll( m.roll )
        , pitch( m.pitch )
        , yaw( m.yaw )
        , rollspeed( m.rollspeed )
        , pitchspeed( m.pitchspeed )
        , yawspeed( m.yawspeed )
        {}
};

struct global_position_t
{
    unsigned int time_boot_ms;
    double latitude;
    double longitude;
    double altitude;
    double relative_altitude;
    double vx;
    double vy;
    double vz;
    double heading;

    global_position_t( const mavlink_global_position_int_t &m )
        : time_boot_ms( m.time_boot_ms )
        , latitude( static_cast< double >( m.lat ) / 1e7 )
        , longitude( static_cast< double >( m.lon ) / 1e7 )
        , altitude( static_cast< double >( m.alt ) / 1e3 )
        , relative_altitude( static_cast< double >( m.relative_alt ) / 1e3 )
        , vx( static_cast< double >( m.vx ) / 100 )
        , vy( static_cast< double >( m.vy ) / 100 )
        , vz( static_cast< double >( m.vz ) / 100 )
        , heading( snark::math::radians( snark::math::degrees( static_cast< double >( m.hdg ) / 100 ) ).value ) // todo: handle unknown value
    {}
};

int main( int argc, char *argv[] )
{
    while( std::cin.good() )
    {
        char c;
        std::cin.read( &c, 1 );
        mavlink_message_t message;
        mavlink_status_t status;
        if( mavlink_parse_char( MAVLINK_COMM_0, static_cast< unsigned char >( c ), &message, &status ) )
        {
            //std::cout << "sysid: " << (int)message.sysid << ", compid: " << (int)message.compid << ", msgid: " << message.msgid << ", len: " << (int)message.len << std::endl;
            switch( message.msgid )
            {
                case MAVLINK_MSG_ID_ATTITUDE:
                    {
                        mavlink_attitude_t m;
                        mavlink_msg_attitude_decode( &message, &m );
                        attitude_t attitude( m );
                        std::cout << "    [attitude] time_boot_ms: " << attitude.time_boot_ms
                                  << ", roll: " << attitude.roll
                                  << ", pitch: " << attitude.pitch
                                  << ", yaw: " << attitude.yaw
                                  << ", rollspeed: " << attitude.rollspeed
                                  << ", pitchspeed: " << attitude.pitchspeed
                                  << ", yawspeed: " << attitude.yawspeed
                                  << std::endl;
                    }
                    break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                    {
                        mavlink_global_position_int_t m;
                        mavlink_msg_global_position_int_decode( &message, &m );
                        global_position_t global_position( m );
                        std::cout << "    [global_position] time_boot_ms: " << global_position.time_boot_ms
                                  << ", longitude: " << global_position.longitude
                                  << ", latitude: " << global_position.latitude
                                  << ", altitude: " << global_position.altitude
                                  << ", relative_altitude: " << global_position.relative_altitude
                                  << ", vx: " << global_position.vx
                                  << ", vy: " << global_position.vy
                                  << ", vz: " << global_position.vz
                                  << ", heading: " << global_position.heading
                                  << std::endl;
                    }
                    break;
                default:
                    break;
            }
        }
    }
    return 0;
}
