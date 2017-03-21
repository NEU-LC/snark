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
#include <Eigen/Core>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../../../math/angle.h"
#include "../../../math/spherical_geometry/coordinates.h"
#include "../../../math/spherical_geometry/traits.h"
#include "../../../visiting/eigen.h"
#include "../c_library_v2/common/mavlink.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "take mavlink input on stdin, output as csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat mavlink.bin | mavlink-to-csv [<options>] > mavlink.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --strict: exit on parsing errors" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "binary options:" << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; } else { std::cerr << "    run mavlink-to-csv --help --verbose for more..." << std::endl; }
    std::cerr << std::endl;
    exit( 0 );
}

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

namespace snark { namespace mavlink {

struct orientation
{
    double roll;
    double pitch;
    double yaw;
    orientation() : roll( 0 ), pitch( 0 ), yaw( 0 ) {}
};
    
struct pose : public Eigen::Vector3d, public orientation { pose(): Eigen::Vector3d( Eigen::Vector3d::Zero() ) {} };

struct output : public snark::spherical::coordinates, public orientation
{
    boost::posix_time::ptime t;
    comma::uint32 milliseconds_from_boot;
    double altitude;
    double relative_altitude;
    pose velocity;
};

} } // namespace snark { namespace mavlink {

namespace comma { namespace visiting {

template <> struct traits< snark::mavlink::orientation >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const snark::mavlink::orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
};

template <> struct traits< snark::mavlink::pose >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::mavlink::pose& t, Visitor& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        traits< snark::mavlink::orientation >::visit( k, t, v );
    }
};

template <> struct traits< snark::mavlink::output >
{
    template < typename Key, class Visitor >
    static void visit( const Key& k, const snark::mavlink::output& t, Visitor& v )
    {
        v.apply( "t", t.t );
        v.apply( "milliseconds_from_boot", t.milliseconds_from_boot );
        traits< snark::spherical::coordinates >::visit( k, t, v );
        v.apply( "altitude", t.altitude );
        v.apply( "relative_altitude", t.relative_altitude );
        traits< snark::mavlink::orientation >::visit( k, t, v );
        v.apply( "velocity", t.velocity );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char *av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::mavlink::output >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< snark::mavlink::output >() << std::endl; return 0; }
        bool strict = options.exists( "--strict" );
        bool verbose = options.exists( "--verbose,-v" );
        comma::csv::options csv( options );
        csv.full_xpath = true;
        comma::csv::output_stream< snark::mavlink::output > ostream( std::cout, csv ); // todo: output only on change
        snark::mavlink::output output;
        while( std::cin.good() )
        {
            char c;
            std::cin.read( &c, 1 );
            mavlink_message_t message;
            mavlink_status_t status;
            output.t = boost::posix_time::microsec_clock::universal_time();
            if( mavlink_parse_char( MAVLINK_COMM_0, static_cast< unsigned char >( c ), &message, &status ) )
            {
                switch( message.msgid )
                {
                    case MAVLINK_MSG_ID_ATTITUDE:
                        {
                            mavlink_attitude_t m;
                            mavlink_msg_attitude_decode( &message, &m );
                            attitude_t attitude( m );
                            output.milliseconds_from_boot = attitude.time_boot_ms;
                            output.roll = attitude.roll;
                            output.pitch = attitude.pitch;
                            output.yaw = attitude.yaw;
                            output.velocity.roll = attitude.rollspeed;
                            output.velocity.pitch = attitude.pitchspeed;
                            output.velocity.yaw = attitude.yawspeed;
                        }
                        break;
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                        {
                            mavlink_global_position_int_t m;
                            mavlink_msg_global_position_int_decode( &message, &m );
                            global_position_t global_position( m );
                            output.milliseconds_from_boot = global_position.time_boot_ms;
                            output.latitude = global_position.latitude;
                            output.longitude = global_position.longitude;
                            output.altitude = global_position.altitude;
                            output.altitude = global_position.relative_altitude;
                            output.velocity.x() = global_position.vx;
                            output.velocity.y() = global_position.vy;
                            output.velocity.z() = global_position.vz;
                        }
                        break;
                    default:
                        break;
                }
                ostream.write( output );
            }
            else
            {
                if( verbose || strict ) { std::cerr << "mavlink-to-csv: failed to parse mavlink message, discarded" << std::endl; if( strict ) { return 1; } }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "mavlink-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "mavlink-to-csv: unknown exception" << std::endl; }
    return 1;
}
