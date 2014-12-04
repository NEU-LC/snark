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

#ifndef COMMA_UR_BASE
#define COMMA_UR_BASE

#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <comma/base/exception.h>
#include <boost/array.hpp>
#include <comma/packed/packed.h>
#include "packet.h"

namespace snark { namespace ur {

//static const unsigned int number_of_joints = 6;
static const unsigned int number_of_pose_fields = 6;

// struct packed_joint_values_t : public comma::packed::packed_struct< packed_joint_values_t, 48 >
// {        
//     typedef comma::packed::net_float64 type;
//     type base;
//     type shoulder;
//     type elbow;
//     type wrist1;
//     type wrist2;
//     type wrist3;    
// };
// 
// struct packet_t : public comma::packed::packed_struct< packet_t, 812  >
// {
//     comma::packed::net_uint32 length;
//     comma::packed::net_float64 time_since_boot;
//     comma::packed::string< 240 > dummy1;
//     packed_joint_values_t actual_joint_angles;
//     packed_joint_values_t actual_joint_speeds;
//     packed_joint_values_t actual_joint_currents;
//     comma::packed::string< 144 > dummy2;
//     boost::array< comma::packed::net_float64, number_of_pose_fields > tool_force;
//     boost::array< comma::packed::net_float64, number_of_pose_fields > tool_pose;
//     boost::array< comma::packed::net_float64, number_of_pose_fields > tool_speed;
//     comma::packed::string< 8 > dummy3;
//     packed_joint_values_t joint_temperatures;
//     comma::packed::string< 16 > dummy4;
//     comma::packed::net_float64 mode;
//     packed_joint_values_t joint_modes;
// };

template < typename T >
struct joint_values_t
{
    typedef T type;
    type base;
    type shoulder;
    type elbow;
    type wrist1;
    type wrist2;
    type wrist3;
    void import_from( const snark::ur::packed_joint_values_t& v )
    {
        base = static_cast< type >( v.base() );
        shoulder = static_cast< type >( v.shoulder() );
        elbow = static_cast< type >( v.elbow() );
        wrist1 = static_cast< type >( v.wrist1() );
        wrist2 = static_cast< type >( v.wrist2() );
        wrist3 = static_cast< type >( v.wrist3() );
    };
};

struct arm_t
{
    joint_values_t< double > angles;
    joint_values_t< double > speeds;
    joint_values_t< double > currents;
    joint_values_t< double > temperatures;
    joint_values_t< int > modes;    
    void import_from( const snark::ur::packet_t& p )
    {
        angles.import_from( p.actual_joint_angles );
        speeds.import_from( p.actual_joint_speeds );
        currents.import_from( p.actual_joint_currents );
        temperatures.import_from( p.joint_temperatures );
        modes.import_from( p.joint_modes );
    };
};

} }

namespace comma { namespace visiting {    

template < typename T > struct traits< snark::ur::joint_values_t< T > >
{
    template< typename K, typename V > static void visit( const K&, snark::ur::joint_values_t< T >& t, V& v )
    {
        v.apply( "base", t.base );
        v.apply( "shoulder", t.shoulder );
        v.apply( "elbow", t.elbow );
        v.apply( "wrist1", t.wrist1 );
        v.apply( "wrist2", t.wrist2 );
        v.apply( "wrist3", t.wrist3 );
    }
    template< typename K, typename V > static void visit( const K&, const snark::ur::joint_values_t< T >& t, V& v )
    {
        v.apply( "base", t.base );
        v.apply( "shoulder", t.shoulder );
        v.apply( "elbow", t.elbow );
        v.apply( "wrist1", t.wrist1 );
        v.apply( "wrist2", t.wrist2 );
        v.apply( "wrist3", t.wrist3 );
    }
};

template < > struct traits< snark::ur::arm_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::arm_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "speeds", t.speeds );
        v.apply( "currents", t.currents );
        v.apply( "temperatures", t.temperatures );
        v.apply( "modes", t.modes );
    }    
    template< typename K, typename V > static void visit( const K& k, const snark::ur::arm_t& t, V& v )
    {
        v.apply( "angles", t.angles );
        v.apply( "speeds", t.speeds );
        v.apply( "currents", t.currents );
        v.apply( "temperatures", t.temperatures );
        v.apply( "modes", t.modes );
    }
};

} }

namespace snark { namespace ur {

typedef boost::bimap< int, std::string > modes_t;

static const modes_t modes = boost::assign::list_of< modes_t::relation >
    ( 0, "running" )
    ( 1, "freedrive" )
    ( 2, "ready" )
    ( 3, "initialising" )
    ( 4, "security-stopped" )
    ( 5, "emergency-stopped" )
    ( 6, "fatal-error" )
    ( 7, "no-power" )
    ( 8, "not-connected" )
    ( 9, "shutdown" )
    ( 10, "safeguard-stop" );
    
static const modes_t joint_modes = boost::assign::list_of< modes_t::relation >
    ( 237, "part-d-calibration" )
    ( 238, "backdrive" )
    ( 239, "power-off" )
    ( 240, "emergency-stopped" )
    ( 241, "calval-initialisation" )
    ( 242, "error" )
    ( 243, "freedrive" )
    ( 244, "simulated" )
    ( 245, "not-responding" )
    ( 246, "motor-initialisation" )
    ( 247, "adc-calibration" )
    ( 248, "dead-commutation" )
    ( 249, "bootloader" )
    ( 250, "calibration" )
    ( 251, "stopped" )
    ( 252, "fault" )
    ( 253, "running" )
    ( 254, "initialisation" )
    ( 255, "idle" );

inline std::string mode_to_name( int mode ) 
{ 
    if( !modes.left.count( mode ) ) { COMMA_THROW( comma::exception, "robot mode " << mode << " is not found" ); };
    return modes.left.at( mode );
}

inline int mode_from_name( std::string name ) 
{ 
    if( !modes.right.count( name ) ) { COMMA_THROW( comma::exception, "robot mode \'" << name << "\' is not found" ); };
    return modes.right.at( name );
}

inline std::string joint_mode_to_name( int mode ) 
{ 
    if( !joint_modes.left.count( mode ) ) { COMMA_THROW( comma::exception, "joint mode " << mode << " is not found" ); };
    return joint_modes.left.at( mode );
}

inline int joint_mode_from_name( std::string name ) 
{ 
    if( !joint_modes.right.count( name ) ) { COMMA_THROW( comma::exception, "joint mode \'" << name << "\' is not found" ); };
    return joint_modes.right.at( name );
}

} } // namespace snark { namespace ur {

#endif // #ifndef COMMA_UR_BASE