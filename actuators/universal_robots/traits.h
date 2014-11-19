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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_TRAITS_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_TRAITS_H
#include <boost/units/quantity.hpp>
#include <comma/visiting/traits.h>
#include <snark/visiting/eigen.h>
#include "commands.h"
#include "commands_handler.h"
#include "units.h"
#include "config.h"
#include "data.h"

namespace comma { namespace visiting {
    
using snark::ur::command_base;

// Commands
template <typename C> struct traits< command_base< C > >
{
    template< typename K, typename V > static void visit( const K& k, command_base< C >& t, V& v )
    {
        v.apply( "robot_id", t.robot_id );
        v.apply( "sequence_number", t.sequence_number );
        v.apply( "name", t.name );
    }
    template< typename K, typename V > static void visit( const K& k, const command_base< C >& t, V& v )
    {
        v.apply( "robot_id", t.robot_id );
        v.apply( "sequence_number", t.sequence_number );
        v.apply( "name", t.name );
    }
};

template <> struct traits< snark::ur::position >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::position& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::position& t, V& v )
    {
        v.apply( "x", t.x );
        v.apply( "y", t.y );
        v.apply( "z", t.z );
    }
};

template <> struct traits< snark::ur::move_effector >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::move_effector& t, V& v )
    {
        traits< command_base < snark::ur::move_effector > >::visit(k, t, v);
        v.apply( "offset", t.offset );
   }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::move_effector& t, V& v )
    {
        traits< command_base < snark::ur::move_effector > >::visit(k, t, v);
        v.apply( "offset", t.offset );
    }
};

template <> struct traits< snark::ur::continuum_t::arm_position_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::continuum_t::arm_position_t& t, V& v )
    {
        for( std::size_t i=0; i<snark::ur::joints_num; ++i ) 
        {
            double d = t[i].value();
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), d );
            t[i] = d * snark::ur::radian;
        }
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::continuum_t::arm_position_t& t, V& v )
    {
        for( std::size_t i=0; i<snark::ur::joints_num; ++i ) {
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), t[i].value() );
        }
    }
};

template <> struct traits< snark::ur::move_joints >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::move_joints& t, V& v )
    {
        traits< command_base < snark::ur::move_joints > >::visit(k, t, v);
        v.apply( "joints", t.joints );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::move_joints& t, V& v )
    {
        traits< command_base < snark::ur::move_joints > >::visit(k, t, v);
        v.apply( "joints", t.joints );
    }
};

template <> struct traits< snark::ur::joint_move >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::joint_move& t, V& v )
    {
        traits< command_base < snark::ur::joint_move > >::visit(k, t, v);
        v.apply( "joint_id", t.joint_id );
        v.apply( "dir", t.dir );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::joint_move& t, V& v )
    {
        traits< command_base < snark::ur::joint_move > >::visit(k, t, v);
        v.apply( "joint_id", t.joint_id );
        v.apply( "dir", t.dir );
    }
};

template <> struct traits< snark::ur::auto_init >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::auto_init& t, V& v ) {
        traits< command_base < snark::ur::auto_init > >::visit(k, t, v);
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::auto_init& t, V& v ) {
        traits< command_base < snark::ur::auto_init > >::visit(k, t, v);
    }
};

template <> struct traits< snark::ur::auto_init_force >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::auto_init_force& t, V& v )
    {
        traits< command_base < snark::ur::auto_init_force > >::visit(k, t, v);
        v.apply( "force", t.force );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::auto_init_force& t, V& v )
    {
        traits< command_base < snark::ur::auto_init_force > >::visit(k, t, v);
        v.apply( "force", t.force );
    }
};

template <> struct traits< snark::ur::power >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::power& t, V& v ) {
        traits< command_base < snark::ur::power > >::visit(k, t, v);
        v.apply( "is_on", t.is_on );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::power& t, V& v ) {
        traits< command_base < snark::ur::power > >::visit(k, t, v);
        v.apply( "is_on", t.is_on );
    }
};

template <> struct traits< snark::ur::brakes >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::brakes& t, V& v ) {
        traits< command_base < snark::ur::brakes > >::visit(k, t, v);
        v.apply( "enable", t.enable );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::brakes& t, V& v ) {
        traits< command_base < snark::ur::brakes > >::visit(k, t, v);
        v.apply( "enable", t.enable );
    }
};

template <> struct traits< snark::ur::set_position >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::set_position& t, V& v )
    {
        traits< command_base < snark::ur::set_position > >::visit(k, t, v);
        v.apply( "position", t.position );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::set_position& t, V& v )
    {
        traits< command_base < snark::ur::set_position > >::visit(k, t, v);
        v.apply( "position", t.position );
    }
};

template <> struct traits< snark::ur::set_home >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::set_home& t, V& v )
    {
        traits< command_base < snark::ur::set_home > >::visit(k, t, v);
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::set_home& t, V& v )
    {
        traits< command_base < snark::ur::set_home > >::visit(k, t, v);
    }
};

template <> struct traits< snark::ur::continuum_t::scan_type >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::continuum_t::scan_type& t, V& v )
    {
        double sweep_angle = t.sweep_angle.value();
        v.apply( "sweep_angle", sweep_angle );
        t.sweep_angle = sweep_angle * snark::ur::degree;

        double vel = t.sweep_velocity.value();
        v.apply( "sweep_velocity", vel );
        t.sweep_velocity = vel * snark::ur::rad_per_sec;

        v.apply( "fields", t.fields );
        v.apply( "range-limit", t.range_limit );
        v.apply( "thinning", t.thinning_value );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::continuum_t::scan_type& t, V& v )
    {
        v.apply( "sweep_angle", t.sweep_angle.value() );
        v.apply( "sweep_velocity", t.sweep_velocity.value() );
        v.apply( "fields", t.fields );
        v.apply( "range-limit", t.range_limit );
        v.apply( "thinning", t.thinning_value );
    }
};

template <> struct traits< snark::ur::continuum_t::lidar_config >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::continuum_t::lidar_config& t, V& v )
    {
        v.apply( "service-host", t.service_host );
        v.apply( "service-port", t.service_port );
        v.apply( "scan-forwarding-port", t.scan_forwarding_port );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::continuum_t::lidar_config& t, V& v )
    {
        v.apply( "service-host", t.service_host );
        v.apply( "service-port", t.service_port );
        v.apply( "scan-forwarding-port", t.scan_forwarding_port );
    }
};

template <> struct traits< snark::ur::continuum_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::continuum_t& t, V& v )
    {
        v.apply( "home_position", t.home_position );
        v.apply( "work_directory", t.work_directory );
        v.apply( "scan", t.scan );
        v.apply( "hokuyo", t.lidar );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::continuum_t& t, V& v )
    {
        v.apply( "home_position", t.home_position );
        v.apply( "work_directory", t.work_directory );
        v.apply( "scan", t.scan );
        v.apply( "hokuyo", t.lidar );
    }
};

template <> struct traits< snark::ur::config >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::config& t, V& v )
    {
        v.apply( "continuum", t.continuum );
    }

    template< typename K, typename V > static void visit( const K& k, const snark::ur::config& t, V& v )
    {
        v.apply( "continuum", t.continuum );
    }
};

template < > struct traits< boost::array< comma::packed::big_endian_double, 6 > >
{
    template< typename K, typename V > static void visit( const K& k, const boost::array< comma::packed::big_endian_double, 6 >& t, V& v )
    {
        std::size_t i = 0;
        for( const comma::packed::big_endian_double* iter=t.cbegin(); iter!=t.cend(); ++iter ) { 
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), (*iter)() ); 
            ++i;
        }
    }
};

template < > struct traits< snark::ur::joints_in_degrees >
{
    template< typename K, typename V > static void visit( const K& k, const snark::ur::joints_in_degrees& t, V& v )
    {
        for( std::size_t i=0; i<snark::ur::joints_num; ++i ) { 
            double d = t.joints[i].value();
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), d ); 
        }
    }
};

template < > struct traits< snark::ur::cartesian >
{
    template< typename K, typename V > static void visit( const K& k, const snark::ur::cartesian& t, V& v )
    {
        v.apply( "x", t.x() );
        v.apply( "y", t.y() );
        v.apply( "z", t.z() );
    }
};

template <> struct traits< snark::ur::fixed_status >
{
    /// Use this for debugging maybe
    template< typename K, typename V > static void visit( const K& k, const  snark::ur::fixed_status& t, V& v )
    {
        v.apply( "time", boost::posix_time::microsec_clock::local_time() );
        v.apply( "coordinates", t.translation );
        v.apply( "rotation", t.rotation );
        snark::ur::joints_in_degrees positions( t.positions );
        v.apply( "positions", positions );
        v.apply( "velocities", t.velocities );
        v.apply( "currents", t.currents );
        v.apply( "forces", t.forces );
        v.apply( "temperatures", t.temperatures );
        v.apply( "robot_mode",  t.mode_str() );
        snark::ur::joint_modes_t jmodes( t.joint_modes );
        v.apply( "joint_modes", jmodes.modes );
        v.apply( "length", t.length() );    /// Binary length of message received
        v.apply( "time_since_boot", t.time_since_boot() );
    }
};

template < > struct traits< boost::array< snark::ur::jointmode::mode, 6 > >
{
    template< typename K, typename V > static void visit( const K& k, boost::array< snark::ur::jointmode::mode, 6 >& t, V& v )
    {
        for( std::size_t i=0; i<snark::ur::joints_num; ++i )
        {
            std::string mode;
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), mode ); 
            t[i] = snark::ur::get_jointmode( mode );
        }
    }
    template< typename K, typename V > static void visit( const K& k, const boost::array< snark::ur::jointmode::mode, 6 >& t, V& v )
    {
        for( std::size_t i=0; i<snark::ur::joints_num; ++i )
        {
            v.apply( boost::lexical_cast< std::string >( i ).c_str(), std::string( snark::ur::jointmode_str( t[i] ) ) ); 
        }
    }
};

template < > struct traits< snark::ur::status_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::status_t& t, V& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "position", t.position );
        v.apply( "laser_position", t.laser_position );
        v.apply( "joint_angles", t.joint_angles );
        v.apply( "velocities", t.velocities );
        v.apply( "currents", t.currents );
        v.apply( "forces", t.forces );
        v.apply( "temperatures", t.temperatures );
        std::string mode;
        v.apply( "robot_mode",  mode );
        t.robot_mode = snark::ur::get_robotmode( mode );
        v.apply( "joint_modes", t.joint_modes );
        v.apply( "length", t.length );    /// Binary length of message received
        v.apply( "time_since_boot", t.time_since_boot );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::status_t& t, V& v )
    {
        v.apply( "timestamp", t.timestamp );
        v.apply( "position", t.position );
        v.apply( "laser_position", t.laser_position );
        v.apply( "joint_angles", t.joint_angles );
        v.apply( "velocities", t.velocities );
        v.apply( "currents", t.currents );
        v.apply( "forces", t.forces );
        v.apply( "temperatures", t.temperatures );
        v.apply( "robot_mode",  t.mode_str() );
        v.apply( "joint_modes", t.joint_modes );
        v.apply( "length", t.length );    /// Binary length of message received
        v.apply( "time_since_boot", t.time_since_boot );
    }
};

template < > struct traits< snark::ur::move_config_t >
{
    template< typename K, typename V > static void visit( const K& k, snark::ur::move_config_t& t, V& v )
    {
        v.apply( "angles", (boost::array< double, snark::ur::joints_num >&) t );
    }
    template< typename K, typename V > static void visit( const K& k, const snark::ur::move_config_t& t, V& v )
    {
        v.apply( "angles", (const boost::array< double, snark::ur::joints_num >&) t );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_TRAITS_H
