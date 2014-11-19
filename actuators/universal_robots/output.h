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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_OUTPUT_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_OUTPUT_H

#include <string>
#include <iostream>
#include <boost/array.hpp>
#include <comma/base/types.h>
#include "units.h"
#include "Arm_controller_v2.h"

namespace snark { namespace ur { namespace handlers {

typedef boost::units::quantity< boost::units::si::angular_acceleration > angular_acceleration_t;
typedef boost::units::quantity< boost::units::si::angular_velocity > angular_velocity_t;

static const int tilt_joint = 3;

class arm_output
{
public:
    arm_output( const angular_acceleration_t& ac, const angular_velocity_t& vel, ExtY_Arm_controller_v2_T& output ) : 
        acceleration_( ac ), velocity_( vel ), joints( output ) { Arm_controller_v2_initialize(); }
   ~arm_output() { Arm_controller_v2_terminate(); }

    const ExtY_Arm_controller_v2_T& data() const { return joints; }

    /// Returns true if Simulink calculation for proposed movement is a success
    /// If true you can proceed to use the output from serialis()
    bool runnable() const { return joints.command_flag > 0; }
    bool will_collide() const { return joints.command_flag < 0; }
    /// The Simulink proposed tilt angle
    plane_angle_t proposed_tilt_angle( comma::uint32 index=0) const { return get_move_config(index)[ tilt_joint ] * snark::ur::radian; }
    
    const angular_acceleration_t& acceleration() const { return acceleration_; }
    const angular_velocity_t& velocity() const { return velocity_; }
    comma::uint32 num_of_moves() const { return boost::lexical_cast< comma::uint32 >( joints.number_waypoints ); }
    
    const move_config_t& get_move_config( comma::uint32 index ) const
    {
        const move_config_t* move = NULL;
        switch( index )
        {
            case 0u:
                move = reinterpret_cast< const move_config_t* >( joints.WP1 );
                return *move;
            case 1u:
                move = reinterpret_cast< const move_config_t* >( joints.WP2 );
                return *move;
            case 2u:
                move = reinterpret_cast< const move_config_t* >( joints.WP3 );
                return *move;
            case 3u:
                move = reinterpret_cast< const move_config_t* >( joints.WP4 );
                return *move;
            case 4u:
                move = reinterpret_cast< const move_config_t* >( joints.WP5 );
                return *move;
            default:
                move = reinterpret_cast< const move_config_t* >( joints.WP6 );
                return *move;
        }
    }
    
    /// Get the simulink angles in degrees
    std::string debug_in_degrees( comma::uint32 index=0 ) const
    {
        std::ostringstream ss;
        ss << "debug: movej([";
        const move_config_t& move = get_move_config( index );
        for(std::size_t i=0; i<6u; ++i) { ss << static_cast< snark::ur::plane_angle_degrees_t >( move[i] * snark::ur::radian ).value(); if( i < 5 ) { ss << ','; } }
        ss << "],a=" << acceleration_.value() << ',' << "v=" << velocity_.value() << ')';
        return ss.str();
    }
    
    /// Produce the robotic arm command with the joint angles given by Simulink output
    /// index points to the position to move to
    std::string serialise( comma::uint32 index=0 ) const { return serialise_waypoint( index, velocity_ ); }
    std::string serialise( const angular_velocity_t& velocity, comma::uint32 index=0 ) const { return serialise_waypoint( index, velocity ); }

private:
    angular_acceleration_t acceleration_;
    angular_velocity_t velocity_;
    ExtY_Arm_controller_v2_T& joints;
    
    std::string serialise_waypoint( comma::uint32 index, const angular_velocity_t& velocity ) const
    {
        static std::string tmp;
        static comma::csv::ascii< move_config_t > ascii;

        angular_velocity_t velocity_capped = velocity > velocity_ + velocity_ ? velocity_ + velocity_ : velocity;
        
        std::ostringstream ss;
        ss << "movej([" << ascii.put( get_move_config(index), tmp )
           << "],a=" << acceleration_.value() << ','
           << "v=" << velocity_capped.value() << ')';
        return ss.str();
    }
   
};

} } } //namespace snark { namespace ur { namespace handlers {

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_OUTPUT_H