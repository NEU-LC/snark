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
#include <comma/base/types.h>
#include "units.h"
extern "C" {
    #include "simulink/Arm_Controller.h"
}
#include "simulink/traits.h"

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    
namespace arm = robotic_arm;
typedef boost::units::quantity< boost::units::si::angular_acceleration > angular_acceleration_t;
typedef boost::units::quantity< boost::units::si::angular_velocity > angular_velocity_t;


static const int tilt_joint = 3;

class arm_output
{
public:
    typedef arm::current_positions current_positions_t;
private:
    angular_acceleration_t acceleration_;
    angular_velocity_t velocity_;
    angular_acceleration_t camera_acceleration_;
    angular_velocity_t camera_velocity_;
    ExtY_Arm_Controller_T& joints;
    current_positions_t& current_positions;
public:
    arm_output( const angular_acceleration_t& ac, const angular_velocity_t& vel,
                const angular_acceleration_t& camera_ac, const angular_velocity_t& camera_vel,
                ExtY_Arm_Controller_T& output ) : 
                acceleration_( ac ), velocity_( vel ), 
                camera_acceleration_( camera_ac ), camera_velocity_( camera_vel ), 
                joints( output ), 
                current_positions( static_cast< current_positions_t& >( output ) ) 
                {
                    Arm_Controller_initialize();
                }
   ~arm_output() 
    { 
        Arm_Controller_terminate(); 
    }

    const ExtY_Arm_Controller_T& data() const { return joints; }

    /// Returns true if Simulink calculation for proposed movement is a success
    /// If true you can proceed to use the output from serialis()
    bool runnable() const { return joints.command_flag > 0; }
    bool will_collide() const { return joints.command_flag < 0; }
    /// The Simulink proposed tilt angle
    plane_angle_t proposed_tilt_angle() const {
        return joints.arm_position[ tilt_joint ] * radian;
    }
    
    const angular_acceleration_t& acceleration() const { return acceleration_; }
    const angular_velocity_t& velocity() const { return velocity_; }
    /// Get the simulink angles in degrees
    std::string debug_in_degrees() const
    {
        std::ostringstream ss;
        ss << "debug: movej([";
        for(std::size_t i=0; i<6u; ++i) 
        {
           ss << static_cast< arm::plane_angle_degrees_t >( joints.joint_angle_vector[i] * arm::radian ).value();
           if( i < 5 ) { ss << ','; }
        }
        ss << "],a=" << acceleration_.value() << ','
           << "v=" << velocity_.value() << ')';
           
           
        return ss.str();
    }
    
    /// Produce the robotic arm command with the joint angles given by Simulink output
    std::string serialise() const
    {
        static std::string tmp;
        static comma::csv::ascii< ExtY_Arm_Controller_T > ascii;
        std::ostringstream ss;
        ss << "movej([" << ascii.put( joints, tmp )
           << "],a=" << acceleration_.value() << ','
           << "v=" << velocity_.value() << ')';
        return ss.str();
    }
    
    /// Serialise to robotic arm command for camera movement which has different accelleration and velocity.
    std::string camera_serialise() const
    {
        static std::string tmp;
        static comma::csv::ascii< ExtY_Arm_Controller_T > ascii;
        std::ostringstream ss;
        ss << "movej([" << ascii.put( joints, tmp )
           << "],a=" << camera_acceleration_.value() << ','
           << "v=" << camera_velocity_.value() << ')';
        return ss.str();
    }
   
};

    
} } } } //namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_OUTPUT_H