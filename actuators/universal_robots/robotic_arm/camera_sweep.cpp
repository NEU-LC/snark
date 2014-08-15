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

#include "camera_sweep.h"
#include "output.h"
#include "traits.h"
#include <comma/math/compare.h>

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    
// TODO how do you cancel an actioned item, stopj and run mode? or set to current angles
// Currently it sets to current joints angles, both work
// The other method requires to be a bit of a wait for mode change
void camera_sweep::stop_movement(std::ostream& rover)
{
//     static const std::string stop_str = "stopj([0.05,0.05,0.05,0.05,0.05,0.05])";
    static comma::csv::ascii< status_t::array_joint_angles_t > ascii;
    static std::string tmp;
    
    std::ostringstream ss;
    status_update_();
    ss << "movej([" << ascii.put( status_.joint_angles, tmp ) 
       << "],a=" << serialiser_.acceleration().value() << ','
       << "v=" << serialiser_.velocity().value() << ')';
    const std::string stop_str = ss.str();
    rover << stop_str << std::endl;
    rover.flush();
//     usleep( 0.1 * 1000000u );
//     rover << "set robotmode run" << std::endl;
//     rover.flush();
}

    
result camera_sweep::run( const length_t& height, const plane_angle_degrees_t& pan, 
                          const plane_angle_degrees_t& tilt_down, const plane_angle_degrees_t& tilt_up, 
                          std::ostream& rover )
{
    
    move_t move1, move2, ret;
    if( !calculate_solution( height, pan, tilt_down, tilt_up, move1, move2, ret ) )
    {
        return result( "cannot perform the proposed camera sweep because of collision", result::error::failure );
    }
    
    bool stop = interrupt_();
    if( signaled_ || stop ) { return result( "camera sweep action is cancelled", result::error::cancelled ); }
    
    std::cerr << name() << "running action 1: " << move1.action << " target angle: " << move1.tilt.value() << std::endl;
    rover << move1.action << std::endl;
    rover.flush();
    
    /// Check that it stopped
    static comma::uint32 usec = 0.1 * 1000000u;
    
    static const arm::plane_angle_t epsilon = static_cast< arm::plane_angle_t >( 0.5 * arm::degree );
    while( !comma::math::equal( status_.joint_angles[ tilt_joint ], move1.tilt, epsilon  ) )
    {
        status_update_();
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    std::cerr << name() << "running action 2: " << move2.action << " target angle:" << move2.tilt.value() << std::endl;
    rover << move2.action << std::endl;
    rover.flush();
    while( !comma::math::equal( status_.joint_angles[ tilt_joint ], move2.tilt, epsilon  ) )
    {
        status_update_();
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    std::cerr << name() << "returning to position: " << ret.action << " target angle:" << ret.tilt.value() << std::endl;
    rover << ret.action << std::endl;
    rover.flush();
    while( !comma::math::equal( status_.joint_angles[ tilt_joint ], ret.tilt, epsilon  ) )
    {
        status_update_();
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    return result();
}

bool camera_sweep::calculate_solution( const length_t& height, const plane_angle_degrees_t& pan, 
                                       const plane_angle_degrees_t& tilt_down, const plane_angle_degrees_t& tilt_up, 
                                       move_t& move1, move_t& move2, move_t& ret )
{
    const plane_angle_t current_tilt = status_.joint_angles[ tilt_joint ];
    
    inputs_reset();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = pan.value();
    inputs_.Input_2 = tilt_down.value();
    inputs_.Input_3 = height.value();
    Arm_Controller_step();
    
    if( !serialiser_.runnable() ) { std::cerr << name() << "failed to find move action 1, is collision: " << serialiser_.will_collide() << std::endl; return false; }
    
    /// Get commands
    move1 = move_t( serialiser_.serialise(), serialiser_.proposed_tilt_angle() );
    
    inputs_reset();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = pan.value();
    inputs_.Input_2 = tilt_up.value();
    inputs_.Input_3 = height.value();
    Arm_Controller_step();
    
    if( !serialiser_.runnable() ) { std::cerr << name() << "failed to find move action 2, will_collide: " << serialiser_.will_collide() << std::endl; return false; }
    
    /// Get commands
    move2 = move_t( serialiser_.serialise(), serialiser_.proposed_tilt_angle() );
    
    // return to former position
    inputs_reset();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = pan.value();
    inputs_.Input_2 = current_tilt.value();
    inputs_.Input_3 = height.value();
    Arm_Controller_step();
    
    if( !serialiser_.runnable() ) { std::cerr << name() << "failed to find move action 3, will_collide:" << serialiser_.will_collide() << std::endl; return false; }
    
    /// Get commands
    ret = move_t( serialiser_.serialise(), current_tilt );
    
    /// Done
    inputs_.motion_primitive = input_primitive::no_action;
    
    return true;
}

    
} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    