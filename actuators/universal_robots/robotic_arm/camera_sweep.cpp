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


namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    
void camera_sweep::stop_movement(std::ostream& rover)
{
    static const std::string stop_str = "stopj([0.05,0.05,0.05,0.05,0.05,0.05])";
    
    rover << stop_str << std::endl;
    rover.flush();
    usleep( 0.1 * 1000000u );
    rover << "set robotmode run" << std::endl;
    rover.flush();
}

    
result camera_sweep::run( const length_t& height, std::ostream& rover )
{
    
    std::string action1; 
    std::string action2;
    if( !calculate_solution( height, action1, action2 ) )
    {
        return result( "cannot perform the proposed camera sweep because of collision", result::error::failure );
    }
    
    bool stop = interrupt_();
    if( signaled_ || stop ) { return result( "camera sweep action is cancelled", result::error::cancelled ); }
    
    rover << action1 << std::endl;
    rover.flush();
    
    /// Check that it stopped
    static comma::uint32 usec = 0.1 * 1000000u;
    
    while( !status_.is_stationary() )
    {
        stop = interrupt_();
        // TODO how do you cancel an actioned item, stopj and run mode? or set to current angles
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    rover << action2 << std::endl;
    rover.flush();
    while( !status_.is_stationary() )
    {
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    return result();
}

bool camera_sweep::calculate_solution( const length_t& height, 
                                       std::string& move1, std::string& move2)
{
    Arm_Controller_initialize();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = 0;
    // inputs_.Input_2 = cam.height.value() != 1.0 ? -cam.tilt.value() : zero_tilt - cam.tilt.value();
    inputs_.Input_2 = -60;
    inputs_.Input_3 = height.value();
    Arm_Controller_step();
    
    if( outputs_.command_flag < 0 ) { std::cerr << name() << "failed to find move action 1" << std::endl; return false; }
    
    /// Get commands
    move1 = serialiser_.serialise();
    
    Arm_Controller_terminate();
        
    Arm_Controller_initialize();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = 0;
    // inputs_.Input_2 = cam.height.value() != 1.0 ? -cam.tilt.value() : zero_tilt - cam.tilt.value();
    inputs_.Input_2 = 60;
    inputs_.Input_3 = height.value();
    Arm_Controller_step();
    
    if( outputs_.command_flag < 0 ) { std::cerr << name() << "failed to find move action 1" << std::endl; return false; }
    
    /// Get commands
    move2 = serialiser_.serialise();
    
    inputs_.motion_primitive = input_primitive::no_action;
    
    return true;
}

    
} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    