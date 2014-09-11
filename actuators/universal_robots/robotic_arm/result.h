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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_RESULT_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_RESULT_H
#include <string>
#include <iostream>

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

struct input_primitive
{
    enum {
        no_action = 0,
        move_cam = 1,
        set_position = 2,
        set_home=3,      // define home position, internal usage
        movej=4,
        scan=5,
        pan_tilt=6,
        move_effector=7
    };  
};
    
struct result
{
    struct error { enum { success=0, action_started=1, invalid_input=2, invalid_robot_state, failure, cancelled, collision }; };
    int code;
    std::string message;
    
    result( const std::string& msg, int code_ ) : code( code_ ), message( msg ) {}
    result() : code( error::success ), message( "success" ) {}
    
    std::string get_message() const 
    {
        std::ostringstream ss;
        ss << code << ',' << '"' << message << '"';
        return ss.str();
    }
    bool is_success() const { return code == error::success; }
};
    
} } } } //namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_RESULT_H