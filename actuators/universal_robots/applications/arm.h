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

#ifndef COMMA_UR_ARM
#define COMMA_UR_ARM

#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <comma/base/exception.h>

namespace comma { namespace ur {

static const unsigned int number_of_joints = 6;
static const unsigned int number_of_pose_fields = 6;

typedef boost::bimap< int, std::string > modes_t;

static const modes_t robot_modes = boost::assign::list_of< modes_t::relation >
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

inline std::string robot_mode_to_name( int mode ) 
{ 
    if( !robot_modes.left.count( mode ) ) { COMMA_THROW( comma::exception, "robot mode " << mode << " is not found" ); };
    return robot_modes.left.at( mode );
}

inline int robot_mode_from_name( std::string name ) 
{ 
    if( !robot_modes.right.count( name ) ) { COMMA_THROW( comma::exception, "robot mode \'" << name << "\' is not found" ); };
    return robot_modes.right.at( name );
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

} } // namespace comma { namespace ur {

#endif // #ifndef COMMA_UR_ARM