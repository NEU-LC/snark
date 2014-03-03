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

#include <cmath>
#include <comma/math/compare.h>
#include "wheel_command.h"

namespace snark { namespace wheels {

wheel_command compute_wheel_command( const steer_command &desired , Eigen::Matrix4d wheel_pose, double wheel_offset )
{
    wheel_command command;

    // a very small non zero turnrate will cause calculation errors
    if( std::fabs( desired.turnrate ) < turnrate_tolerance )
    {
        command.velocity = desired.velocity.norm();
        
        // get angle in wheel pose
        Eigen::Vector4d origin( 0, 0, 0, 1 );
        origin = frame_transforms::inverse_transform( wheel_pose ) * origin;

        Eigen::Vector4d forward( desired.velocity.x(), desired.velocity.y(), 0, 1 );
        forward = frame_transforms::inverse_transform( wheel_pose ) * forward;

        command.turnrate = std::atan2( forward(2) - origin(2), forward(0) - origin(0) );

        // limit movement to -pi / 2 and pi / 2
        if( comma::math::less( M_PI / 2, command.turnrate, 1e-9 ) )
        {
            command.turnrate -= M_PI;
            command.velocity = -command.velocity;
        }
        else if( comma::math::less( command.turnrate, -M_PI / 2, 1e-9 ) )
        {
            command.turnrate += M_PI;
            command.velocity = -command.velocity;
        }
    }
    else
    {
        // find ICR position, ICR is at a 90 deg rotation from velocity vector
        Eigen::Vector4d icr_position( -desired.velocity.y() / desired.turnrate, desired.velocity.x() / desired.turnrate, 0, 1 );

        // ICR in wheel steering axis frame
        icr_position = frame_transforms::inverse_transform( wheel_pose ) * icr_position;

        // transform ICR to wheel coordinates
        command.turnrate = -std::atan2( icr_position(0), icr_position(2) ); // take x and z positions only
        command.velocity = desired.turnrate * ( icr_position.norm() - wheel_offset );

        // limit movement to -pi / 2 and pi / 2
        if( comma::math::less( M_PI / 2, command.turnrate, 1e-9 ) )
        {
            command.turnrate -= M_PI;
            command.velocity = -desired.turnrate * ( icr_position.norm() + wheel_offset );
        }
        else if( comma::math::less( command.turnrate, -M_PI / 2, 1e-9 ) )
        {
            command.turnrate += M_PI;
            command.velocity = -desired.turnrate * ( icr_position.norm() + wheel_offset );
        }
    }

    return command;
}

} } // namespace snark { namespace wheels {
