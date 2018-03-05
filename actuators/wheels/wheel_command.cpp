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

#include <cmath>
#include <comma/base/exception.h>
#include <comma/math/compare.h>
#include "wheel_command.h"

namespace {

bool apply_limit_reverses_direction( snark::wheels::wheel_command& i_command, snark::wheels::limit const& i_angle_limit, bool const wrapped = false )
{
    bool direction_reversed = false;
    if( wrapped )
    {
        if( i_angle_limit.min <= i_angle_limit.max )
        {
            if( comma::math::less( i_angle_limit.max, i_command.turnrate, 1e-9 ) )
            {
                i_command.turnrate -= M_PI;
                direction_reversed = !direction_reversed;
            }
            if( comma::math::less( i_command.turnrate, i_angle_limit.min, 1e-9 ) )
            {
                i_command.turnrate += M_PI;
                direction_reversed = !direction_reversed;
            }
            if( i_command.turnrate > M_PI ) { i_command.turnrate -= 2 * M_PI; }
            else if( i_command.turnrate < -M_PI ) { i_command.turnrate += 2 * M_PI; }

            if( comma::math::less( i_angle_limit.max, i_command.turnrate, 1e-9 ) ||
                    comma::math::less( i_command.turnrate, i_angle_limit.min, 1e-9 ) )
            {
                COMMA_THROW( comma::exception, "angle is not between " << i_angle_limit.min << " and " << i_angle_limit.max );
            }
        }
        else
        {
            if( comma::math::less( i_command.turnrate, i_angle_limit.min, 1e-9 ) && 
                comma::math::less( i_angle_limit.max, i_command.turnrate, 1e-9 ) )
            {
                i_command.turnrate += ( i_command.turnrate <= 0 ? M_PI : -M_PI );
                direction_reversed = !direction_reversed;
            }

            if( comma::math::less( i_command.turnrate, i_angle_limit.min, 1e-9 ) && 
                comma::math::less( i_angle_limit.max, i_command.turnrate, 1e-9 ) )
            {
                COMMA_THROW( comma::exception, "angle is not between " << i_angle_limit.min << " and " << i_angle_limit.max );
            }
        }
    }
    else
    {
        while( comma::math::less( i_angle_limit.max, i_command.turnrate, 1e-9 ) )
        {
            i_command.turnrate -= M_PI;
            direction_reversed = !direction_reversed;
        }

        while( comma::math::less( i_command.turnrate, i_angle_limit.min, 1e-9 ) )
        {
            i_command.turnrate += M_PI;
            direction_reversed = !direction_reversed;
        }

        if( comma::math::less( i_angle_limit.max, i_command.turnrate, 1e-9 ) ||
                comma::math::less( i_command.turnrate, i_angle_limit.min, 1e-9 ) )
        {
            COMMA_THROW( comma::exception, "angle is outside limit of " << i_angle_limit.min << " and " << i_angle_limit.max );
        }
    }
    return direction_reversed;
}

}
namespace snark { namespace wheels {

wheel_command compute_wheel_command( const steer_command &desired, const Eigen::Matrix4d& wheel_pose, double wheel_offset, const boost::optional< limit >& angle_limit, boost::optional< double > current_angle, bool wrap )
{
    boost::optional<Eigen::Vector2d> icr;
    if( std::fabs( desired.turnrate ) >= turnrate_tolerance )
    {
        // find ICR position, ICR is at a 90 deg rotation from velocity vector
        icr=Eigen::Vector2d(-desired.velocity.y() / desired.turnrate, desired.velocity.x() / desired.turnrate);
    }
    return compute_wheel_command(desired, wheel_pose, wheel_offset, angle_limit, current_angle, wrap,icr);
}

wheel_command compute_wheel_command(const steer_command& desired, const Eigen::Matrix4d& wheel_pose, double wheel_offset, const boost::optional< limit >& angle_limit, boost::optional< double > current_angle, bool wrap,const boost::optional<Eigen::Vector2d>& icr)
{
    wheel_command command;

    double positive_velocity;
    double negative_velocity;

    // a very small non zero turnrate will cause calculation errors
    if( !icr )
    {
        // get angle in wheel pose
        Eigen::Vector4d origin( 0, 0, 0, 1 );
        origin = frame_transforms::inverse_transform( wheel_pose ) * origin;

        Eigen::Vector4d forward( desired.velocity.x(), desired.velocity.y(), 0, 1 );
        forward = frame_transforms::inverse_transform( wheel_pose ) * forward;

        command.turnrate = std::atan2( forward(2) - origin(2), forward(0) - origin(0) );

        positive_velocity = desired.velocity.norm();
        negative_velocity = -positive_velocity;
    }
    else
    {
        Eigen::Vector4d icr_position( icr->x(), icr->y(), 0, 1 );

        // ICR in wheel steering axis frame
        icr_position = frame_transforms::inverse_transform( wheel_pose ) * icr_position;

        // transform ICR to wheel coordinates
        command.turnrate = -std::atan2( icr_position(0), icr_position(2) ); // take x and z positions only

        // distance from wheel to ICR
        double d = Eigen::Vector3d( icr_position(0), 0, icr_position(2) ).norm();

        positive_velocity = desired.turnrate * ( d + wheel_offset );
        negative_velocity = -desired.turnrate * ( d - wheel_offset );
    }

    bool positive_direction = true;

    if( comma::math::equal( std::abs( command.turnrate ), M_PI, 1e-9 ) )
    {
        command.turnrate = 0;
        positive_direction = !positive_direction;
    }

    // minimize change to new angle if current angle is known
    if( current_angle )
    {
        // reduce current angle to [-180,180]
        double current = std::fmod( *current_angle, 2 * M_PI );
        if( current > M_PI ) { current -= 2 * M_PI; }
        else if( current < -M_PI ) { current += 2 * M_PI; }

        double delta = command.turnrate - current;

        // reduce delta to [-90,270)
        if( !comma::math::less( std::abs( delta ), M_PI * 1.5, 1e-9 ) ) // [270,360]
        {
            delta = delta > 0 ? delta - 2 * M_PI : delta + 2 * M_PI;
        }

        // reduce delta to [-90,90]
        if( comma::math::less( M_PI/2, std::abs( delta ), 1e-9 ) ) // (90,270)
        {
            delta = delta > 0 ? delta - M_PI : delta + M_PI;
            positive_direction = !positive_direction;
        }

        if( wrap )
        {
            command.turnrate = *current_angle + delta;
            if( command.turnrate > M_PI ) { command.turnrate -= 2 * M_PI; }
            else if( command.turnrate < -M_PI ) { command.turnrate += 2 * M_PI; }
            if( angle_limit ) positive_direction = positive_direction != apply_limit_reverses_direction( command, *angle_limit, true );
            command.velocity = positive_direction ? positive_velocity : negative_velocity;
            return command;
        }

        // if |delta| > 60 then bias towards keeping wheel on the "outside"
        if( comma::math::less( M_PI/3, std::abs( delta ), 1e-9 ) ) // (60,90]
        {
            int sign = wheel_pose( 0, 3 ) * wheel_pose( 1, 3 ) < 0 ? 1 : -1;
            int preferred_direction = current > sign * M_PI/4 ? -1 : 1;
            if( delta > 0 && preferred_direction == -1 )
            {
                delta -= M_PI;
                positive_direction = !positive_direction;
            }
            else if ( delta < 0 && preferred_direction == 1 )
            {
                delta += M_PI;
                positive_direction = !positive_direction;
            }
        }
        command.turnrate = *current_angle + delta;
    }
    if( angle_limit ) positive_direction = positive_direction != apply_limit_reverses_direction( command, *angle_limit );
    command.velocity = positive_direction ? positive_velocity : negative_velocity;

    return command;
}

} } // namespace snark { namespace wheels {
