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

#include <iostream>
#include <iomanip>
#include <boost/static_assert.hpp>
#include "control.h"
#include "wrap_angle.h"

namespace snark { namespace control {

mode_t mode_from_string( std::string s )
{
    if( !named_modes.right.count( s ) ) { COMMA_THROW( comma::exception, "control mode '" << s << "' is not found" ); };
    return  named_modes.right.at( s );
}

std::string mode_to_string( mode_t m ) { return  named_modes.left.at( m ); }

std::string serialise( const vector_t& p )
{
    std::stringstream s;
    s << std::fixed << std::setprecision(12) << p.x() << ',' << p.y();
    return s.str();
}

vector_t normalise( const vector_t& v ) { return v.normalized(); }

wayline_t::wayline_t( const vector_t& from, const vector_t& to )
    : v_( normalise( to - from ) )
    , line_( Eigen::ParametrizedLine< double, dimensions >::Through( from, to ) )
    , perpendicular_line_at_end_( v_, to )
    , heading( atan2( v_.y(), v_.x() ) )
    {
        BOOST_STATIC_ASSERT( dimensions == 2 );
    }

bool wayline_t::is_past_endpoint( const vector_t& location ) const
{
    return perpendicular_line_at_end_.signedDistance( location ) > 0;
}

double wayline_t::cross_track_error( const vector_t& location ) const
{
    return -line_.signedDistance( location );
}

double wayline_t::heading_error( double yaw, double target_heading ) const
{
    return wrap_angle( heading + target_heading - yaw );
}

void wayline_follower::set_target( const target_t& target, const vector_t& current_position )
{
    vector_t from = ( mode_ == fixed && target_ ) ? target_->position : current_position;
    target_ = target;
    reached_ = ( from - target_->position ).norm() < proximity_;
    wayline_ = reached_ ? snark::control::wayline_t() : snark::control::wayline_t( from, target_->position );
}

void wayline_follower::update( const feedback_t& feedback )
{
    if( reached_ ) { return; }
    reached_ = ( ( feedback.position - target_->position ).norm() < proximity_ )
        || ( use_past_endpoint_ && wayline_.is_past_endpoint( feedback.position ) );
    error_.cross_track = wayline_.cross_track_error( feedback.position );
    error_.heading = target_->is_absolute ? snark::control::wrap_angle( target_->heading_offset - feedback.yaw )
        : wayline_.heading_error( feedback.yaw, target_->heading_offset );
}

} } // namespace snark { namespace control {