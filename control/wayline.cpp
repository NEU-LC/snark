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
#include "wayline.h"
#include <comma/math/cyclic.h>

namespace snark { namespace control {

wayline::position_t normalise( const wayline::position_t& v ) { return v.normalized(); }

wayline::wayline( const position_t& from, const position_t& to )
    : v_( ( to - from ).normalized() )
    , heading_( atan2( v_.y(), v_.x() ) )
    , line_( Eigen::ParametrizedLine< double, dimensions >::Through( from, to ) )
    , perpendicular_line_at_end_( v_, to )
    {
        BOOST_STATIC_ASSERT( dimensions == 2 );
    }

bool wayline::is_past_endpoint( const position_t& position ) const
{
    return perpendicular_line_at_end_.signedDistance( position ) > 0;
}

double wayline::cross_track_error( const position_t& position ) const
{
    return -line_.signedDistance( position );
}

double wayline::heading_error( double current_heading, double target_heading ) const
{
    return comma::math::cyclic< double >( comma::math::interval< double >( -M_PI, M_PI ), heading_ + target_heading - current_heading )();
}

} } // namespace snark { namespace control {