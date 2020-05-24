// Copyright (c) 2015 The University of Sydney

#include <type_traits>
#include <comma/math/cyclic.h>
#include "wayline.h"

namespace snark { namespace control {

wayline::wayline( const position_t& from, const position_t& to )
    : v_( ( to - from ).normalized() )
    , heading_( atan2( v_.y(), v_.x() ) )
    , line_( Eigen::ParametrizedLine< double, dimensions >::Through( from, to ) )
    , perpendicular_line_at_end_( v_, to )
    {
        static_assert( dimensions == 2, "expected dimensions of 2" );
    }

double wayline::endpoint_overshoot( const position_t& position ) const
{
    return perpendicular_line_at_end_.signedDistance( position );
}

bool wayline::is_past_endpoint( const position_t& position ) const
{
    return endpoint_overshoot( position ) > 0;
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
