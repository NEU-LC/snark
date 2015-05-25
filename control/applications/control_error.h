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

#ifndef SNARK_CONTROL_ERROR_H
#define SNARK_CONTROL_ERROR_H

#include <cmath>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/static_assert.hpp>
#include <comma/visiting/traits.h>
#include <snark/visiting/eigen.h>
#include <snark/timing/timestamped.h>

static const unsigned int dimensions = 2;
typedef Eigen::Matrix< double, dimensions, 1 > vector_t;
double distance( const vector_t& p1, const vector_t& p2 ) { return ( p1 - p2 ).norm(); }
vector_t normalise( const vector_t& v ) { return v.normalized(); }
std::string serialise( const vector_t& p )
{
    std::stringstream s;
    s << p.x() << ',' << p.y();
    return s.str();
}

struct orientation_t
{
    double yaw;
};

struct position_t
{
    vector_t location;
    orientation_t orientation;
};

typedef snark::timestamped< position_t > feedback_t;

struct decoration_t
{
    double speed;
};

struct input_t
{
    vector_t location;
    decoration_t decoration;
};

struct control_error_t
{
    double cross_track;
    double heading;
};


class wayline_t
{
    vector_t v;
    double heading;
    Eigen::Hyperplane< double, dimensions > line;
    Eigen::Hyperplane< double, dimensions > perpendicular_line_at_end;

public:
    wayline_t( bool verbose = false ) {}
    wayline_t( const vector_t& start, const vector_t& end, bool verbose = false ) :
          v( normalise( end - start ) )
        , heading( atan2( v.y(), v.x() ) )
        , line( Eigen::ParametrizedLine< double, dimensions >::Through( start, end ) )
        , perpendicular_line_at_end( v, end )
        {
            BOOST_STATIC_ASSERT( dimensions == 2 );
            if( verbose ) { std::cerr << "wayline from " << serialise( start ) << " to " << serialise( end ) << std::endl; }
        }
    bool is_past_endpoint( const vector_t& location ) { return perpendicular_line_at_end.signedDistance( location ) > 0; }
    double cross_track_error( const vector_t& location ) { return line.signedDistance( location ); }
    double heading_error( double yaw )
    {
        if( yaw < -M_PI || yaw > M_PI ) { COMMA_THROW( comma::exception, "expected yaw within [-pi,pi], got" << yaw ); }
        double diff = yaw - heading;
        if( diff <= -M_PI ) { return diff + 2*M_PI; }
        else if( diff > M_PI ) { return diff - 2*M_PI; }
        else { return diff; }
    }
};

namespace comma { namespace visiting {

template <> struct traits< orientation_t >
{
    template < typename K, typename V > static void visit( const K&, orientation_t& p, V& v )
    {
        v.apply( "yaw", p.yaw );
    }

    template < typename K, typename V > static void visit( const K&, const orientation_t& p, V& v )
    {
        v.apply( "yaw", p.yaw );
    }
};

template <> struct traits< position_t >
{
    template < typename K, typename V > static void visit( const K&, position_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "orientation", p.orientation );
    }

    template < typename K, typename V > static void visit( const K&, const position_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "orientation", p.orientation );
    }
};

template <> struct traits< decoration_t >
{
    template < typename K, typename V > static void visit( const K&, decoration_t& p, V& v )
    {
        v.apply( "speed", p.speed );
    }

    template < typename K, typename V > static void visit( const K&, const decoration_t& p, V& v )
    {
        v.apply( "speed", p.speed );
    }
};

template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K&, input_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "decoration", p.decoration );
    }

    template < typename K, typename V > static void visit( const K&, const input_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "decoration", p.decoration );
    }
};

template <> struct traits< control_error_t >
{
    template < typename K, typename V > static void visit( const K&, control_error_t& p, V& v )
    {
        v.apply( "cross_track", p.cross_track );
        v.apply( "heading", p.heading );
    }

    template < typename K, typename V > static void visit( const K&, const control_error_t& p, V& v )
    {
        v.apply( "cross_track", p.cross_track );
        v.apply( "heading", p.heading );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_CONTROL_ERROR_H