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

#ifndef SNARK_CONTROL_H
#define SNARK_CONTROL_H

#include <cmath>
#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/static_assert.hpp>
#include <boost/bimap.hpp>
#include <boost/assign.hpp>
#include <comma/visiting/traits.h>
#include <comma/base/exception.h>
#include <comma/math/cyclic.h>
#include <snark/visiting/eigen.h>
#include <snark/timing/timestamped.h>

namespace snark { namespace control {

static std::string command_app_name = "control-command";
static std::string error_app_name = "control-error";

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

struct parameters_t
{
    parameters_t() : speed( 0 ), heading_offset( 0 ) {}
    double speed;
    double heading_offset;
};

struct target_t
{
    vector_t location;
    parameters_t parameters;
};

struct error_t
{
    error_t() : cross_track( 0 ), heading( 0 ) {}
    double cross_track;
    double heading;
};

double wrap_angle( double value ) { return comma::math::cyclic< double >( comma::math::interval< double >( -M_PI, M_PI ), value )(); }

struct wayline_t
{
public:
    wayline_t() {}
    wayline_t( const vector_t& start, const vector_t& end, bool verbose = false ) :
          v( normalise( end - start ) )
        , heading( atan2( v.y(), v.x() ) )
        , line( Eigen::ParametrizedLine< double, dimensions >::Through( start, end ) )
        , perpendicular_line_at_end( v, end )
        {
            BOOST_STATIC_ASSERT( dimensions == 2 );
            if( verbose ) { std::cerr << "wayline from " << serialise( start ) << " to " << serialise( end ) << std::endl; }
        }
    bool is_past_endpoint( const vector_t& location ) const { return perpendicular_line_at_end.signedDistance( location ) > 0; }
    double cross_track_error( const vector_t& location ) const { return line.signedDistance( location ); }
    double heading_error( double yaw, double heading_offset ) const { return wrap_angle( yaw - heading - heading_offset ); }
    double get_heading() const { return heading; }
    void set_heading( double h ) { heading = h; }
private:
    vector_t v;
    double heading;
    Eigen::Hyperplane< double, dimensions > line;
    Eigen::Hyperplane< double, dimensions > perpendicular_line_at_end;
};

struct control_data_t
{
    control_data_t() {}
    control_data_t( const target_t& target, const wayline_t& wayline, const feedback_t& feedback, const error_t& error ) : target( target ), wayline( wayline ), feedback( feedback ), error( error ) {}
    target_t target;
    wayline_t wayline;
    feedback_t feedback;
    error_t error;
};

struct command_t
{
    command_t() : turn_rate( 0 ), local_heading( 0 ) {}
    double turn_rate;
    double local_heading;
};

} } // namespace snark { namespace control

namespace comma { namespace visiting {

template <> struct traits< snark::control::orientation_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::orientation_t& p, V& v )
    {
        v.apply( "yaw", p.yaw );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::orientation_t& p, V& v )
    {
        v.apply( "yaw", p.yaw );
    }
};

template <> struct traits< snark::control::position_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::position_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "orientation", p.orientation );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::position_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "orientation", p.orientation );
    }
};

template <> struct traits< snark::control::parameters_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::parameters_t& p, V& v )
    {
        v.apply( "speed", p.speed );
        v.apply( "heading_offset", p.heading_offset );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::parameters_t& p, V& v )
    {
        v.apply( "speed", p.speed );
        v.apply( "heading_offset", p.heading_offset );
    }
};

template <> struct traits< snark::control::target_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::target_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "parameters", p.parameters );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::target_t& p, V& v )
    {
        v.apply( "location", p.location );
        v.apply( "parameters", p.parameters );
    }
};

template <> struct traits< snark::control::error_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::error_t& p, V& v )
    {
        v.apply( "cross_track", p.cross_track );
        v.apply( "heading", p.heading );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::error_t& p, V& v )
    {
        v.apply( "cross_track", p.cross_track );
        v.apply( "heading", p.heading );
    }
};

template <> struct traits< snark::control::wayline_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::wayline_t& p, V& v )
    {
        double heading;
        v.apply( "heading", heading );
        p.set_heading( heading );
    }
    template < typename K, typename V > static void visit( const K&, const snark::control::wayline_t& p, V& v )
    {
        v.apply( "heading", p.get_heading() );
    }
};

template <> struct traits< snark::control::control_data_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::control_data_t& p, V& v )
    {
        v.apply( "target", p.target );
        v.apply( "wayline", p.wayline );
        v.apply( "feedback", p.feedback );
        v.apply( "error", p.error );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::control_data_t& p, V& v )
    {
        v.apply( "target", p.target );
        v.apply( "wayline", p.wayline );
        v.apply( "feedback", p.feedback );
        v.apply( "error", p.error );
    }
};

template <> struct traits< snark::control::command_t >
{
    template < typename K, typename V > static void visit( const K&, snark::control::command_t& p, V& v )
    {
        v.apply( "turn_rate", p.turn_rate );
        v.apply( "local_heading", p.local_heading );
    }

    template < typename K, typename V > static void visit( const K&, const snark::control::command_t& p, V& v )
    {
        v.apply( "turn_rate", p.turn_rate );
        v.apply( "local_heading", p.local_heading );
    }
};

} } // namespace comma { namespace visiting {

#endif // SNARK_CONTROL_H