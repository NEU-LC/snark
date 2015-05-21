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
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>
#include <snark/visiting/eigen.h>
#include <comma/csv/traits.h>
#include <snark/visiting/eigen.h>
#include <snark/timing/timestamped.h>

static const unsigned int dimensions = 2;
typedef Eigen::Matrix< double, dimensions, 1 > coordinates_t;
std::string serialise( const coordinates_t& c )
{
    std::stringstream s;
    s << c.x() << ',' << c.y();
    return s.str();
}

typedef Eigen::ParametrizedLine< double, dimensions > line_t;
typedef Eigen::Hyperplane< double, dimensions > hyperplane_t;
bool is_past_endpoint( coordinates_t current, coordinates_t start, coordinates_t end )
{
    line_t wayline = line_t::Through( start, end );
    coordinates_t v = end - start;
    v.normalize();
    hyperplane_t perp( v, end );
    return perp.signedDistance( current ) > 0;
}

struct orientation_t
{
    double yaw;
};

struct position_t
{
    coordinates_t coordinates;
    orientation_t orientation;
};

typedef snark::timestamped< position_t > feedback_t;

struct decoration_t
{
    double speed;
};

struct input_t
{
    coordinates_t coordinates;
    decoration_t decoration;
};

struct control_error_t
{
    double cross_track;
    double heading;
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
        v.apply( "coordinates", p.coordinates );
        v.apply( "orientation", p.orientation );
    }

    template < typename K, typename V > static void visit( const K&, const position_t& p, V& v )
    {
        v.apply( "coordinates", p.coordinates );
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
        v.apply( "coordinates", p.coordinates );
        v.apply( "decoration", p.decoration );
    }

    template < typename K, typename V > static void visit( const K&, const input_t& p, V& v )
    {
        v.apply( "coordinates", p.coordinates );
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