// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2018 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#include <array>
#include <cmath>
#include <functional>
#include <sstream>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include "../../../math/geometry/polygon.h"
#include "../../../math/geometry/traits.h"
#include "triangles_discretise.h"

namespace snark { namespace points_calc { namespace triangles_discretise {

std::string traits::usage()
{
    std::ostringstream oss;
    oss
        << "    triangles-discretise: read triangles on stdin, append/output points that reasonably cover the triangle\n"
        << "                          (currently pretty quick and dirty)\n"
        << "        fields\n"
        << "            default input: \"corners\"\n"
        << "            default output: \"x,y,z,internal\"\n"
        << "        options\n"
        << "            --output-points-only,--points-only,--discard-input; output points, don't output input triangles\n"
        << "            --output-internal-only,--internal-only; output only points inside triangles\n"
        << "            --output-sides-only,--sides-only; output points only on the sides of triangles\n"
        << "            --radius,-r=<meters>; max distance between the points\n"
        << "            --tolerance,-t=[<meters>]; min distance between the points, todo\n"
        << std::endl
        << "        examples\n"
        << "            echo 0,0,0,0,0,10,0,10,0 \\\n"
        << "                | points-calc triangles-discretise --fields corners \\\n"
        << "                | view-points --fields ,,,,,,,,,x,y,z,id\n"
        << "            echo 0,0,0,0,0,10,0,10,0 \\\n"
        << "                | points-calc triangles-discretise --fields corners --discard-input \\\n"
        << "                | view-points --fields x,y,z,id\n"
        << std::endl;
    return oss.str();    
}

struct input
{
    snark::triangle triangle;
    comma::uint32 id;
    input() : id( 0 ) {}
};

struct output
{
    Eigen::Vector3d point;
    bool internal;
    comma::uint32 id;
    output( const Eigen::Vector3d& point = Eigen::Vector3d::Zero(), bool internal = false, comma::uint32 id = 0 ): point( point ), internal( internal ), id( id ) {}
};

} } } // namespace snark { namespace points_calc { namespace triangles_discretise {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::triangles_discretise::input >
{
    template < typename K, typename V > static void visit( const K& k, snark::points_calc::triangles_discretise::input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.triangle, v );
        v.apply( "id", t.id );
    }
    
    template < typename K, typename V > static void visit( const K& k, const snark::points_calc::triangles_discretise::input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.triangle, v );
        v.apply( "id", t.id );
    }
};

template <> struct traits< snark::points_calc::triangles_discretise::output >
{
    template < typename K, typename V > static void visit( const K& k, snark::points_calc::triangles_discretise::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.point, v );
        v.apply( "internal", t.internal );
        v.apply( "id", t.id );
    }
    
    template < typename K, typename V > static void visit( const K& k, const snark::points_calc::triangles_discretise::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.point, v );
        v.apply( "internal", t.internal );
        v.apply( "id", t.id );
    }
};

} }

namespace snark { namespace points_calc { namespace triangles_discretise {

std::string traits::input_fields() { return comma::join( comma::csv::names< input >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output >( false ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }

int traits::run( const comma::command_line_options& options )
{
    double radius = options.value< double >( "--radius,-r" );
    auto tolerance = options.optional< double >( "--tolerance,-t" );
    if( tolerance ) { std::cerr << "points-calc: triangles-discretise: --tolerance: todo" << std::endl; return 1; }
    comma::csv::options csv( options );
    if( csv.fields.empty() ) { csv.fields = "corners"; }
    comma::csv::options output_csv;
    output_csv.flush = csv.flush;
    output_csv.full_xpath = false;
    bool discard_input = options.exists( "--output-points-only,--points-only,--discard-input" );
    bool output_sides = !options.exists( "--output-internal-only,--internal-only" );
    bool output_internal = !options.exists( "--output-sides-only,--sides-only" );
    if( csv.has_field( "id" ) )
    {
        if( csv.binary() ) { output_csv.format( "3d,b,ui" ); }
    }
    else
    {
        output_csv.fields = "x,y,z,internal";
        if( csv.binary() ) { output_csv.format( "3d,b" ); }
    }
    comma::csv::input_stream< input > istream( std::cin, csv );
    comma::csv::output_stream < output > ostream( std::cout, output_csv );
    comma::csv::tied< input, output > tied( istream, ostream );
    std::function< void( const output& ) > do_output = [&]( const output& o ) { tied.append( o ); };
    if( discard_input ) { do_output = [&]( const output& o ) { ostream.write( o ); }; }
    auto discretise_line = [&]( const Eigen::Vector3d& begin, const Eigen::Vector3d& end, bool internal, comma::uint32 id )
    {
        const auto& line = end - begin;
        const auto& norm = line.norm();
        const auto& step = line * ( radius / norm );
        for( unsigned int k = 0; radius * k < norm; ++k ) { do_output( output( begin + step * k, internal, id ) ); }
    };
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const input* r = istream.read();
        if( !r ) { break; }
        if( output_sides )
        {
            for( unsigned int i = 0; i < 3; ++i )
            {
                discretise_line( r->triangle.corners[i], r->triangle.corners[ i == 2 ? 0 : i + 1 ], false, r->id );
            }
        }
        if( output_internal )
        {
            const auto& a = r->triangle.corners[1] - r->triangle.corners[0]; // todo? wasteful to calculate sided twice? watch performance
            const auto& b = r->triangle.corners[2] - r->triangle.corners[0]; // todo? wasteful to calculate sided twice? watch performance
            const auto& c = r->triangle.corners[2] - r->triangle.corners[1]; // todo? wasteful to calculate sided twice? watch performance
            double n = std::abs( b.dot( a.cross( a.cross( b ) ).normalized() ) ); // quick and dirty
            double bn = b.norm();
            double cn = c.norm();
            static double s = radius * std::sqrt( 3 ) / 2;
            double bs = ( bn / n ) * s;
            auto bd = b / bn * bs;
            double cs = ( cn / n ) * s;
            auto cd = c / cn * cs;
            for( unsigned int k = 1; bs * k < bn; ++k )
            {
                discretise_line( r->triangle.corners[0] + bd * k, r->triangle.corners[1] + cd * k, true, r->id );
            }
        }
    }
    return 0;
}

} } } // namespace snark { namespace points_calc { namespace triangles_discretise {
