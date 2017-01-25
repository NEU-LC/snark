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

#include <comma/csv/stream.h>
#include "../../../visiting/eigen.h"
#include "projection.h"

namespace snark { namespace points_calc { namespace projection {

struct plane
{
    Eigen::Vector3d point;
    Eigen::Vector3d normal;
    
    plane() : point( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ) {}
    plane( const Eigen::Vector3d& point, const Eigen::Vector3d& normal ) : point( point ), normal( normal ) {}
};

struct line
{
    typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > pair;
    
    Eigen::Vector3d point;
    Eigen::Vector3d vector;
    
    line() : point( Eigen::Vector3d::Zero() ), vector( Eigen::Vector3d::Zero() ) {}
    line( const Eigen::Vector3d& point, const Eigen::Vector3d& vector ) : point( point ), vector( vector ) {}
    line( const pair& p ) : point( p.first ), vector( p.second - p.first ) {}
};

namespace onto_plane {
    
struct input : public Eigen::Vector3d
{
    projection::plane plane;
    
    input() : Eigen::Vector3d( Eigen::Vector3d::Zero() ) {}
    input( const Eigen::Vector3d& v, const projection::plane& p ) : Eigen::Vector3d( v ), plane( p ) {}
};

struct output : public Eigen::Vector3d
{
    double dot;
    output() : Eigen::Vector3d( Eigen::Vector3d::Zero() ), dot( 0 ) {}
    output( const Eigen::Vector3d& v, double dot ) : Eigen::Vector3d( v ), dot( dot ) {}
};

} // namespace onto_plane {

namespace onto_line {

template < typename Line = projection::line > struct input : public Eigen::Vector3d
{
    Line line;
    
    input() : Eigen::Vector3d( Eigen::Vector3d::Zero() ) {}
    input( const Eigen::Vector3d& v, const Line& line ) : Eigen::Vector3d( v ), line( line ) {}
};

struct output : public Eigen::Vector3d
{
    enum where_values { before = -1, inside = 0, after = 1 };
    double distance;
    comma::int32 where;
    output() : Eigen::Vector3d( Eigen::Vector3d::Zero() ), distance( 0 ), where( 0 ) {}
    output( const Eigen::Vector3d& v, double distance = 0, comma::int32 where = inside ) : Eigen::Vector3d( v ), distance( distance ), where( where ) {}
};

} // namespace onto_line {
    
} } } // namespace snark { namespace points_calc { namespace projection {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::projection::plane >
{
    template< typename K, typename V > static void visit( const K&, snark::points_calc::projection::plane& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "normal", t.normal );
    }
    
    template< typename K, typename V > static void visit( const K&, const snark::points_calc::projection::plane& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "normal", t.normal );
    }
};

template <> struct traits< snark::points_calc::projection::onto_plane::input >
{
    template< typename K, typename V > static void visit( const K& k, snark::points_calc::projection::onto_plane::input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "plane", t.plane );
    }
    
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::projection::onto_plane::input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "plane", t.plane );
    }
};

template <> struct traits< snark::points_calc::projection::onto_plane::output >
{
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::projection::onto_plane::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "dot", t.dot );
    }
};

template <> struct traits< snark::points_calc::projection::line >
{
    template< typename K, typename V > static void visit( const K& k, snark::points_calc::projection::line& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "vector", t.vector );
    }
    
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::projection::line& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "vector", t.vector );
    }
};

template < typename Line > struct traits< snark::points_calc::projection::onto_line::input< Line > >
{
    template< typename K, typename V > static void visit( const K& k, typename snark::points_calc::projection::onto_line::input< Line >& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "line", t.line );
    }
    
    template< typename K, typename V > static void visit( const K& k, const typename snark::points_calc::projection::onto_line::input< Line >& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "line", t.line );
    }
};

template <> struct traits< snark::points_calc::projection::onto_line::output >
{
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::projection::onto_line::output& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "distance", t.distance );
        v.apply( "where", t.where );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace points_calc {

static Eigen::Vector3d vector_( const std::string& option, const comma::command_line_options& options )
{
    static const comma::csv::ascii< Eigen::Vector3d > ascii;
    return ascii.get( options.value< std::string >( option, "0,0,0" ) );
}

} } // namespace snark { namespace points_calc {

namespace snark { namespace points_calc { namespace projection { namespace onto_plane {

std::string traits::input_fields() { return comma::join( comma::csv::names< input >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output >( true ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }

std::string traits::usage()
{
    std::ostringstream oss;
    oss << "    projection-onto-plane: output projection of point onto plane and signed distance (dot product of projection vector and plane normal)" << std::endl
        << "        options" << std::endl
        << "            --v=<x,y,z>: point to project" << std::endl
        << "            --plane-point,--point=<x,y,z>: a point on the plane" << std::endl
        << "            --plane-normal,--normal=<x,y,z>: normal to the plane" << std::endl;
    return oss.str();
}

int traits::run( const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    csv.full_xpath = true;
    comma::csv::input_stream< input > istream( std::cin, csv, input( vector_( "--v", options ), plane( vector_( "--plane-point,--point", options ), vector_( "--plane-normal,--normal", options ) ) ) );
    comma::csv::options output_csv;
    output_csv.full_xpath = true;
    if( csv.binary() ) { output_csv.format( output_format() ); }
    comma::csv::output_stream< output > ostream( std::cout, output_csv );
    comma::csv::tied< input, output > tied( istream, ostream );
    while( istream.ready() || std::cin.good() )
    {
        const input* p = istream.read();
        if( !p ) { break; }
        const Eigen::Vector3d& n = p->plane.normal.normalized();
        double dot = ( *p - p->plane.point ).dot( n );
        tied.append( output( *p - n * dot, dot ) );
    }
    return 0;
}

} } } } // namespace snark { namespace points_calc { namespace projection { namespace onto_plane {

namespace snark { namespace points_calc { namespace projection { namespace onto_line {

std::string traits::input_fields() { return comma::join( comma::csv::names< input< projection::line > >( true ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< input< projection::line > >(); }
    
std::string traits::output_fields() { return comma::join( comma::csv::names< output >( true ), ',' ); }

std::string traits::output_format() { return comma::csv::format::value< output >(); }

std::string traits::usage()
{
    std::ostringstream oss;
    oss << "    projection-onto-line: output projection of point onto line, distance to it, and flag indicating where projection is relative to the points defining line" << std::endl
        << "        options" << std::endl
        << "            --v=<x,y,z>: point to project" << std::endl
        << "            --line-point,--point,--line-first,--first=<x,y,z>: first point on the line" << std::endl
        << "            --line-second,--second=<x,y,z>: second point of the line" << std::endl
        << "            --line-vector,--vector=<x,y,z>: vector parallel to line" << std::endl
        << "        input fields" << std::endl
        << "            point,vector line definition (default): " << comma::join( comma::csv::names< input< projection::line > >( true ), ',' ) << std::endl
        << "            first,second point line definition: " << comma::join( comma::csv::names< input< projection::line::pair > >( true ), ',' ) << std::endl;
    return oss.str();
}

template < typename Line > static int run_impl( const comma::csv::options& csv, const input< Line >& sample )
{
    typedef input< Line > input_t;
    comma::csv::input_stream< input_t > istream( std::cin, csv, sample );
    comma::csv::options output_csv;
    output_csv.full_xpath = true;
    if( csv.binary() ) { output_csv.format( traits::output_format() ); }
    comma::csv::output_stream< output > ostream( std::cout, output_csv );
    comma::csv::tied< input_t, output > tied( istream, ostream );
    while( istream.ready() || std::cin.good() )
    {
        const input_t* p = istream.read();
        if( !p ) { break; }
        projection::line line( p->line );
        const Eigen::Vector3d& n = line.vector.normalized();
        double dot = ( line.point - *p ).dot( n );
        const Eigen::Vector3d& intersection = line.point - n * dot;
        output o( intersection, ( *p - intersection ).norm() );
        
        // todo: fill o.where
        
        tied.append( o );
    }
    return 0;
}

int traits::run( const comma::command_line_options& options )
{
    options.assert_mutually_exclusive( "--line-second,--second", "--line-vector,--vector" );
    
    comma::csv::options csv( options );
    csv.full_xpath = true;
    const std::vector< std::string >& v = comma::split( csv.fields, ',' );
    bool as_pair = false;
    for( unsigned int i = 0; i < v.size() && !as_pair; ++i ) { as_pair =    v[i] == "line/first" || v[i] == "line/first/x" || v[i] == "line/first/y" || v[i] == "line/first/z" || v[i] == "line/second" || v[i] == "line/second/x" || v[i] == "line/second/y" || v[i] == "line/second/z"; }
    projection::line line;
    line.point = vector_( "--line-point,--point,--line-first,--first", options );
    line.vector = options.exists( "--line-vector,--vector" ) ? vector_( "--line-vector,--vector", options ) : ( vector_( "--line-second,--second", options ) - line.point );
    input<> sample( vector_( "--v", options ), line );
    if( as_pair ) { return run_impl< projection::line::pair >( csv, input< projection::line::pair >( sample, std::make_pair( sample.line.point, sample.line.point + sample.line.vector ) ) ); }
    else { return run_impl< projection::line >( csv, sample ); }
}

} } } } // namespace snark { namespace points_calc { namespace projection { namespace onto_line {
