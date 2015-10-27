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

/// @author vsevolod vlaskine

#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include "../../math/geometry/polygon.h"
#include "../../math/geometry/traits.h"
#include "../../math/interval.h"
#include "../../point_cloud/voxel_map.h"
#include "../../visiting/eigen.h"

static void usage( bool more = false )
{
    std::cerr << std::endl;
    std::cerr << "join two point clouds by distance" << std::endl;
    std::cerr << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.1.csv | points-join \"points.2.csv[;<csv options>]\" [<options>] > joined.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    if the second set is not given, for each point output the nearest point in the same set; todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --all: output all points in the given radius instead of the nearest" << std::endl;
    std::cerr << "    --radius=<radius>: lookup radius" << std::endl;
    std::cerr << "    --strict: exit, if nearest point not found" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields: x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "todo: add support for block field" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

bool verbose;
bool strict;
double radius;
double face_depth;
comma::csv::options stdin_csv;
comma::csv::options filter_csv;

// todo: add block field
struct record
{ 
    Eigen::Vector3d value;
    std::string line;
    record() : value( Eigen::Vector3d::Zero() ) {}
    record( const Eigen::Vector3d& value, const std::string& line ) : value( value ), line( line ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const Eigen::Vector3d& rhs ) const { return value; } // watch performance
};

// todo: add block field
struct triangle_record
{ 
    snark::triangle value;
    std::string line;
    triangle_record() {}
    triangle_record( const snark::triangle& value, const std::string& line ) : value( value ), line( line ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const Eigen::Vector3d& rhs ) const // quick and dirty, watch performance
    {
        boost::optional< Eigen::Vector3d > p = value.projection_of( rhs );
        return value.includes( *p ) && !comma::math::less( value.normal().dot( rhs - *p ), face_depth ) ? p : boost::none;
    } 
};

template < typename V > struct traits;

template <> struct traits< Eigen::Vector3d >
{
    typedef record record_t;
    typedef std::vector< const record_t* > voxel_t; // todo: is vector a good container? use deque
    typedef snark::voxel_map< voxel_t, 3 > grid_t;
    static Eigen::Vector3d default_value() { return Eigen::Vector3d::Zero(); }
    static void set_hull( snark::math::closed_interval< double, 3 >& extents, const Eigen::Vector3d& p ) { extents.set_hull( p ); }
    static bool touch( grid_t& grid, const record_t& record ) { ( grid.touch_at( record.value ) )->second.push_back( &record ); return true; }
    template < typename S > static void output( const S& istream, const record_t& nearest, const Eigen::Vector3d& nearest_point )
    {
        if( stdin_csv.binary() ) // quick and dirty
        {
            std::cout.write( istream.binary().last(), stdin_csv.format().size() );
            std::cout.write( &nearest.line[0], filter_csv.format().size() );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter ) << stdin_csv.delimiter;
            if( filter_csv.binary() ) { std::cout << filter_csv.format().bin_to_csv( &nearest.line[0], stdin_csv.delimiter, stdin_csv.precision ) << std::endl; }
            else { std::cout << nearest.line << std::endl; }
        }
    }
};

template <> struct traits< snark::triangle >
{
    typedef triangle_record record_t;
    typedef std::vector< const record_t* > voxel_t; // todo: is vector a good container? use deque
    typedef snark::voxel_map< voxel_t, 3 > grid_t;
    static snark::triangle default_value() { return snark::triangle(); }
    static void set_hull( snark::math::closed_interval< double, 3 >& extents, const snark::triangle& t )
    { 
        extents.set_hull( t.corners[0] );
        extents.set_hull( t.corners[1] );
        extents.set_hull( t.corners[2] );
    }
    static bool touch( grid_t& grid, const record_t& record )
    {
        if( record.value.circumscribing_radius() > radius )
        {
            if( verbose || strict ) { std::cerr << "points-join: expected triangles that fit into voxels; got: " << std::endl << record.value.corners[0].transpose() << ";" << record.value.corners[1].transpose() << ";" << record.value.corners[2].transpose() << std::endl; }
            return false;
        }
        typename grid_t::iterator i0 = grid.touch_at( record.value.corners[0] );
        typename grid_t::iterator i1 = grid.touch_at( record.value.corners[1] );
        typename grid_t::iterator i2 = grid.touch_at( record.value.corners[2] );
        i0->second.push_back( &record );
        if( i1 != i0 ) { i1->second.push_back( &record ); }
        if( i2 != i0 && i2 != i1 ) { i2->second.push_back( &record ); }
        return true;
    }
    template < typename S > static void output( const S& istream, const record_t& nearest, const Eigen::Vector3d& nearest_point )
    {
        if( stdin_csv.binary() ) // quick and dirty
        {
            std::cout.write( istream.binary().last(), stdin_csv.format().size() );
            std::cout.write( &nearest.line[0], filter_csv.format().size() );
            static comma::csv::binary< Eigen::Vector3d > b;
            static std::vector< char > buf( b.format().size() );
            b.put( nearest_point, &buf[0] );
            std::cout.write( &buf[0], buf.size() );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter ) << stdin_csv.delimiter;
            if( filter_csv.binary() ) { std::cout << filter_csv.format().bin_to_csv( &nearest.line[0], stdin_csv.delimiter, stdin_csv.precision ); }
            else { std::cout << nearest.line; }
            std::cout << stdin_csv.delimiter << nearest_point.x() << stdin_csv.delimiter << nearest_point.y() << stdin_csv.delimiter << nearest_point.z() << std::endl;
        }
    }
};

template < typename V > static int run( const comma::command_line_options& options )
{
    typedef V filter_value_t;
    typedef typename traits< V >::record_t filter_record_t;
    std::ifstream ifs( &filter_csv.filename[0] );
    if( !ifs.is_open() ) { std::cerr << "points-join: failed to open \"" << filter_csv.filename << "\"" << std::endl; }
    strict = options.exists( "--strict" );
    radius = options.value< double >( "--radius" );
    face_depth = options.value< double >( "--face-depth", 0 );
    bool all = options.exists( "--all" );
    Eigen::Vector3d resolution( radius, radius, radius );
    comma::csv::input_stream< V > ifstream( ifs, filter_csv, traits< V >::default_value() );
    std::deque< filter_record_t > filter_points;
    snark::math::closed_interval< double, 3 > extents;
    if( verbose ) { std::cerr << "points-join: reading input points..." << std::endl; }
    while( ifstream.ready() || ( ifs.good() && !ifs.eof() ) )
    {
        const filter_value_t* p = ifstream.read();
        if( !p ) { break; }
        std::string line;
        if( filter_csv.binary() ) // quick and dirty
        {
            line.resize( filter_csv.format().size() );
            ::memcpy( &line[0], ifstream.binary().last(), filter_csv.format().size() );
        }
        else
        {
            line = comma::join( ifstream.ascii().last(), filter_csv.delimiter );
        }
        filter_points.push_back( filter_record_t( *p, line ) );
        traits< V >::set_hull( extents, *p );
    }
    if( verbose ) { std::cerr << "points-join: loading " << filter_points.size() << " records into grid..." << std::endl; }
    typedef typename traits< V >::grid_t grid_t;
    grid_t grid( extents.min(), resolution );
    for( std::size_t i = 0; i < filter_points.size(); ++i ) { if( !traits< V >::touch( grid, filter_points[i] ) && strict ) { return 1; } }
    if( verbose ) { std::cerr << "points-join: joining..." << std::endl; }
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, stdin_csv, Eigen::Vector3d::Zero() );
    #ifdef WIN32
    if( stdin_csv.binary() ) { _setmode( _fileno( stdout ), _O_BINARY ); }
    #endif
    if( !stdin_csv.binary() ) { std::cout.precision( stdin_csv.precision ); }
    std::size_t count = 0;
    std::size_t discarded = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        typename grid_t::index_type index = grid.index_of( *p );
        typename grid_t::index_type i;
        const filter_record_t* nearest = NULL;
        boost::optional< Eigen::Vector3d > nearest_point;
        double distance = 0;
        for( i[0] = index[0] - 1; i[0] < index[0] + 2; ++i[0] )
        {
            for( i[1] = index[1] - 1; i[1] < index[1] + 2; ++i[1] )
            {
                for( i[2] = index[2] - 1; i[2] < index[2] + 2; ++i[2] )
                {
                    typename grid_t::iterator it = grid.find( i );
                    if( it == grid.end() ) { continue; }
                    for( std::size_t k = 0; k < it->second.size(); ++k )
                    {
                        nearest_point = it->second[k]->nearest_to( *p ); // todo: fix! currently, visiting each triangle 3 times
                        if( !nearest_point ) { continue; }
                        double norm = ( *p - *nearest_point ).norm();
                        if( norm > radius ) { continue; }
                        if( all )
                        {
                            if( stdin_csv.binary() )
                            {
                                std::cout.write( istream.binary().last(), stdin_csv.format().size() );
                                std::cout.write( &it->second[k]->line[0], filter_csv.format().size() );
                            }
                            else
                            {
                                std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter ) << stdin_csv.delimiter;
                                if( filter_csv.binary() ) { std::cout << filter_csv.format().bin_to_csv( &it->second[k]->line[0], stdin_csv.delimiter, stdin_csv.precision ) << std::endl; }
                                else { std::cout << &it->second[k]->line[0] << std::endl; }
                            }
                        }
                        else
                        {
                            if( nearest && distance < norm ) { continue; }
                            nearest = it->second[k];
                            distance = norm;
                        }
                    }
                }
            }
        }
        if( !all )
        {
            if( !nearest )
            {
                if( verbose ) { std::cerr.precision( 12 ); std::cerr << "points-join: record " << count << " at " << p->x() << "," << p->y() << "," << p->z() << ": no matches found" << std::endl; }
                if( strict ) { return 1; }
                ++discarded;
                continue;
            }
            traits< V >::output( istream, *nearest, *nearest_point );
        }
        ++count;
    }
    std::cerr << "points-join: processed " << count << " records; discarded " << discarded << " record" << ( count == 1 ? "" : "s" ) << " with no matches" << std::endl;
    return 0;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        stdin_csv = comma::csv::options( options );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v,--strict,--all", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-join: please specify the second source; self-join: todo" << std::endl; return 1; }
        if( unnamed.size() > 1 ) { std::cerr << "points-join: expected one file or stream to join, got " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        comma::name_value::parser parser( "filename", ';', '=', false );
        filter_csv = parser.get< comma::csv::options >( unnamed[0] );
        filter_csv.full_xpath = true;
        if( stdin_csv.binary() && !filter_csv.binary() ) { std::cerr << "points-join: stdin stream binary and filter stream ascii: this combination is not supported" << std::endl; return 1; }
        const std::vector< std::string >& v = comma::split( filter_csv.fields, ',' );
        bool filter_triangulated = false;
        for( unsigned int i = 0; !filter_triangulated && i < v.size(); ++i ) { filter_triangulated = v[i].substr( 0, ::strlen( "corners" ) ) == "corners"; }
        return filter_triangulated ? run< snark::triangle >( options )
                                   : run< Eigen::Vector3d >( options );
    }
    catch( std::exception& ex ) { std::cerr << "points-join: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-join: unknown exception" << std::endl; }
    return 1;
}
