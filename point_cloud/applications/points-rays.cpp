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

/// @author james underwood
/// @author vsevolod vlaskine

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <cmath>
#include <string.h>
#include <fstream>
#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <tbb/parallel_for.h>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../math/range_bearing_elevation.h"
#include "../../math/spherical_geometry/great_circle.h"
#include "../../point_cloud/voxel_map.h"
#include "../../visiting/traits.h"
//#include <google/profiler.h>

void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "ray-tracing operations" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-rays <operation> [<options>] > points.with-distance.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help; --help --verbose: more help" << std::endl;
    std::cerr << "    --angle-threshold,-a=<value>: angular radius in radians" << std::endl;
    std::cerr << "    --verbose,-v: more debug output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    trace: for each point on stdin output its distance from the point on the ray with the shortest range and id of that point" << std::endl;
    std::cerr << "           if id field is not present in the input, point number in the input will be used" << std::endl;
    std::cerr << "        fields: r,b,e,block: range, bearing, elevation; default: r,b,e" << std::endl;
    std::cerr << "        output fields: id,distance" << std::endl;
    std::cerr << "        output format: ui,d" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    exit( 0 );
}

typedef snark::range_bearing_elevation point_t;

struct input_t
{
    point_t point;
    comma::uint32 id;
    comma::uint32 block;
    const input_t* traced;
    
    input_t() : block( 0 ), traced( NULL ) {}
};

struct trace_output_t
{
    comma::uint32 id;
    double distance;
    
    trace_output_t() : id( 0 ), distance( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K&, const input_t& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "id", t.id );
        v.apply( "block", t.block );
    }
    
    template< typename K, typename V > static void visit( const K&, input_t& t, V& v )
    {
        v.apply( "point", t.point );
        v.apply( "id", t.id );
        v.apply( "block", t.block );
    }
};
    
template <> struct traits< trace_output_t >
{
    template< typename K, typename V > static void visit( const K&, trace_output_t& t, V& v )
    {
        v.apply( "id", t.id );
        v.apply( "distance", t.distance );
    }
};

} } // namespace comma { namespace visiting {

static double distance( const point_t& p, const point_t& q ) // todo: quick and dirty, watch performance
{
    return snark::spherical::great_circle::arc( snark::spherical::coordinates( p.bearing_elevation() ), snark::spherical::coordinates( q.bearing_elevation() ) ).angle();
}

struct cell
{
    std::vector< input_t* > points;
    
    const input_t* trace( const point_t& p, double threshold ) const
    {
        const input_t* min = NULL;
        for( std::size_t i = 0; i < points.size(); ++i )
        {
            if( distance( p, points[i]->point ) > threshold ) { continue; }
            if( !min || points[i]->point.range() < min->point.range() ) { min = points[i]; }
        }
        return min;
    }
};

typedef std::pair< input_t, std::vector< char > > pair_t;
typedef snark::voxel_map< cell, 2 > grid_t;

static void trace_points( const tbb::blocked_range< std::size_t >& range, std::deque< pair_t >& records, const grid_t& grid, double threshold )
{
    for( std::size_t i = range.begin(); i < range.end(); ++i )
    {
        grid_t::const_iterator it = grid.find( grid_t::point_type( records[i].first.point.bearing(), records[i].first.point.elevation() ) );
        if( it == grid.end() )
        { 
            records[i].first.traced = &records[i].first;
        }
        else
        {
            records[i].first.traced = it->second.trace( records[i].first.point, threshold );
            if( !records[i].first.traced ) { records[i].first.traced = &records[i].first; }
        }
    }
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        bool verbose = options.exists( "--verbose,-v" );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-rays: please specify operation" << std::endl; return 1; }
        const std::string& operation = unnamed[0];
        if( operation == "trace" )
        {
            comma::csv::options csv( options, "range,bearing,elevation" );
            csv.full_xpath = false;
            std::vector< std::string > v = comma::split( csv.fields, ',' );
            bool has_id = csv.has_field( "id" );
            for( unsigned int i = 0; i < v.size(); ++i )
            {
                if( v[i] == "r" ) { v[i] = "range"; }
                else if( v[i] == "b" ) { v[i] = "bearing"; }
                else if( v[i] == "e" ) { v[i] = "elevation"; }
            }
            csv.fields = comma::join( v, ',' );
            csv.full_xpath = false;
            double threshold = options.value< double >( "--angle-threshold,-a" );
            comma::csv::input_stream< input_t > istream( std::cin, csv );
            snark::voxel_map< int, 2 >::point_type resolution = grid_t::point_type( threshold, threshold );
            comma::uint32 id = 0;
            boost::optional< pair_t > last;
            while( istream.ready() || std::cin.good() )
            {
                grid_t grid( resolution );
                if( verbose ) { std::cerr << "points-rays: block" << ( last ? boost::lexical_cast< std::string >( last->first.block ) : std::string() ) << ": loading..." << std::endl; }
                std::deque< pair_t > records;
                if( last ) { records.push_back( *last ); }
                while( istream.ready() || std::cin.good() )
                {
                    const input_t* p = istream.read();
                    if( !p ) { break; }
                    if( !last ) { last = pair_t(); }
                    last->first = *p;
                    if( !has_id ) { last->first.id = id; }
                    if( csv.binary() ) // quick and dirty
                    {
                        static unsigned int s = istream.binary().binary().format().size();
                        last->second.resize( s );
                        ::memcpy( &last->second[0], istream.binary().last(), s );
                    }
                    else
                    {
                        std::string s = comma::join( istream.ascii().last(), csv.delimiter );
                        last->second.resize( s.size() );
                        ::memcpy( &last->second[0], &s[0], s.size() );
                    }
                    if( !records.empty() && records.back().first.block != p->block ) { break; }
                    records.push_back( *last ); // todo: quick and dirty; watch performance
                    for( int i = -1; i < 2; ++i )
                    {
                        for( int j = -1; j < 2; ++j )
                        {
                            double bearing = records.back().first.point.bearing() + threshold * i;
                            if( bearing < -M_PI ) { bearing += ( M_PI * 2 ); }
                            else if( bearing >= M_PI ) { bearing -= ( M_PI * 2 ); }
                            double elevation = records.back().first.point.elevation() + threshold * j;
                            grid_t::iterator it = grid.touch_at( grid_t::point_type( bearing, elevation ) );
                            it->second.points.push_back( &records.back().first );
                        }
                    }
                    ++id;
                }
                if( records.empty() ) { break; }
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": loaded " << records.size() << " points in a grid of size " << grid.size() << " voxels" << std::endl; }
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": tracing..." << std::endl; }
                tbb::parallel_for( tbb::blocked_range< std::size_t >( 0, records.size(), records.size() / 4 ), boost::bind( &trace_points, _1, boost::ref( records ), boost::cref( grid ), threshold ) );
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": outputting..." << std::endl; }
                for( std::size_t i = 0; i < records.size(); ++i )
                {
                    double distance = records[i].first.point.range() - records[i].first.traced->point.range();
                    std::cout.write( &records[i].second[0], records[i].second.size() );
                    if( csv.binary() )
                    {
                        std::cout.write( reinterpret_cast< const char* >( &records[i].first.traced->id ), sizeof( comma::uint32 ) );
                        std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
                    }
                    else
                    {
                        std::cout << csv.delimiter << records[i].first.traced->id << csv.delimiter << distance << std::endl;
                    }
                }
                if( csv.flush ) { std::cout.flush(); }
                if( verbose ) { std::cerr << "points-rays: block " << records[0].first.block << ": done" << std::endl; }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-rays: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-rays: unknown exception" << std::endl; }
    return 1;
}
