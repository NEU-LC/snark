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

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <boost/function.hpp>
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

#ifdef SNARK_USE_CUDA
#include <cuda_runtime.h>
#include "points-join/points_join_cuda.h"
#endif

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
    std::cerr << "    --blocks-ordered: stdin and filter blocks are ordered and therefore missing filter blocks can be handled correctly" << std::endl;
    std::cerr << "    --id-not-matching,--not-matching-id: if id field present in --fields, match only points with different ids" << std::endl;
    std::cerr << "                                         default: if id field present, match points with the same id" << std::endl;
    std::cerr << "    --input-fields: output input fields and exit" << std::endl;
    std::cerr << "    --matching: output only points that have a match, do not append nearest point; a convenience option" << std::endl;
    std::cerr << "    --not-matching: output only points that do not have a match, i.e. opposite of --matching" << std::endl;
    std::cerr << "    --radius=<radius>: max lookup radius, required even if radius field is present" << std::endl;
    std::cerr << "    --radius-min,--min-radius=<radius>: min lookup radius, e.g. to filter out points on poor-man's self-join" << std::endl;
    std::cerr << "    --strict: exit, if nearest point not found" << std::endl;
    std::cerr << "    --permissive: discard invalid points or triangles and continue" << std::endl;
    std::cerr << "    --size,--number-of-points,--number-of-nearest-points=<number_of_points>; default=1: output up to a given number of nearest points in the given radius" << std::endl;
    #ifdef SNARK_USE_CUDA
    std::cerr << "    --use-cuda,--cuda: experimental option; currently 40 times slower then normal operation, thus don't use it, yet" << std::endl;
    #endif
    std::cerr << std::endl;
    std::cerr << "points filter (default): for each input point find the nearest point of the filter in given radius" << std::endl;
    std::cerr << "    input: points; fields: x,y,z,[block],[normal/x,normal/y,normal/z]" << std::endl;
    std::cerr << "    filter: points; fields: x,y,z,[block],[normal/x,normal/y,normal/z]" << std::endl;
    std::cerr << "    output: concatenated input and corresponding line of filter" << std::endl;
    std::cerr << "            if the angle between the input and filter point normals is greater than 90 degrees" << std::endl;
    std::cerr << "            then the filter point will not be considered" << std::endl;
    std::cerr << std::endl;
    std::cerr << "triangulated filter: for each input point find the nearest triangle of the filter, if any, in given radius; i.e." << std::endl;
    std::cerr << "                     nearest point of a triangle is the input point projection onto the triangle plane" << std::endl;
    std::cerr << "                     if the projection is inside of the triangle, border included" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    input fields: x,y,z,[block]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter: triangles; fields: corners; or corners[0],corners[1],corners[2]; or corners[0]/x,or corners[0]/y,or corners[0]/z etc" << std::endl;
    std::cerr << "    output: concatenated input, corresponding line of filter, and nearest point of triangle (3d in binary)" << std::endl;
    std::cerr << "    options" << std::endl;
    std::cerr << "        --max-triangle-side=<value>: triangles with any side longer than <value> will be discarded; default: value of --radius" << std::endl;
    std::cerr << "        --origin=<x>,<y>,<z>: point from which the input points were seen" << std::endl;
    std::cerr << "                              if the angle between the normal of the triangle and the vector from" << std::endl;
    std::cerr << "                              the point to the origin greater or equal 90 degrees, the triangle" << std::endl;
    std::cerr << "                              will not be considered" << std::endl;
    std::cerr << "                              default: 0,0,0" << std::endl;
    std::cerr << std::endl;
    if( more ) { std::cerr << "csv options" << std::endl << comma::csv::options::usage() << std::endl << std::endl; }
    std::cerr << "examples: todo" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

static bool verbose;
static bool strict;
static bool permissive;
static bool use_normal;
static bool use_block;
static bool use_radius;
static double radius;
static double squared_radius;
static double min_radius;
static double squared_min_radius;
static double max_triangle_side;
static bool matching;
static bool append_nearest;
static bool matching_id;
static Eigen::Vector3d origin = Eigen::Vector3d::Zero();
static Eigen::Vector3d resolution;
static comma::csv::options stdin_csv;
static comma::csv::options filter_csv;
static boost::optional< comma::uint32 > block;
#ifdef SNARK_USE_CUDA
bool use_cuda;
void* cuda_buf = NULL;
static void cuda_deallocate()
{
    if( !use_cuda ) { return; }
    if( cuda_buf ) { cudaFree( cuda_buf ); }
    cudaDeviceReset();
}
#endif

struct point_input
{
    Eigen::Vector3d value;
    Eigen::Vector3d normal;
    double radius;
    comma::uint32 block;
    comma::uint32 id;
    point_input() : value( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ), radius( 0 ), block( 0 ), id( 0 ) {}
};

static double get_squared_radius( const point_input& p ) { return use_radius ? p.radius * p.radius : squared_radius; }

struct triangle_input
{
    snark::triangle value;
    comma::uint32 block;
    comma::uint32 id;
    triangle_input() : block( 0 ), id( 0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< point_input >
{
    template< typename K, typename V > static void visit( const K& k, point_input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.value, v );
        v.apply( "normal", t.normal );
        v.apply( "radius", t.radius );
        v.apply( "block", t.block );
        v.apply( "id", t.id );
    }

    template< typename K, typename V > static void visit( const K& k, const point_input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.value, v );
        v.apply( "normal", t.normal );
        v.apply( "radius", t.radius );
        v.apply( "block", t.block );
        v.apply( "id", t.id );
    }
};

template <> struct traits< triangle_input >
{
    template< typename K, typename V > static void visit( const K& k, triangle_input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.value, v );
        v.apply( "block", t.block );
        v.apply( "id", t.id );
    }

    template< typename K, typename V > static void visit( const K& k, const triangle_input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.value, v );
        v.apply( "block", t.block );
        v.apply( "id", t.id );
    }
};

} }

// todo: add block field
struct record
{
    Eigen::Vector3d value;
    Eigen::Vector3d normal;
    comma::uint32 id;
    std::string line;
    record() : value( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ), id( 0 ) {}
    record( const point_input& input, const std::string& line ) : value( input.value ), normal( input.normal ), id( input.id ), line( line ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const point_input& rhs ) const
    {
        if( ( rhs.id == id ) != matching_id ) { return boost::none; }
        if( use_normal ) { return !comma::math::less( normal.dot( rhs.normal ), 0 ) ? boost::make_optional( value ) : boost::none; }
        return value;
    } // watch performance
    bool is_valid() const { return true; }
};

// todo: add block field
struct triangle_record
{
    snark::triangle value;
    comma::uint32 id;
    std::string line;
    triangle_record(): id( 0 ) {}
    triangle_record( const triangle_input& input, const std::string& line ) : value( input.value ), id( input.id ), line( line ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const point_input& rhs ) const // quick and dirty, watch performance
    {
        if( ( rhs.id == id ) != matching_id ) { return boost::none; }
        boost::optional< Eigen::Vector3d > p = value.projection_of( rhs.value );
        return value.includes( *p ) && !comma::math::less( value.normal().dot( origin - rhs.value ), 0 ) ? p : boost::none;
    }
    bool is_valid() const { return value.is_valid(); }
};

template < typename S > static void output_last( const S& istream )
{
    if( stdin_csv.binary() ) { std::cout.write( istream.binary().last(), stdin_csv.format().size() ); }
    else { std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter ) << std::endl; }
}

template < typename V > struct traits;

template <> struct traits< Eigen::Vector3d >
{
    typedef Eigen::Vector3d value_t;
    typedef record record_t;
    typedef point_input input_t;
    struct voxel_t
    {
        std::vector< const record_t* > records;
        #ifdef SNARK_USE_CUDA
            snark::cuda::buffer buffer;
            void calculate_squared_norms( const Eigen::Vector3d& rhs ) { snark::cuda::squared_norms( rhs, buffer ); }
            boost::optional< std::pair< Eigen::Vector3d, double > > nearest_to( const point_input& rhs, unsigned int k ) const { return std::make_pair( records[k]->value, use_cuda ? buffer.out[k] : (records[k]->value - rhs.value ).squaredNorm() ); }
        #else // SNARK_USE_CUDA
            boost::optional< std::pair< Eigen::Vector3d, double > > nearest_to( const point_input& rhs, unsigned int k ) const
            {
                const boost::optional< Eigen::Vector3d >& n = records[k]->nearest_to( rhs );
                if( !n ) { return boost::none; }
                return std::make_pair( *n, ( *n - rhs.value ).squaredNorm() );
            }
        #endif // SNARK_USE_CUDA
    };
    typedef snark::voxel_map< voxel_t, 3 > grid_t;
    struct nearest_t // quick and dirty
    {
        const record_t* record;
        Eigen::Vector3d point;
        double squared_distance;
        nearest_t() : record( NULL ), squared_distance( 0 ) {}
        nearest_t( const record_t* record, const Eigen::Vector3d& point, double squared_distance ): record( record ), point( point ), squared_distance( squared_distance ) {}
    };
    static Eigen::Vector3d default_value() { return Eigen::Vector3d::Zero(); }
    static void set_hull( snark::math::closed_interval< double, 3 >& extents, const Eigen::Vector3d& p ) { extents.set_hull( p ); }
    static bool touch( grid_t& grid, const record_t& record )
    {
        grid_t::iterator i = grid.touch_at( record.value );
        i->second.records.push_back( &record );
        return true;
    }
    #ifdef SNARK_USE_CUDA
    static void* to_cuda( grid_t& grid, const std::deque< record_t >& records )
    {
        if( !use_cuda ) { return NULL; }
        char* buf = NULL;
        cudaError_t err = cudaMalloc( &buf, records.size() * sizeof( double ) * 4 );
        if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "failed to allocate cuda memory for " << records.size() << " records (" << ( records.size() * sizeof( double ) * 4 ) << " bytes); " << cudaGetErrorString( err ) ); }
        char* cur = buf;
        std::vector< double > v;
        for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
        {
            v.resize( it->second.records.size() * 3 ); // quick and dirty
            for( std::size_t i( 0 ), k( 0 ); i < it->second.records.size(); ++i ) { v[ k++ ] = it->second.records[i]->value.x(); v[ k++ ] = it->second.records[i]->value.y(); v[ k++ ] = it->second.records[i]->value.z(); }
            err = cudaMemcpy( cur, &v[0], v.size() * sizeof( double ), cudaMemcpyHostToDevice );
            if( err != cudaSuccess ) { COMMA_THROW( comma::exception, "failed to copy; " << cudaGetErrorString( err ) ); }
            it->second.buffer.cuda_in = reinterpret_cast< double* >( cur );
            cur += v.size() * sizeof( double );
            it->second.buffer.cuda_out = reinterpret_cast< double* >( cur );
            cudaMemset( cur, 0, it->second.records.size() * sizeof( double ) );
            cur += it->second.records.size() * sizeof( double );
            it->second.buffer.out.resize( it->second.records.size() );
        }
        return buf;
    }
    #endif // #ifdef SNARK_USE_CUDA
    template < typename S > static void output( const S& istream, const record_t& nearest, const Eigen::Vector3d& nearest_point )
    {
        if( stdin_csv.binary() ) // quick and dirty
        {
            std::cout.write( istream.binary().last(), stdin_csv.format().size() );
            if( append_nearest ) { std::cout.write( &nearest.line[0], filter_csv.format().size() ); }
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter );
            if( !append_nearest ) { std::cout << std::endl; return; }
            std::cout << stdin_csv.delimiter;
            if( filter_csv.binary() ) { std::cout << filter_csv.format().bin_to_csv( &nearest.line[0], stdin_csv.delimiter, stdin_csv.precision ) << std::endl; }
            else { std::cout << nearest.line << std::endl; }
        }
    }
};

template <> struct traits< snark::triangle >
{
    typedef triangle_record record_t;
    typedef snark::triangle value_t;
    typedef triangle_input input_t;
    struct voxel_t
    {
        std::vector< const record_t* > records;
        #ifdef SNARK_USE_CUDA
        snark::cuda::buffer buffer;
        void calculate_squared_norms( const Eigen::Vector3d& ) {}
        #endif
        boost::optional< std::pair< Eigen::Vector3d, double > > nearest_to( const point_input& rhs, unsigned int k ) const
        {
            // todo: #ifdef SNARK_USE_CUDA
            const boost::optional< Eigen::Vector3d >& n = records[k]->nearest_to( rhs );
            if( !n ) { return boost::none; }
            return std::make_pair( *n, ( *n - rhs.value ).squaredNorm() );
        }
    };
    typedef snark::voxel_map< voxel_t, 3 > grid_t;
    struct nearest_t // quick and dirty
    {
        const record_t* record;
        Eigen::Vector3d point;
        double squared_distance;
        nearest_t() : record( NULL ), squared_distance( 0 ) {}
        nearest_t( const record_t* record, const Eigen::Vector3d& point, double squared_distance ): record( record ), point( point ), squared_distance( squared_distance ) {}
    };
    static snark::triangle default_value() { return snark::triangle(); }
    static void set_hull( snark::math::closed_interval< double, 3 >& extents, const snark::triangle& t )
    {
        extents.set_hull( t.corners[0] );
        extents.set_hull( t.corners[1] );
        extents.set_hull( t.corners[2] );
    }
    static bool touch( grid_t& grid, const record_t& record )
    {
        if(    ( record.value.corners[0] - record.value.corners[1] ).norm() > max_triangle_side
            || ( record.value.corners[1] - record.value.corners[2] ).norm() > max_triangle_side
            || ( record.value.corners[2] - record.value.corners[0] ).norm() > max_triangle_side )
        {
            if( verbose || strict ) { std::cerr << "points-join: expected triangles with longest side of " << max_triangle_side << "; got: " << std::endl << record.value.corners[0].transpose() << ";" << record.value.corners[1].transpose() << ";" << record.value.corners[2].transpose() << std::endl; }
            return false;
        }
        grid_t::iterator i0 = grid.touch_at( record.value.corners[0] );
        grid_t::iterator i1 = grid.touch_at( record.value.corners[1] );
        grid_t::iterator i2 = grid.touch_at( record.value.corners[2] );
        i0->second.records.push_back( &record ); // todo: cuda
        if( i1 != i0 ) { i1->second.records.push_back( &record ); } // todo: cuda
        if( i2 != i0 && i2 != i1 ) { i2->second.records.push_back( &record ); } // todo: cuda
        return true;
    }
    #ifdef SNARK_USE_CUDA
    static double* to_cuda( grid_t& grid, const std::deque< record_t >& records ) { /* todo */ return NULL; }
    #endif // #ifdef SNARK_USE_CUDA
    template < typename S > static void output( const S& istream, const record_t& nearest, const Eigen::Vector3d& nearest_point )
    {
        if( stdin_csv.binary() ) // quick and dirty
        {
            std::cout.write( istream.binary().last(), stdin_csv.format().size() );
            if( !append_nearest ) { return; }
            std::cout.write( &nearest.line[0], filter_csv.format().size() );
            static comma::csv::binary< Eigen::Vector3d > b;
            static std::vector< char > buf( b.format().size() );
            b.put( nearest_point, &buf[0] );
            std::cout.write( &buf[0], buf.size() );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter );
            if( !append_nearest ) { std::cout << std::endl; return; }
            std::cout << stdin_csv.delimiter;
            if( filter_csv.binary() ) { std::cout << filter_csv.format().bin_to_csv( &nearest.line[0], stdin_csv.delimiter, stdin_csv.precision ); }
            else { std::cout << nearest.line; }
            std::cout << stdin_csv.delimiter << nearest_point.x() << stdin_csv.delimiter << nearest_point.y() << stdin_csv.delimiter << nearest_point.z() << std::endl;
        }
    }
};

template < typename V > struct join_impl_
{
    typedef typename traits< V >::input_t filter_value_t;
    typedef typename traits< V >::record_t filter_record_t;
    typedef typename traits< V >::grid_t grid_t;
    typedef typename traits< Eigen::Vector3d >::input_t input_t;
    static std::deque< filter_record_t > filter_points;
    
    static grid_t read_filter_block( comma::csv::input_stream< filter_value_t >& istream )
    {
        filter_points.clear();
        snark::math::closed_interval< double, 3 > extents;
        if( verbose ) { std::cerr << "points-join: reading filter records..." << std::endl; }
        std::size_t count = 0;
        static const filter_value_t* p = nullptr;
        if( !block ) { p = istream.read(); }
        if( p ) { block = p->block; }
        while( p )
        {
            if( use_block && ( p->block != *block ) ) { break; } // todo: is the condition excessive? is not it just ( p->block != *block )?
            std::string line;
            if( filter_csv.binary() ) // quick and dirty
            {
                line.resize( filter_csv.format().size() );
                ::memcpy( &line[0], istream.binary().last(), filter_csv.format().size() );
            }
            else
            {
                line = comma::join( istream.ascii().last(), filter_csv.delimiter );
            }
            filter_record_t filter_record( filter_record_t( *p, line ) );
            if( filter_record.is_valid() )
            {
                filter_points.push_back( filter_record );
                traits< V >::set_hull( extents, p->value );
            }
            else
            {
                if( !permissive ) { COMMA_THROW( comma::exception, "points-join: filter point " << count << " invalid; use --permissive" ); }
                if( verbose ) { std::cerr << "points-join: filter point " << count << " invalid; discarded" << std::endl; }
            }
            ++count;
            p = istream.read();
        }
        if( verbose ) { std::cerr << "points-join: loading " << filter_points.size() << " records from block " << *block << " into grid..." << std::endl; }
        grid_t grid( extents.min(), resolution );
        for( std::size_t i = 0; i < filter_points.size(); ++i ) { if( !traits< V >::touch( grid, filter_points[i] ) && strict ) { COMMA_THROW(comma::exception, "filter point " << i << " is invalid; don't use --strict"); } }
        #ifdef SNARK_USE_CUDA
        if( use_cuda ) { cuda_buf = traits< V >::to_cuda( grid, filter_points ); }
        #endif
        return grid;
    }

    static grid_t read_filter_block( bool self_join )
    {
        if( self_join )
        {
            static comma::csv::input_stream< filter_value_t > istream( std::cin, stdin_csv );
            return read_filter_block( istream );
        }
        static std::ifstream ifs( &filter_csv.filename[0] );
        if( !ifs.is_open() ) { std::cerr << "points-join: failed to open \"" << filter_csv.filename << "\"" << std::endl; }
        static comma::csv::input_stream< filter_value_t > ifstream( ifs, filter_csv, filter_value_t() );
        return read_filter_block( ifstream );
    }
    
    static void handle_record( const typename traits< Eigen::Vector3d >::input_t& r ) {}
    
    static int run( const comma::command_line_options& options, bool self_join )
    {
        if( self_join ) { std::cerr << "points-join: self-join: todo" << std::endl; return 1; }
        // todo: self-join
        // todo: --ignore 0 distance (or --min-distance)
        // todo? --incremental or it belongs to points-calc?
        options.assert_mutually_exclusive( "--all", "--size,--number-of-points,--number-of-nearest-points" );
        unsigned int size = options.value( "--size,--number-of-points,--number-of-nearest-points", 1 );
        strict = options.exists( "--strict" );
        permissive = options.exists( "--permissive" );
        bool blocks_ordered = options.exists( "--blocks-ordered" );
        bool all = options.exists( "--all" );
        #ifdef SNARK_USE_CUDA
        use_cuda = options.exists( "--use-cuda,--cuda" );
        options.assert_mutually_exclusive( "--use-cuda,--cuda,--all" );
        #endif
        grid_t grid = read_filter_block( self_join );
        if( !block && !self_join ) { std::cerr << "points-join: got no records from filter" << std::endl; return 1; }
        if( self_join ) { return 0; }
        if( verbose ) { std::cerr << "points-join: joining..." << std::endl; }
        use_radius = stdin_csv.has_field( "radius" );
        if( self_join && use_radius ) { std::cerr << "points-join: self-join: radius field: not supported" << std::endl; return 1; }
        comma::csv::input_stream< input_t > istream( std::cin, stdin_csv ); // quick and dirty, don't mind self_join
        #ifdef WIN32
        if( stdin_csv.binary() ) { _setmode( _fileno( stdout ), _O_BINARY ); }
        #endif
        if( !stdin_csv.binary() ) { std::cout.precision( stdin_csv.precision ); }
        std::size_t count = 0;
        std::size_t discarded = 0;
        typedef typename traits< V >::nearest_t nearest_t;
        std::multimap< double, nearest_t > nearest_map; // boost::optional< nearest_t > nearest;
        auto read_ = [&]() -> const input_t*
        {
            if( self_join )
            {
                // todo
            }
            return istream.ready() || ( std::cin.good() && !std::cin.eof() ) ? istream.read() : nullptr; // quick and dirty
        };
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const input_t* p = read_(); //const input_t* p = istream.read();
            if( !p ) { break; }
            double current_squared_radius = get_squared_radius( *p );
            if( verbose && comma::math::less( squared_radius, current_squared_radius ) ) { std::cerr << "points-join: expected point-specific radius not exceeding --radius " << std::sqrt( squared_radius ) << "; got: " << p->radius << std::endl; }
            if( use_block )
            {
                if( blocks_ordered )
                {
                    while( *block < p->block ) { grid = read_filter_block( self_join ); }
                    if( *block > p->block )
                    {
                        if( matching ) { ++discarded; ++count; } else { output_last( istream ); }
                        continue;
                    }
                }
                else
                {
                    if( *block != p->block ) { grid = read_filter_block( self_join ); }
                    if( *block != p->block ) { COMMA_THROW( comma::exception, "expected blocks in input and filter to match, got input block " << p->block << " and filter block " << *block << "; make sure block ids are in ascending order and use --blocks-ordered" );}
                }
            }
            typename grid_t::index_type index = grid.index_of( p->value );
            typename grid_t::index_type i;
            if( all )
            {
                for( i[0] = index[0] - 1; i[0] < index[0] + 2; ++i[0] )
                {
                    for( i[1] = index[1] - 1; i[1] < index[1] + 2; ++i[1] )
                    {
                        for( i[2] = index[2] - 1; i[2] < index[2] + 2; ++i[2] )
                        {
                            typename grid_t::iterator it = grid.find( i );
                            if( it == grid.end() ) { continue; }
                            #ifdef SNARK_USE_CUDA
                            if( use_cuda ) { it->second.calculate_squared_norms( p->value ); }
                            #endif
                            for( std::size_t k = 0; k < it->second.records.size(); ++k )
                            {
                                const boost::optional< std::pair< Eigen::Vector3d, double > >& q = it->second.nearest_to( *p, k ); // todo: fix! currently, visiting each triangle 3 times
                                if( !q || q->second > current_squared_radius || q->second < squared_min_radius ) { continue; }
                                if( stdin_csv.binary() )
                                {
                                    std::cout.write( istream.binary().last(), stdin_csv.format().size() );
                                    std::cout.write( &it->second.records[k]->line[0], filter_csv.format().size() );
                                }
                                else
                                {
                                    std::cout << comma::join( istream.ascii().last(), stdin_csv.delimiter ) << stdin_csv.delimiter;
                                    if( filter_csv.binary() ) { std::cout << filter_csv.format().bin_to_csv( &it->second.records[k]->line[0], stdin_csv.delimiter, stdin_csv.precision ) << std::endl; }
                                    else { std::cout << &it->second.records[k]->line[0] << std::endl; }
                                }
                            }
                        }
                    }
                }
            }
            else
            {
                boost::optional< nearest_t > nearest;
                bool enough = false;
                if( size > 1 ) { nearest_map.clear(); }
                for( i[0] = index[0] - 1; !enough && i[0] < index[0] + 2; ++i[0] )
                {
                    for( i[1] = index[1] - 1; !enough && i[1] < index[1] + 2; ++i[1] )
                    {
                        for( i[2] = index[2] - 1; !enough && i[2] < index[2] + 2; ++i[2] )
                        {
                            typename grid_t::iterator it = grid.find( i );
                            if( it == grid.end() ) { continue; }
                            #ifdef SNARK_USE_CUDA
                            if( use_cuda ) { it->second.calculate_squared_norms( p->value ); }
                            #endif
                            if( size == 1 ) // have to handle size 1 separately due to poorer performance of std::map
                            {
                                for( std::size_t k = 0; k < it->second.records.size(); ++k )
                                {
                                    const boost::optional< std::pair< Eigen::Vector3d, double > >& q = it->second.nearest_to( *p, k ); // todo: fix! currently, visiting each triangle 3 times
                                    if( !q || q->second > current_squared_radius || q->second < squared_min_radius ) { continue; }
                                    if( !nearest || nearest->squared_distance > q->second ) { nearest.reset( nearest_t( it->second.records[k], q->first, q->second ) ); }
                                    if( !append_nearest ) { enough = true; break; }
                                }
                            }
                            else
                            {
                                for( std::size_t k = 0; k < it->second.records.size(); ++k )
                                {
                                    const boost::optional< std::pair< Eigen::Vector3d, double > >& q = it->second.nearest_to( *p, k ); // todo: fix! currently, visiting each triangle 3 times
                                    if( !q || q->second > current_squared_radius || q->second < squared_min_radius ) { continue; }
                                    if( nearest_map.size() >= size )
                                    {
                                        if( nearest_map.rbegin()->second.squared_distance < q->second || q->second < squared_min_radius ) { continue; }
                                        nearest_map.erase( std::prev( nearest_map.end() ) );
                                    }
                                    nearest_map.insert( std::make_pair( q->second, nearest_t( it->second.records[k], q->first, q->second ) ) );
                                }
                            }
                        }
                    }
                }
                if( matching )
                {
                    if( nearest_map.empty() && !nearest )
                    {
                        if( verbose ) { std::cerr.precision( 12 ); std::cerr << "points-join: record " << count << " at " << p->value.x() << "," << p->value.y() << "," << p->value.z() << ": no matches found" << std::endl; }
                        if( strict ) { return 1; }
                        ++discarded;
                    }
                    else
                    {
                        if( size == 1 ) { traits< V >::output( istream, *nearest->record, nearest->point ); }
                        else { for( const auto& n: nearest_map ) { traits< V >::output( istream, *n.second.record, n.second.point ); } }
                    }
                }
                else
                { 
                    if( nearest_map.empty() && !nearest ) { output_last( istream ); } else { ++discarded; }
                }
            }
            ++count;
        }
        std::cerr << "points-join: processed " << count << " record(s); discarded " << discarded << " record(s)" << ( count == 1 ? "" : "s" ) << " with " << ( matching ? "no " : "" ) << "matches" << std::endl;
        #ifdef SNARK_USE_CUDA
        cuda_deallocate();
        #endif
        return 0;
    }
};

template < typename V > std::deque< typename join_impl_< V >::filter_record_t > join_impl_< V >::filter_points;

bool find_in_fields( const std::vector< std::string >& fields, const std::vector< std::string >& strings )
{
    for( const auto& s : strings ) { for( const auto& f : fields ) { if( comma::split( f, "[/" )[0] == s ) { return true; } } }
    return false;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< point_input >( true ), ',' ) << std::endl; return 0; }
        verbose = options.exists( "--verbose,-v" );
        stdin_csv = comma::csv::options( options );
        if( stdin_csv.fields.empty() ) { stdin_csv.fields = "x,y,z"; }
        stdin_csv.full_xpath = true;
        radius = options.value< double >( "--radius" );
        squared_radius = radius * radius;
        min_radius = options.value( "--radius-min,--min-radius", 0. );
        squared_min_radius = min_radius * min_radius;
        options.assert_mutually_exclusive( "--matching,--not-matching" );
        matching = !options.exists( "--not-matching" );
        append_nearest = !options.exists( "--matching" ) && matching;
        matching_id = !options.exists( "--id-not-matching,--not-matching-id" );
        std::vector< std::string > unnamed = options.unnamed( "--all,--blocks-ordered,--id-not-matching,--not-matching-id,--matching,--not-matching,--strict,--permissive,--use-cuda,--cuda,--flush,--full-xpath,--verbose,-v", "-.*" );
        if( unnamed.size() > 1 ) { std::cerr << "points-join: expected one file or stream to join, got: " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        comma::name_value::parser parser( "filename", ';', '=', false );
        filter_csv = unnamed.empty() ? stdin_csv : parser.get< comma::csv::options >( unnamed[0] );
        if ( filter_csv.fields.empty() ) { filter_csv.fields="x,y,z"; }
        filter_csv.full_xpath = true;
        if( append_nearest && stdin_csv.binary() && !filter_csv.binary() ) { std::cerr << "points-join: stdin stream binary and filter stream ascii: this combination is not supported" << std::endl; return 1; }
        const std::vector< std::string >& v = comma::split( filter_csv.fields, ',' );
        const std::vector< std::string >& w = comma::split( stdin_csv.fields, ',' );
        std::vector< std::string > corner_strings = { "corners" };
        std::vector< std::string > normal_strings = { "normal" };
        std::vector< std::string > block_strings = { "block" };
        bool filter_triangulated = find_in_fields( v, corner_strings );
        use_normal = ( find_in_fields( w, normal_strings ) && find_in_fields( v, normal_strings ) );
        use_block = filter_csv.has_field( "block" ) && stdin_csv.has_field( "block" );
        #ifdef SNARK_USE_CUDA
        if (use_cuda && use_normal) { std::cerr << "todo: point normals not implemented for cuda" << std::endl; return 1; }
        #endif
        double r = radius;
        if( filter_triangulated ) // quick and dirty
        {
            if( unnamed.empty() ) { std::cerr << "points-join: self-join is not supported for triangles" << std::endl; return 1; }
            max_triangle_side = options.value< double >( "--max-triangle-side", r );
            if( max_triangle_side > r ) { r = max_triangle_side; }
            r *= 2; // todo: quick and dirty, calculate precise upper bound; needed to contain all triangles in given radius
            origin = options.exists( "--origin" ) ? comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--origin" ) ) : Eigen::Vector3d::Zero();
        }
        resolution = Eigen::Vector3d( r, r, r );
        if( unnamed.empty() )  { return join_impl_< Eigen::Vector3d >::run( options, unnamed.empty() ); }
        if( !filter_triangulated && unnamed.empty() ) { return join_impl_< Eigen::Vector3d >::run( options, unnamed.empty() ); }
        return filter_triangulated ? join_impl_< snark::triangle >::run( options, unnamed.empty() ) : join_impl_< Eigen::Vector3d >::run( options, unnamed.empty() );
    }
    catch( std::exception& ex ) { std::cerr << "points-join: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-join: unknown exception" << std::endl; }
    #ifdef SNARK_USE_CUDA
    cuda_deallocate();
    #endif
    return 1;
}
