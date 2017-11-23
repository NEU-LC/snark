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
    #ifdef SNARK_USE_CUDA
    std::cerr << "    --use-cuda,--cuda: experimental option; currently 40 times slower then normal operation, thus don't use it, yet" << std::endl;
    #endif
    std::cerr << "    --radius=<radius>: lookup radius" << std::endl;
    std::cerr << "    --strict: exit, if nearest point not found" << std::endl;
    std::cerr << "    --permissive: discard invalid points or triangles and continue" << std::endl;
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
static double radius;
static double squared_radius;
static double max_triangle_side;
static Eigen::Vector3d origin = Eigen::Vector3d::Zero();
static Eigen::Vector3d resolution;
static comma::csv::options stdin_csv;
static comma::csv::options filter_csv;
static boost::optional<comma::uint32> block;
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
    comma::uint32 block;
    point_input() : value( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ), block( 0 ) {}
};

struct triangle_input
{
    snark::triangle value;
    comma::uint32 block;
    triangle_input() : block( 0 ) {}
};

namespace comma { namespace visiting {
template <> struct traits< point_input >
{
    template< typename K, typename V > static void visit( const K& k, point_input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.value, v );
        v.apply( "normal", t.normal );
        v.apply( "block", t.block );
    }

    template< typename K, typename V > static void visit( const K& k, const point_input& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t.value, v );
        v.apply( "normal", t.normal );
        v.apply( "block", t.block );
    }
};

template <> struct traits< triangle_input >
{
    template< typename K, typename V > static void visit( const K& k, triangle_input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.value, v );
        v.apply( "block", t.block );
    }

    template< typename K, typename V > static void visit( const K& k, const triangle_input& t, V& v )
    {
        traits< snark::triangle >::visit( k, t.value, v );
        v.apply( "block", t.block );
    }
};

} }

// todo: add block field
struct record
{
    Eigen::Vector3d value;
    Eigen::Vector3d normal;
    std::string line;
    record() : value( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ) {}
    record( const point_input& input, const std::string& line ) : value( input.value ), normal( input.normal ), line( line ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const point_input& rhs ) const
    {
        if( use_normal ) { return !comma::math::less( normal.dot(rhs.normal), 0 ) ? boost::make_optional(value) : boost::none; }
        return value;
    } // watch performance
    bool is_valid() const { return true; }
};

// todo: add block field
struct triangle_record
{
    snark::triangle value;
    std::string line;
    triangle_record() {}
    triangle_record( const triangle_input& input, const std::string& line ) : value( input.value ), line( line ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const point_input& rhs ) const // quick and dirty, watch performance
    {
        boost::optional< Eigen::Vector3d > p = value.projection_of( rhs.value );
        return value.includes( *p ) && !comma::math::less( value.normal().dot( origin - rhs.value ), 0 ) ? p : boost::none;
    }
    bool is_valid() const { return value.is_valid(); }
};

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
                return std::make_pair(*n, (*n - rhs.value).squaredNorm());
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

template < typename V > struct join_impl_
{
    typedef typename traits< V >::input_t filter_value_t;
    typedef typename traits< V >::record_t filter_record_t;
    typedef typename traits< V >::grid_t grid_t;

    static grid_t read_filter_block() {
        static std::ifstream ifs( &filter_csv.filename[0] );
        if( !ifs.is_open() ) { std::cerr << "points-join: failed to open \"" << filter_csv.filename << "\"" << std::endl; }
        static comma::csv::input_stream< filter_value_t > ifstream( ifs, filter_csv, filter_value_t() );
        static std::deque< filter_record_t > filter_points;
        filter_points.clear();
        snark::math::closed_interval< double, 3 > extents;
        if( verbose ) { std::cerr << "points-join: reading filter records..." << std::endl; }
        std::size_t count = 0;
        static const filter_value_t* p;
        if (!block) { p = ifstream.read(); }
        if (p) { block = p->block; }
        while( p )
        {
            if (use_block && ( p->block != *block ) ) { break; }
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
            filter_record_t filter_record( filter_record_t( *p, line ) );
            if( filter_record.is_valid() )
            {
                filter_points.push_back( filter_record );
                traits< V >::set_hull( extents, p->value );
            }
            else
            {
                if( !permissive ) { COMMA_THROW(comma::exception, "points-join: filter point " << count << " invalid; use --permissive"); }
                if( verbose ) { std::cerr << "points-join: filter point " << count << " invalid; discarded" << std::endl; }
            }
            ++count;
            p = ifstream.read();
        }
        if( verbose ) { std::cerr << "points-join: loading " << filter_points.size() << " records from block " << *block << " into grid..." << std::endl; }
        grid_t grid( extents.min(), resolution );
        for( std::size_t i = 0; i < filter_points.size(); ++i ) { if( !traits< V >::touch( grid, filter_points[i] ) && strict ) { COMMA_THROW(comma::exception, "filter point " << i << " is invalid; don't use --strict"); } }
        #ifdef SNARK_USE_CUDA
        if( use_cuda ) { cuda_buf = traits< V >::to_cuda( grid, filter_points ); }
        #endif

        return grid;
    }

    static int run( const comma::command_line_options& options )
    {
        strict = options.exists( "--strict" );
        permissive = options.exists( "--permissive" );
        bool all = options.exists( "--all" );
        #ifdef SNARK_USE_CUDA
        use_cuda = options.exists( "--use-cuda,--cuda" );
        options.assert_mutually_exclusive( "--use-cuda,--cuda,--all" );
        #endif

        grid_t grid = read_filter_block();
        if (!block) { std::cerr << "got 0 records from filter" << std::endl; return 1; }

        typedef traits< Eigen::Vector3d >::input_t input_t;

        if( verbose ) { std::cerr << "points-join: joining..." << std::endl; }
        comma::csv::input_stream< input_t > istream( std::cin, stdin_csv, input_t() );
        #ifdef WIN32
        if( stdin_csv.binary() ) { _setmode( _fileno( stdout ), _O_BINARY ); }
        #endif
        if( !stdin_csv.binary() ) { std::cout.precision( stdin_csv.precision ); }
        std::size_t count = 0;
        std::size_t discarded = 0;
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            const input_t* p = istream.read();
            if( !p ) { break; }
            // todo: if blocks don't match, which input to wait for?
            if ( use_block && *block != p->block ) { grid = read_filter_block(); }
            if ( use_block && *block != p->block ) { COMMA_THROW(comma::exception, "expected blocks in input and filter to match, got input block " << p->block << " and filter block " << *block);}
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
                                if( !q || q->second > squared_radius ) { continue; }
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
                typename traits< V >::nearest_t nearest;
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
                                if( !q || q->second > squared_radius ) { continue; }
                                if( nearest.record && nearest.squared_distance < q->second ) { continue; }
                                nearest.record = it->second.records[k];
                                nearest.point = q->first;
                                nearest.squared_distance = q->second;
                            }
                        }
                    }
                }
                if( !nearest.record )
                {
                    if( verbose ) { std::cerr.precision( 12 ); std::cerr << "points-join: record " << count << " at " << p->value.x() << "," << p->value.y() << "," << p->value.z() << ": no matches found" << std::endl; }
                    if( strict ) { return 1; }
                    ++discarded;
                    continue;
                }
                traits< V >::output( istream, *nearest.record, nearest.point );
            }
            ++count;
        }
        std::cerr << "points-join: processed " << count << " records; discarded " << discarded << " record" << ( count == 1 ? "" : "s" ) << " with no matches" << std::endl;
        #ifdef SNARK_USE_CUDA
        cuda_deallocate();
        #endif
        return 0;
    }
};

bool find_in_fields( const std::vector<std::string>& fields, const std::vector<std::string>& strings )
{
    for (const auto& s : strings )
    {
        for( const auto& f : fields )
        {
            if( comma::split( f, "[/" )[0] == s ) { return true; }
        }
    }
    return false;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        stdin_csv = comma::csv::options( options );
        if( stdin_csv.fields.empty() ) { stdin_csv.fields = "x,y,z"; }
        stdin_csv.full_xpath = true;
        std::vector< std::string > unnamed = options.unnamed( "--use-cuda,--cuda,--verbose,-v,--strict,--all", "-.*" );
        if( unnamed.empty() ) { std::cerr << "points-join: please specify the second source; self-join: todo" << std::endl; return 1; }
        if( unnamed.size() > 1 ) { std::cerr << "points-join: expected one file or stream to join, got " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        comma::name_value::parser parser( "filename", ';', '=', false );
        filter_csv = parser.get< comma::csv::options >( unnamed[0] );
        if ( filter_csv.fields.empty() ) { filter_csv.fields="x,y,z"; }
        filter_csv.full_xpath = true;
        if( stdin_csv.binary() && !filter_csv.binary() ) { std::cerr << "points-join: stdin stream binary and filter stream ascii: this combination is not supported" << std::endl; return 1; }
        const std::vector< std::string >& v = comma::split( filter_csv.fields, ',' );
        const std::vector< std::string >& w = comma::split( stdin_csv.fields, ',' );
        const std::vector<std::string> corner_strings = { "corners" };
        const std::vector<std::string> normal_strings = { "normal" };
        const std::vector<std::string> block_strings = { "block" };
        bool filter_triangulated = find_in_fields(v, corner_strings);
        use_normal = ( find_in_fields(w, normal_strings) && find_in_fields(v, normal_strings) );
        use_block = ( find_in_fields(w, block_strings) && find_in_fields(v, block_strings) );

        #ifdef SNARK_USE_CUDA
        if (use_cuda && use_normal) { std::cerr << "todo: point normals not implemented for cuda" << std::endl; return 1; }
        #endif

        radius = options.value< double >( "--radius" );
        squared_radius = radius * radius;
        double r = radius;
        if( filter_triangulated ) // quick and dirty
        {
            max_triangle_side = options.value< double >( "--max-triangle-side", r );
            if( max_triangle_side > r ) { r = max_triangle_side; }
            r *= 2; // todo: quick and dirty, calculate precise upper bound; needed to contain all triangles in given radius
            origin = options.exists( "--origin" ) ? comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--origin" ) ) : Eigen::Vector3d::Zero();
        }
        resolution = Eigen::Vector3d( r, r, r );
        return filter_triangulated ? join_impl_< snark::triangle >::run( options )
                                   : join_impl_< Eigen::Vector3d >::run( options );
    }
    catch( std::exception& ex ) { std::cerr << "points-join: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-join: unknown exception" << std::endl; }
    #ifdef SNARK_USE_CUDA
    cuda_deallocate();
    #endif
    return 1;
}
