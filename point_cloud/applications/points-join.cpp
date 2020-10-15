// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2019 Vsevolod Vlaskine
// Copyright (c) 2020 Abyss Solutions

/// @authors vsevolod vlaskine, toby dunne, kent hu

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <boost/bind.hpp>
#include <boost/optional.hpp>
#include <tbb/pipeline.h>

#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/base/last_error.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/application/signal_flag.h>
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
    std::cerr << "usage: cat points.1.csv | points-join \"points.2.csv[;<csv options>]\" [<options>] > joined.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    if the second set is not given, for each point output the nearest point in the same set; todo" << std::endl;
    std::cerr << "                " << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --all: output all points in the given radius instead of the nearest" << std::endl;
    std::cerr << "    --blocks-ordered: stdin and filter blocks are ordered and therefore missing filter blocks can be handled correctly" << std::endl;
    std::cerr << "    --count: if --all, append to input just count of points in a given radius as 4-byte integer, not the points themselves" << std::endl;
    std::cerr << "    --count-fast: same as --count, but instead of sphere, simply count all points in neighbouring voxels" << std::endl;
    std::cerr << "    --id-not-matching,--not-matching-id: if id field present in --fields, match only points with different ids" << std::endl;
    std::cerr << "                                         default: if id field present, match points with the same id" << std::endl;
    std::cerr << "    --input-fields: output input fields and exit" << std::endl;
    std::cerr << "    --matching: output only points that have a match, do not append nearest point; a convenience option" << std::endl;
    std::cerr << "    --not-matching: output only points that do not have a match, i.e. opposite of --matching" << std::endl;
    std::cerr << "    --parallel-threads,--threads=<count>; default=" << std::thread::hardware_concurrency() << "; number of threads" << std::endl;
    std::cerr << "                                        LIMITATION: if the input data stream is intermittent, i.e. there are intervals of idleness" << std::endl;
    std::cerr << "                                                    between batches of points, points-join may start taking 100% of CPU" << std::endl;
    std::cerr << "                                                    this is due to how multithreading is implemented (most likely a design" << std::endl;
    std::cerr << "                                                    bug/drawback in TBB)" << std::endl;
    std::cerr << "                                                    we are trying to fix it, but meanwhile, if it becomes a problem" << std::endl;
    std::cerr << "                                                    use --threads=1: you will lose parallelisation, but avoid constant 100% of CPU load" << std::endl;
    std::cerr << "    --parallel-chunk-size,--chunk-size=<size>; default=256: read input in chunks of <size> record; if --flush or ascii input, automatically set to --chunk-size=1" << std::endl;
    std::cerr << "    --permissive: discard invalid points or triangles and continue" << std::endl;
    std::cerr << "    --radius=<radius>: max lookup radius, required even if radius field is present" << std::endl;
    std::cerr << "    --radius-min,--min-radius=<radius>; default=0: min lookup radius, e.g. to filter out points on poor-man's self-join" << std::endl;
    std::cerr << "    --size,--number-of-points,--number-of-nearest-points=<number_of_points>; default=1: output up to a given number of nearest points in the given radius" << std::endl;
    std::cerr << "    --strict: exit, if nearest point not found; may not exit immediately sometimes" << std::endl;
    if( more )
    {
        std::cerr << "              if --parallel-threads is greater than 1, points-join may not always exit immediately on the current point but on the next input point" << std::endl;
        std::cerr << "              the output will still be the same (the next input point will not be processed)" << std::endl;
        std::cerr << "              if you require points-join to exit immediately consistently, then --parallel-threads should be set to 1" << std::endl;
    }
    else
    {
        std::cerr << "              for more help, use --verbose..." << std::endl;
    }
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
static Eigen::Vector3d origin;
static Eigen::Vector3d resolution;
static comma::csv::options stdin_csv;
static comma::csv::options filter_csv;
static boost::optional< comma::uint64 > block;
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
    comma::uint64 block;
    comma::uint32 id;
    point_input() : value( Eigen::Vector3d::Zero() ), normal( Eigen::Vector3d::Zero() ), radius( 0 ), block( 0 ), id( 0 ) {}
};

static double get_squared_radius( const point_input& p ) { return use_radius ? p.radius * p.radius : squared_radius; }

struct triangle_input
{
    snark::triangle value;
    comma::uint64 block;
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
    record( const point_input& input, std::string line ) : value( input.value ), normal( input.normal ), id( input.id ), line( std::move( line ) ) {}
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
    triangle_record( const triangle_input& input, std::string line ) : value( input.value ), id( input.id ), line( std::move( line ) ) {}
    boost::optional< Eigen::Vector3d > nearest_to( const point_input& rhs ) const // quick and dirty, watch performance
    {
        if( ( rhs.id == id ) != matching_id ) { return boost::none; }
        boost::optional< Eigen::Vector3d > p = value.projection_of( rhs.value );
        return value.includes( *p ) && !comma::math::less( value.normal().dot( origin - rhs.value ), 0 ) ? p : boost::none;
    }
    bool is_valid() const { return value.is_valid(); }
};

static void write_( int fd, const char* buf, std::size_t count )
{
    while( count )
    {
        auto bytes_written = ::write( fd, buf, count );
        if( bytes_written == -1 ) { COMMA_THROW( comma::last_error::exception, "error" ); }
        if( bytes_written == 0 ) { COMMA_THROW( comma::exception, "write() wrote 0 bytes" ); } // shouldn't occur with stdout
        count -= bytes_written;
        buf += bytes_written;
    }
}

static void write_( int fd, const std::string& buf ) { ::write_( fd, &buf[0], buf.size() ); }

static void write_( int fd, char c ) { ::write_( fd, &c, sizeof( c ) ); }

static std::string bin_to_csv_( const std::string& filter_line ) { return filter_csv.format().bin_to_csv( &filter_line[0], stdin_csv.delimiter, stdin_csv.precision ); }

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
        nearest_t() : record( nullptr ), squared_distance( 0 ) {}
        nearest_t( const record_t* record, Eigen::Vector3d point, double squared_distance ): record( record ), point( std::move( point ) ), squared_distance( squared_distance ) {}
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
    static std::string to_string( const record_t& nearest, const Eigen::Vector3d& nearest_point )
    {
        if( !append_nearest ) { return {}; }
        if( stdin_csv.binary() ) { return nearest.line; }
        if( filter_csv.binary() ) { return ::bin_to_csv_( nearest.line ); }
        return nearest.line;
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
        nearest_t() : record( nullptr ), squared_distance( 0 ) {}
        nearest_t( const record_t* record, Eigen::Vector3d point, double squared_distance ): record( record ), point( std::move( point ) ), squared_distance( squared_distance ) {}
    };
    static snark::triangle default_value() { return {}; }
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
    static std::string to_string( const record_t& nearest, const Eigen::Vector3d& nearest_point )
    {
        if( !append_nearest ) { return {}; }
        if( stdin_csv.binary() ) // quick and dirty
        {
            static comma::csv::binary< Eigen::Vector3d > b;
            std::string line( &nearest.line[0], filter_csv.format().size() );
            line.resize( filter_csv.format().size() + b.format().size(), {} );
            b.put( nearest_point, &line[filter_csv.format().size()] );
            return line;
        }
        std::stringstream buffer;
        buffer << ( filter_csv.binary() ? ::bin_to_csv_( nearest.line ) : nearest.line );
        buffer << stdin_csv.delimiter << nearest_point.x() << stdin_csv.delimiter << nearest_point.y() << stdin_csv.delimiter << nearest_point.z();
        return buffer.str();
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
        else
        {
            // reached end of istream
            block = boost::none;
            if( verbose ) { std::cerr << "points-join: no filter records read" << std::endl; }
            return { extents.min(), resolution };
        }
        while( p )
        {
            if( use_block && ( p->block != *block ) ) { break; } // todo: is the condition excessive? is not it just ( p->block != *block )?
            filter_record_t filter_record( *p, istream.last() ); // istream.last()
            if( filter_record.is_valid() )
            {
                filter_points.emplace_back( std::move( filter_record ) );
                traits< V >::set_hull( extents, p->value );
            }
            else
            {
                if( !permissive ) { COMMA_THROW( comma::exception, "points-join: filter point " << count << " invalid; use --permissive" ); }
                if( verbose ) { std::cerr << "points-join: filter point " << count << " invalid; discarded" << std::endl; }
            }
            p = istream.read();
            ++count;
        }
        if( verbose ) { std::cerr << "points-join: loading " << filter_points.size() << " filter records of block " << *block << std::endl; }
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
        // todo: self-join
        // todo: --ignore 0 distance (or --min-distance)
        // todo? --incremental or it belongs to points-calc?
        if( verbose ) { std::cerr << "points-join: joining..." << std::endl; }
        if( self_join ) { std::cerr << "points-join: self-join: todo" << std::endl; return 1; }
        std::size_t size = options.value( "--size,--number-of-points,--number-of-nearest-points", 1 );
        bool blocks_ordered = options.exists( "--blocks-ordered" );
        bool all = options.exists( "--all" );
        bool output_count = options.exists( "--count,--count-fast" );
        bool fast = options.exists( "--count-fast" );
        if( output_count && !all ) { std::cerr << "points-join: --count requires --all; please specify --all" << std::endl; return 1; }
        #ifdef SNARK_USE_CUDA
        use_cuda = options.exists( "--use-cuda,--cuda" );
        options.assert_mutually_exclusive( "--use-cuda,--cuda,--all" );
        #endif
        grid_t grid = read_filter_block( self_join );
        bool empty_filter_and_matching = !block && !self_join && matching;
        if( empty_filter_and_matching && !strict ) { return 0; }
        if( self_join && use_radius ) { std::cerr << "points-join: self-join: radius field: not supported" << std::endl; return 1; }
        comma::csv::input_stream< input_t > istream( std::cin, stdin_csv ); // quick and dirty, don't mind self_join
        #ifdef WIN32
        if( stdin_csv.binary() ) { _setmode( _fileno( stdout ), _O_BINARY ); }
        #endif
        struct output_row_t
        {
            const std::string left_buffer;
            const std::string join_buffer;
            output_row_t( std::string left_buffer, std::string join_buffer ) : left_buffer( std::move( left_buffer ) ), join_buffer( std::move( join_buffer ) ) {}
        };
        struct output_container
        {
            std::vector< output_row_t > outputs;
            std::size_t count = 0;
            std::size_t discarded = 0;
            bool strict_and_nearest_point_not_found = false;
        };
        auto parallel_threads = options.value( "--parallel-threads,--threads", std::thread::hardware_concurrency() );
        if( parallel_threads <= 0 ) { parallel_threads = std::thread::hardware_concurrency(); }
        if( strict && stdin_csv.flush && parallel_threads != 1 )
        {
            std::cerr << "points-join: WARNING: --strict and --flush have been set with " << parallel_threads << " threads" << std::endl;
            std::cerr << "                      as an implementation limitation, you may need to specify --threads=1, depending on your use case, for consistent behaviour:" << std::endl;
            std::cerr << "                      - suppose you set --strict and --flush and expect that once there is a not matched input record, points-join exits immediately" << std::endl;
            std::cerr << "                      - however, if --threads is not set to 1, points-join would exit once it reads the NEXT input record, which may not unexpected" << std::endl;
            std::cerr << "                        when points-join is used for realtime or interactive data streams" << std::endl;
        }
        const auto& parallel_chunk_size = stdin_csv.flush || !stdin_csv.binary() ? 1 : static_cast< std::size_t >( options.value( "--parallel-chunk-size,--chunk-size", 256 ) );
        typedef std::vector< std::pair< input_t, std::string > > input_container;
        std::size_t count = 0;
        std::size_t discarded = 0;
        bool read_next_filter_block = false;
        const input_t* p = nullptr;
        auto read_ = [&]() { return istream.ready() || ( std::cin.good() && !std::cin.eof() ) ? istream.read() : nullptr; };
        auto read_points = [&]( tbb::flow_control& fc ) -> input_container
        {
            if( read_next_filter_block ) { fc.stop(); return {}; }
            input_container inputs;
            inputs.reserve( parallel_chunk_size );
            for( std::size_t i = 0; i < parallel_chunk_size; ++i )
            {
                static comma::signal_flag is_shutdown( comma::signal_flag::hard );
                if( is_shutdown || std::cin.eof() || std::cin.bad() || !std::cin.good() ) { fc.stop(); return {}; }
                if( !p ) { p = read_(); }
                if( !p ) { break; }
                read_next_filter_block = use_block && block && *block != p->block; // non-matching block, read next block in
                if( read_next_filter_block ) { break; } // process current input records
                inputs.emplace_back( *p, istream.last() );
                p = nullptr;
            }
            return inputs;
        };

        auto join_points = [&]( const input_container& inputs ) -> output_container
        {
            typedef typename traits< V >::nearest_t nearest_t;
            output_container container;
            auto& outputs = container.outputs;
            const auto& input_count = inputs.size();
            outputs.reserve( input_count ); // generally there will be 1 output to 1 input
            boost::optional< std::multimap< double, nearest_t > > nearest_map = boost::none;
            if( size > 1 ) { nearest_map.emplace(); } // nearest_map used to store <size> nearest points if more than one point is to be joined
            for( std::size_t input_i = 0; input_i < input_count; ++input_i )
            {
                const input_t& p = inputs[input_i].first;
                const std::string& left_line = inputs[input_i].second;
                double current_squared_radius = get_squared_radius( p );
                if( verbose && comma::math::less( squared_radius, current_squared_radius ) ) { std::cerr << "points-join: expected point-specific radius not exceeding --radius " << radius << "; got: " << p.radius << std::endl; }
                typename grid_t::index_type index = grid.index_of( p.value );
                typename grid_t::index_type i;
                if( all )
                {
                    if( output_count )
                    {
                        comma::uint32 c = 0;
                        for( i[0] = index[0] - 1; i[0] <= index[0] + 1; ++i[0] )
                        {
                            for( i[1] = index[1] - 1; i[1] <= index[1] + 1; ++i[1] )
                            {
                                for( i[2] = index[2] - 1; i[2] <= index[2] + 1; ++i[2] )
                                {
                                    typename grid_t::iterator it = grid.find( i );
                                    if( it == grid.end() ) { continue; }
                                    const auto& voxel_map = it->second;
                                    if( fast ) { c += voxel_map.records.size(); continue; }
                                    for( std::size_t k = 0; k < voxel_map.records.size(); ++k )
                                    {
                                        const boost::optional< std::pair< Eigen::Vector3d, double > >& q = voxel_map.nearest_to( p, k ); // todo: fix! currently, visiting each triangle 3 times
                                        if( q && q->second <= current_squared_radius && q->second >= squared_min_radius ) { ++c; }
                                    }
                                }
                            }
                        }
                        const std::string join_line = stdin_csv.binary() ? std::string( reinterpret_cast< const char* >( &c ), 4 ): boost::lexical_cast< std::string >( c );
                        outputs.emplace_back( left_line, join_line );
                    }
                    else
                    {
                        for( i[0] = index[0] - 1; i[0] <= index[0] + 1; ++i[0] )
                        {
                            for( i[1] = index[1] - 1; i[1] <= index[1] + 1; ++i[1] )
                            {
                                for( i[2] = index[2] - 1; i[2] <= index[2] + 1; ++i[2] )
                                {
                                    typename grid_t::iterator it = grid.find( i );
                                    if( it == grid.end() ) { continue; }
                                    #ifdef SNARK_USE_CUDA
                                    if( use_cuda ) { it->second.calculate_squared_norms( p.value ); }
                                    #endif
                                    const auto& voxel_map = it->second;
                                    for( std::size_t k = 0; k < voxel_map.records.size(); ++k )
                                    {
                                        const boost::optional< std::pair< Eigen::Vector3d, double > >& q = voxel_map.nearest_to( p, k ); // todo: fix! currently, visiting each triangle 3 times
                                        if( !q || q->second > current_squared_radius || q->second < squared_min_radius ) { continue; }
                                        const std::string& join_line = voxel_map.records[k]->line;
                                        if( stdin_csv.binary() ) { outputs.emplace_back( left_line, join_line ); continue; }
                                        if( filter_csv.binary() ) { outputs.emplace_back( left_line, bin_to_csv_( join_line ) ); continue; }
                                        outputs.emplace_back( left_line, join_line );
                                    }
                                }
                            }
                        }
                    }
                }
                else
                {
                    if( size > 1 ) { nearest_map->clear(); }
                    bool enough = false;
                    boost::optional< nearest_t > nearest = boost::none;
                    for( i[0] = index[0] - 1; !enough && i[0] <= index[0] + 1; ++i[0] )
                    {
                        for( i[1] = index[1] - 1; !enough && i[1] <= index[1] + 1; ++i[1] )
                        {
                            for( i[2] = index[2] - 1; !enough && i[2] <= index[2] + 1; ++i[2] )
                            {
                                typename grid_t::iterator it = grid.find( i );
                                if( it == grid.end() ) { continue; }
                                #ifdef SNARK_USE_CUDA
                                if( use_cuda ) { it->second.calculate_squared_norms( p.value ); }
                                #endif
                                const auto& voxel_map = it->second;
                                if( size == 1 ) // have to handle size 1 separately due to poorer performance of std::map
                                {
                                    for( std::size_t k = 0; k < voxel_map.records.size(); ++k )
                                    {
                                        const boost::optional< std::pair< Eigen::Vector3d, double > >& q = voxel_map.nearest_to( p, k ); // todo: fix! currently, visiting each triangle 3 times
                                        if( !q || q->second > current_squared_radius || q->second < squared_min_radius ) { continue; }
                                        if( !nearest || nearest->squared_distance > q->second ) { nearest.emplace( voxel_map.records[k], q->first, q->second ); }
                                        if( !append_nearest ) { enough = true; break; }
                                    }
                                }
                                else
                                {
                                    for( std::size_t k = 0; k < voxel_map.records.size(); ++k )
                                    {
                                        const boost::optional< std::pair< Eigen::Vector3d, double > >& q = voxel_map.nearest_to( p, k ); // todo: fix! currently, visiting each triangle 3 times
                                        if( !q || q->second > current_squared_radius || q->second < squared_min_radius ) { continue; }
                                        if( nearest_map->size() >= size )
                                        {
                                            auto end_it = std::prev( nearest_map->end() );
                                            if( end_it->second.squared_distance < q->second ) { continue; }
                                            nearest_map->erase( end_it );
                                        }
                                        nearest_map->emplace( std::make_pair( q->second, nearest_t( voxel_map.records[k], q->first, q->second ) ) );
                                    }
                                }
                            }
                        }
                    }
                    const bool nearest_point_not_found = ( size == 1 && !nearest ) || ( size > 1 && nearest_map->empty() );
                    if( matching )
                    {
                        if( nearest_point_not_found )
                        {
                            if( verbose ) { std::cerr << "points-join: record at " << p.value.x() << ',' << p.value.y() << ',' << p.value.z() << ": no matches found" << std::endl; }
                            if( strict ) { container.strict_and_nearest_point_not_found = true; return container; }
                            ++container.discarded;
                        }
                        else
                        {
                            if( size == 1 )
                            {
                                outputs.emplace_back( left_line, traits< V >::to_string( *nearest->record, nearest->point ) );
                            }
                            else
                            {
                                for( const auto& n: *nearest_map ) { outputs.emplace_back( left_line, traits< V >::to_string( *n.second.record, n.second.point ) ); }
                            }
                        }
                    }
                    else
                    {
                        if( nearest_point_not_found ) { outputs.emplace_back( left_line, "" ); } else { ++container.discarded; }
                    }
                }
                ++container.count;
            }
            return container;
        };
        bool strict_and_nearest_point_not_found = false;
        auto write_points = [&]( const output_container& container ) -> void
        {
            count += container.count;
            discarded += container.discarded;
            std::for_each( std::begin( container.outputs ), std::end( container.outputs ), [&]( const output_row_t& output_row ) -> void
            {
                ::write_( 1, output_row.left_buffer );
                if( append_nearest )
                {
                    if( !stdin_csv.binary() ) { ::write_( 1, stdin_csv.delimiter ); }
                    ::write_( 1, output_row.join_buffer );
                }
                if( !stdin_csv.binary() ) { ::write_( 1, '\n' ); }
            });
            if( stdin_csv.flush || !stdin_csv.binary() ) { ::fflush( stdout ); }
            if( container.strict_and_nearest_point_not_found ) { strict_and_nearest_point_not_found = true; tbb::task::self().cancel_group_execution(); }
        };
        tbb::filter_t< void, input_container > read_points_filter( tbb::filter::serial_in_order, read_points );
        tbb::filter_t< input_container, output_container > join_points_filter( tbb::filter::parallel, join_points );
        tbb::filter_t< output_container, void > write_points_filter( tbb::filter::serial_in_order, write_points );
        while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
        {
            tbb::parallel_pipeline( parallel_threads, read_points_filter & join_points_filter & write_points_filter );
            if( strict_and_nearest_point_not_found )
            {
                if( verbose && empty_filter_and_matching ) { std::cerr << "points-join: could not match input records as filter is empty" << std::endl; }
                return 1;
            }
            if( !read_next_filter_block ) { continue; }
            read_next_filter_block = false;
            auto pass_through = [&]()
            {
                while( p && ( !block || p->block < *block ) )
                {
                    ++count;
                    ::write_( 1, istream.last());
                    if( !stdin_csv.binary() ) { ::write_( 1, '\n' ); ::fflush( stdout ); }
                    else if( stdin_csv.flush ) { ::fflush( stdout ); }
                    p = read_();
                }
            };
            if( blocks_ordered )
            {
                while( block && p->block > *block ) { grid = read_filter_block( self_join ); }
                if( !block )
                {
                    std::cerr << "points-join: reached end of filter stream" << std::endl;
                    if( matching )
                    {
                        if( strict ) { std::cerr << "points-join: record at " << p->value.x() << ',' << p->value.y() << ',' << p->value.z() << ": no matches found" << std::endl; return 1; }
                        break; // stop processing input records since no filter block to match with
                    }
                    pass_through();
                    break;
                }
                if( p->block == *block ) { continue; } // found matching block grid, start joining
                if( matching )
                {
                    if( strict ) { std::cerr << "points-join: record at " << p->value.x() << ',' << p->value.y() << ',' << p->value.z() << ": no matches found" << std::endl; return 1; }
                    while( p && p->block < *block ) { ++count; ++discarded; p = read_(); }
                    continue;
                }
                pass_through();
            }
            else
            {
                if( p->block != *block )
                {
                    if( count == 0 ) { COMMA_THROW( comma::exception, "expected blocks in input and filter to match, got input block " << p->block << " and filter block " << *block << "; make sure block ids are in ascending order and use --blocks-ordered" ); }
                    grid = read_filter_block( self_join ); // read next filter block
                    if( block )
                    {
                        if( p->block != *block ) { COMMA_THROW( comma::exception, "expected blocks in input and filter to match, got input block " << p->block << " and filter block " << *block << "; make sure block ids are in ascending order and use --blocks-ordered" ); }
                        continue;
                    }
                    std::cerr << "points-join: reached end of filter stream" << std::endl;
                    if( matching )
                    {
                        if( strict ) { std::cerr << "points-join: record at " << p->value.x() << ',' << p->value.y() << ',' << p->value.z() << ": no matches found" << std::endl; return 1; }
                        break; // stop processing input records since no filter block to match with
                    }
                    pass_through();
                }
            }
        }
        std::cerr << "points-join: processed " << count << " record(s); discarded " << discarded << " record(s) with " << ( matching ? "no " : "" ) << "matches" << std::endl;
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
        options.assert_mutually_exclusive( "--all", "--size,--number-of-points,--number-of-nearest-points" );
        options.assert_mutually_exclusive( "--matching,--strict", "--not-matching");
        options.assert_mutually_exclusive( "--size,--number-of-points,--number-of-nearest-points,--all", "--matching,--not-matching" );
        options.assert_mutually_exclusive( "--flush", "--parallel-chunk-size,--chunk-size" );
        strict = options.exists( "--strict" );
        verbose = options.exists( "--verbose,-v" );
        permissive = options.exists( "--permissive" );
        stdin_csv = comma::csv::options( options, "x,y,z" );
        if( !stdin_csv.binary() ) { std::cout.precision( stdin_csv.precision ); std::cerr.precision( stdin_csv.precision ); }
        radius = options.value< double >( "--radius" );
        squared_radius = radius * radius;
        min_radius = options.value( "--radius-min,--min-radius", 0. );
        squared_min_radius = min_radius * min_radius;
        matching = !options.exists( "--not-matching" );
        append_nearest = !options.exists( "--matching" ) && matching;
        matching_id = !options.exists( "--id-not-matching,--not-matching-id" );
        std::vector< std::string > unnamed = options.unnamed( "--all,--count,--count-fast,--fast,--blocks-ordered,--id-not-matching,--not-matching-id,--matching,--not-matching,--strict,--permissive,--use-cuda,--cuda,--flush,--full-xpath,--verbose,-v", "-.*" );
        if( unnamed.size() > 1 ) { std::cerr << "points-join: expected one file or stream to join, got: " << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        comma::name_value::parser parser( "filename", ';', '=', false );
        filter_csv = unnamed.empty() ? stdin_csv : parser.get< comma::csv::options >( unnamed[0] );
        if( filter_csv.fields.empty() ) { filter_csv.fields = "x,y,z"; }
        if( !options.exists( "--count,--count-fast" ) && append_nearest && stdin_csv.binary() && !filter_csv.binary() ) { std::cerr << "points-join: stdin stream binary and filter stream ascii: this combination is not supported" << std::endl; return 1; }
        const std::vector< std::string >& v = comma::split( filter_csv.fields, ',' );
        const std::vector< std::string >& w = comma::split( stdin_csv.fields, ',' );
        const std::vector< std::string > normal_strings = { "normal" };
        const bool filter_triangulated = find_in_fields( v, { "corners" } );
        use_radius = stdin_csv.has_field( "radius" );
        use_normal = find_in_fields( w, normal_strings ) && find_in_fields( v, normal_strings );
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
        if( unnamed.empty() ) { return join_impl_< Eigen::Vector3d >::run( options, unnamed.empty() ); }
        return filter_triangulated ? join_impl_< snark::triangle >::run( options, unnamed.empty() ) : join_impl_< Eigen::Vector3d >::run( options, unnamed.empty() );
    }
    catch( std::exception& ex ) { std::cerr << "points-join: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-join: unknown exception" << std::endl; }
    #ifdef SNARK_USE_CUDA
    cuda_deallocate();
    #endif
    return 1;
}
