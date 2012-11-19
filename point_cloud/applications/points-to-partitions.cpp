// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

/// @author vsevolod vlaskine

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif
#include <deque>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <vector>
#include <tbb/pipeline.h>
#include <tbb/task_scheduler_init.h>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/sync/synchronized.h>
#include <comma/visiting/traits.h>
#include <snark/math/interval.h>
#include <snark/point_cloud/partition.h>
#include <snark/tbb/bursty_reader.h>
#include <snark/visiting/eigen.h>

#ifdef PROFILE
#include <google/profiler.h>
#endif

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "partition 3d point cloud" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-to-partitions [<options>] > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    partitioning options:" << std::endl;
    std::cerr << "        --min-id: minimum partition id; default 0" << std::endl;
    std::cerr << "        --min-points-per-voxel <n>: min number of points in a non-empty voxel; default 1" << std::endl;
    std::cerr << "        --min-voxels-per-partition <n>: min number of voxels in a partition; default 1" << std::endl;
    std::cerr << "        --min-points-per-partition <n>: min number of points in a partition; default 1" << std::endl;
    std::cerr << "        --resolution <resolution>: default: 0.2 metres" << std::endl;
    std::cerr << "    data flow options:" << std::endl;
    std::cerr << "        --discard,-d: if present, partition as many points as possible, discard the rest" << std::endl;
    std::cerr << "        --output-all: output all points, even non-partitioned; the latter with id: max uint32" << std::endl;
    std::cerr << "        --verbose, -v: output progress info" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<fields>" << std::endl;
    std::cerr << "    required fields: x,y,z" << std::endl;
    std::cerr << "    default: \"x,y,z\"" << std::endl;
    std::cerr << "    block: data block id, if present, accumulate and partition each data block separately" << std::endl;
    std::cerr << "           if absent, read until the end of file/stream and then partition" << std::endl;
    std::cerr << "    flag: if present, partition only the points with this field equal 1" << std::endl;
    std::cerr << "          if --output-all specified; skipped points will have partition id as max uint32" << std::endl;
    std::cerr << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    partition all points in points.csv" << std::endl;
    std::cerr << "    cat points.csv | points-to-partitions > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    partition points.csv individually each data block (e.g. a scan) :" << std::endl;
    std::cerr << "    cat points.csv | points-to-partitions --fields=\",x,y,z,,,block\" > partitions.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    partition a point stream that gives delta only from a socket, if too slow, drop points:" << std::endl;
    std::cerr << "    netcat localhost 1234 | points-to-partitions --fields=\",x,y,z,,,block\" --discard > partitions.csv" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

class stopwatch_type // quick and dirty (and ugly)
{
    public:
        stopwatch_type( bool verbose ) : m_verbose( verbose ) { reset(); }
        void stop() { if( m_verbose ) { m_stop = boost::posix_time::microsec_clock::local_time(); } }
        void reset() { if( m_verbose ) { m_start = m_stop = boost::posix_time::microsec_clock::local_time(); } }
        boost::posix_time::time_duration elapsedDuration() const { return m_stop - m_start; }
        double elapsed() const { return double( elapsedDuration().total_microseconds() ) / 1000000; }
    private:
        bool m_verbose;
        boost::posix_time::ptime m_start;
        boost::posix_time::ptime m_stop;
};

static bool verbose;
static boost::mutex log_mutex; // real quick and dirty
#define STDERR verbose && boost::mutex::scoped_lock( log_mutex ) && std::cerr

static std::size_t min_points_per_voxel = 1;
static std::size_t min_voxels_per_partition = 1;
static std::size_t min_points_per_partition = 1;
static Eigen::Vector3d resolution;
static comma::csv::options csv;
static comma::uint32 min_id;
static bool discard;
static bool output_all;
static boost::scoped_ptr< snark::partition > partition;

struct input_t
{
    Eigen::Vector3d point;
    bool flag;
    comma::uint32 block;
    const boost::optional< comma::uint32 >* id;

    input_t() : point( 0, 0, 0 ), flag( true ), id( NULL ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K&, input_t& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "flag", p.flag );
    }

    template < typename K, typename V > static void visit( const K&, const input_t& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "flag", p.flag );
    }
};

} } // namespace ark { namespace visiting {

struct block // quick and dirty, no optimization for now
{
    typedef std::pair< input_t, std::string > pair_t;
    typedef std::deque< pair_t > pairs_t;
    
    pairs_t points;
    comma::uint32 id;
    bool empty;
    boost::scoped_ptr< snark::partition > partition;
    
    block() : id( 0 ), empty( true ) {}
    void clear() { partition.reset(); points.clear(); empty = true; }
};

static boost::array< block, 3 > blocks;
static comma::signal_flag shutdown_flag;

static unsigned int read_block_impl_( ::tbb::flow_control* flow = NULL )
{
    static boost::optional< block::pair_t > last;
    static comma::uint32 block_id = 0;
    static unsigned int index = 0;
    for( unsigned int i = 0; i < blocks.size(); ++i ) { if( blocks[i].empty ) { index = i; break; } }
    blocks[index].clear();
    blocks[index].empty = false;
    static comma::csv::input_stream< input_t > istream( std::cin, csv );
    while( true )
    {
        if( last )
        {
            block_id = last->first.block;
            blocks[index].id = block_id;
            blocks[index].points.push_back( *last );
            last.reset();
        }
        if( shutdown_flag || std::cout.bad() || std::cin.bad() || std::cin.eof() ) { if( flow ) { flow->stop(); } break; }
        const input_t* p = istream.read();
        if( !p ) { break; }
        std::string line;
        if( csv.binary() )
        {
            line.resize( csv.format().size() );
            ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
        }
        else
        { 
            line = comma::join( istream.ascii().last(), csv.delimiter );
        }
        last = std::make_pair( *p, line );
        if( p->block != block_id ) { break; }
    } 
    return index;
}

static unsigned int read_block_( ::tbb::flow_control& flow ) { return read_block_impl_( &flow ); }
static unsigned int read_block_bursty_() { return read_block_impl_(); }

static void write_block_( unsigned int index )
{
    for( std::size_t i = 0; i < blocks[index].points.size(); ++i )
    {
        const block::pair_t& p = blocks[index].points[i];
        if( !p.first.id && !output_all ) { continue; }
        comma::uint32 id = p.first.id && *p.first.id ? **p.first.id : std::numeric_limits< comma::uint32 >::max();
        std::cout.write( &p.second[0], p.second.size() );
        if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &id ), sizeof( comma::uint32 ) ); }
        else { std::cout << csv.delimiter << id << std::endl; }
    }
    std::cout.flush();
    blocks[index].clear();
}

static unsigned int partition_( unsigned int index )
{
    if( blocks[index].points.empty() ) { return index; }
    snark::math::closed_interval< double, 3 > extents;
    for( std::size_t i = 0; i < blocks[index].points.size(); ++i ) { extents.set_hull( blocks[index].points[i].first.point ); }
    blocks[index].partition.reset( new snark::partition( extents, resolution, min_points_per_voxel ) );
    for( std::size_t i = 0; i < blocks[index].points.size(); ++i )
    { 
        if( blocks[index].points[i].first.flag ) { blocks[index].points[i].first.id = &blocks[index].partition->insert( blocks[index].points[i].first.point ); }
    }
    blocks[index].partition->commit( min_voxels_per_partition, min_points_per_partition, min_id );
    return index;
}

int main( int ac, char** av )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        csv = comma::csv::options( options, "x,y,z" );
        min_points_per_voxel = options.value( "--min-points-per-voxel", 1u );
        min_voxels_per_partition = options.value( "--min-voxels-per-partition", 1u );
        min_points_per_partition = options.value( "--min-points-per-partition", 1u );
        if( min_points_per_voxel == 0 ) { std::cerr << "points-to-partitions: expected minimum number of points in a non-empty voxel, got zero" << std::endl; usage(); }
        verbose = options.exists( "--verbose,-v" );
        double r = options.value( "--resolution", double( 0.2 ) );
        resolution = Eigen::Vector3d( r, r, r );
        discard = options.exists( "--discard,-d" );
        min_id = options.value( "--min-id", 0 );
        output_all = options.exists( "--output-all" );
        ::tbb::filter_t< unsigned int, unsigned int > partition_filter( ::tbb::filter::serial_in_order, &partition_ );
        ::tbb::filter_t< unsigned int, void > write_filter( ::tbb::filter::serial_in_order, &write_block_ );
        #ifdef PROFILE
        ProfilerStart( "points-to-partitions.prof" ); {
        #endif
        if( discard )
        { 
            snark::tbb::bursty_reader< unsigned int > read_filter( &read_block_bursty_ );
            ::tbb::filter_t< void, void > filters = read_filter.filter() & partition_filter & write_filter;
            while( read_filter.wait() ) { ::tbb::parallel_pipeline( 3, filters ); }
        }
        else
        {
            ::tbb::filter_t< void, unsigned int > read_filter( ::tbb::filter::serial_in_order, &read_block_ );
            ::tbb::filter_t< void, void > filters = read_filter & partition_filter & write_filter;
            ::tbb::parallel_pipeline( 3, filters );
        }
        #ifdef PROFILE
        ProfilerStop(); }
        #endif
        if( shutdown_flag ) { STDERR << "points-to-partitions: caught signal" << std::endl; }
        else { STDERR << "points-to-partitions: no more data" << std::endl; }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-partitions: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-partitions: unknown exception" << std::endl; }
    return 1;
}
