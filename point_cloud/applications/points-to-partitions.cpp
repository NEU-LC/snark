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
    std::cerr << "        --discard-points: if present, discard points from voxels having less than --min-points-per-voxel" << std::endl;
    std::cerr << "        --min-id: minimum partition id; default 0" << std::endl;
    std::cerr << "        --min-points-per-voxel <n>: min number of points in a non-empty voxel; default 1" << std::endl;
    std::cerr << "        --min-voxels-per-partition <n>: min number of voxels in a partition; default 1" << std::endl;
    std::cerr << "        --min-points-per-partition <n>: min number of points in a partition; default 1" << std::endl;
    std::cerr << "        --resolution <resolution>: default: 0.2 metres" << std::endl;
    std::cerr << "    data flow options:" << std::endl;
    std::cerr << "        --delta: if present, input is delta (points added and removed)" << std::endl;
    std::cerr << "                 if absent, clear points after partitioning a data block" << std::endl;
    std::cerr << "        --discard,-d: if present, partition as many points as possible, discard the rest" << std::endl;
    std::cerr << "        --verbose, -v: output progress info" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<fields>" << std::endl;
    std::cerr << "    required fields: x,y,z" << std::endl;
    std::cerr << "    default: \"x,y,z\"" << std::endl;
    std::cerr << "    block: data block id, if present, accumulate and partition each data block separately" << std::endl;
    std::cerr << "           if absent, read until the end of file/stream and then partition" << std::endl;
    std::cerr << "    flag: if present, partition only the points with this field equal 1" << std::endl;
    std::cerr << "          skipped points will have partition id as max int32" << std::endl;
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
static bool discard_points;
static boost::scoped_ptr< snark::partition > partition;

struct voxel
{
    bool visited;
    comma::uint32 id;
    std::size_t count;

    voxel() : visited( false ), id( 0 ), count( 0 ) {}
};

struct methods // quick and dirty
{
    static bool skip( const voxel& e ) { return e.count < min_points_per_voxel; }
    static bool same( const voxel& lhs, const voxel& rhs ) { return true; }
    static bool visited( const voxel& e ) { return e.visited; }
    static void set_visited( voxel& e, bool v ) { e.visited = v; }
    static comma::uint32 id( const voxel& e ) { return e.id; }
    static void set_id( voxel& e, comma::uint32 id ) { e.id = id; }
};

namespace points_to_partitions {

struct point
{
    Eigen::Vector3d point;
    bool flag;
    comma::uint32 block;
    mutable voxel* voxel;

    point() : point( 0, 0, 0 ), voxel( NULL ) {}
    comma::uint32 id() const { return voxel == NULL ? std::numeric_limits< comma::uint32 >::max() : voxel->id; }
};

} // namespace points_to_partitions {

namespace comma { namespace visiting {

template <> struct traits< points_to_partitions::point >
{
    template < typename K, typename V > static void visit( const K&, points_to_partitions::point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "flag", p.flag );
    }

    template < typename K, typename V > static void visit( const K&, const points_to_partitions::point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "flag", p.flag );
    }
};

} } // namespace ark { namespace visiting {

struct block // quick and dirty, no optimization for now
{
    typedef std::pair< point, std::string > pair_t;
    typedef std::deque< pair_t > pairs_t;
    
    pairs_t points;
    comma::uint32 id;
    bool empty;
    boost::scoped_ptr< snark::partition > partition;
    
    block() : id( 0 ) {}
    void clear() { partition.reset(); points.clear(); empty = true; }
};

static boost::array< block, 3 > blocks;
static bool has_flag;
static comma::signal_flag shutdown_flag;

static unsigned int read_block_( ::tbb::flow_control& flow )
{
    static boost::optional< block::pair_t > last;
    static comma::uint32 block_id = 0;
    static unsigned int index = 0;
    for( unsigned int i = 0; i < blocks.size(); ++i ) { if( blocks[i].empty ) { index = i; break; } }
    blocks[index].clear();
    static comma::csv::input_stream< points_to_partitions::point > istream( std::cin, csv );
    while( !shutdown_flag && std::cout.good() )
    {
        if( shutdown_flag || std::cout.bad() || std::cin.bad() || std::cin.eof() ) { flow.stop(); break; }
        if( last )
        {
            blocks[index].id = block_id;
            blocks[index].points.push_back( *last );
        }
        const points_to_partitions::point* p = istream.read();
        if( !p ) { flow.stop(); break; }
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

static void write_block_( unsigned int index )
{

}

static unsigned int partition_( unsigned int index )
{
    STDERR << "velodyne-to-mesh-ground: partitioning..." << std::endl;
    boost::optional< snark::math::interval< double, 3 > > extents;
    for( std::size_t i = 0; i < blocks[index].points.size(); ++i )
    {
        if( points[i].label && points[i].label == Ark::Robotics::MeshGround::Label::non_ground )
        {
            if( !extents )
            {
                extents = snark::math::interval< double, 3 >( points[i].point );
            }
            else
            {
                extents = extents->hull( points[i].point );
            }
        }
    }
    snark::partition partition( *extents, resolution, min_points_per_voxel );
    for( std::size_t i = 0; i < points.size(); ++i )
    {
        if( points[i].label && points[i].label == Ark::Robotics::MeshGround::Label::non_ground )
        {
            points[i].partition = &partition.insert( points[i].point );
        }
    }
    partition.commit( min_voxels_per_partition, min_points_per_partition, output_ground ? 1 : 0 );
    if( verbose ) { stop = boost::posix_time::microsec_clock::universal_time(); }
    if( verbose ) { std::cerr << "velodyne-to-mesh-ground: partitioned; elapsed " << double( ( stop - start ).total_microseconds() ) / 1000000 << " seconds" << std::endl; }
    if( verbose ) { std::cerr << "velodyne-to-mesh-ground: outputting partitions..." << std::endl; }
    for( std::size_t i = 0; i < points.size(); ++i )
    {
        const InputPoint& point = points[i];
        bool is_ground = output_ground && point.element && point.element->value.label == Ark::Robotics::MeshGround::Label::ground;
        if( !( point.partition && *point.partition ) && !( output_ground && is_ground ) ) { continue; }
        comma::uint32 id = is_ground ? 0 : **point.partition;
        if( csv.binary() )
        {
            ostream->write( point );
            std::cout.write( reinterpret_cast< const char* >( &id ), sizeof( comma::uint32 ) );
        }
        else
        {
            std::string s;
            ostream->ascii().ascii().put( point, s );
            std::cout << s << csv.delimiter << id << std::endl;
        }
    }
    std::cout.flush();
    STDERR << "velodyne-to-mesh-ground: outputting partitions done" << std::endl;
    
    
}

static void partition( const List& r, comma::uint32 block )
{
    snark::Extents< Eigen::Vector3d > extents;
    stopwatch_type stopwatch( verbose );
    STDERR << "points-to-partitions: block: " << block << "; got list to partition, list size = " << r.size() << std::endl;
    stopwatch.reset();
    for( List::const_iterator it = r.begin(); it != r.end(); ++it )
    {
        if( it->value.flag ) { extents.add( it->value.point ); }
    }
    stopwatch.stop();
    STDERR << "points-to-partitions: block: " << block << "; extents calculated: (" << extents.min().x() << "," << extents.min().y() << "," << extents.min().z() << ") to (" << extents.max().x() << "," << extents.max().y() << "," << extents.max().z() << "): elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
    Eigen::Vector3d floor = extents.min() - resolution / 2;
    Eigen::Vector3d ceil = extents.max() + resolution / 2;
    extents.add( Eigen::Vector3d( std::floor( floor.x() ), std::floor( floor.y() ), std::floor( floor.z() ) ) );
    extents.add( Eigen::Vector3d( std::ceil( ceil.x() ), std::ceil( ceil.y() ), std::ceil( ceil.z() ) ) );
    voxel_grid< voxel > voxels( extents, resolution );
    STDERR << "points-to-partitions: block: " << block << "; filling voxel grid..." << std::endl; // todo: parallelize with a double buffer, if it becomes a bottleneck
    stopwatch.reset();
    for( List::const_iterator i = r.begin(); i != r.end(); ++i )
    {
        if( !i->value.flag ) { continue; }
        voxel* voxel = voxels.touch_at( i->value.point );
        i->value.voxel = voxel;
        if( !voxel ) { continue; }
        ++voxel->count;
    }
    stopwatch.stop();
    STDERR << "points-to-partitions: block: " << block << "; voxel grid filled: elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
    STDERR << "points-to-partitions: block: " << block << "; partitioning voxel grid..." << std::endl;
    stopwatch.reset();
    typedef std::list< voxel_grid< voxel >::iterator > set_type;
    typedef std::map< comma::uint32, std::list< voxel_grid< voxel >::iterator > > partitions;
    typedef voxel_grid< voxel >::iterator It;
    typedef voxel_grid< voxel >::neighbourhood_iterator nit_type;
    const partitions& partitions = snark::equivalence_classes< It, nit_type, methods >( voxels.begin(), voxels.end(), min_id );
    stopwatch.stop();
    STDERR << "points-to-partitions: block: " << block << "; partitioned; elapsed: " << stopwatch.elapsed() << " seconds" << std::endl;
    STDERR << "points-to-partitions: block: " << block << "; found " << partitions.size() << " partitions; elapsed: " << stopwatch.elapsed() << " seconds" << std::endl;
    if( partitions.empty() ) { return; }
    STDERR << "points-to-partitions: block: " << block << "; outputting partitions" << "..." << std::endl;
    stopwatch.reset();
    std::deque< bool > discarded( partitions.rbegin()->first - min_id + 1, false );
    for( partitions::const_iterator i = partitions.begin(); i != partitions.end(); ++i )
    {
        if( i->second.size() < min_voxels_per_partition )
        {
            discarded[ i->first - min_id ] = true;
        }
        else if( min_points_per_partition > min_voxels_per_partition * min_points_per_voxel ) // watch performance
        {
            std::size_t size = 0;
            for( set_type::const_iterator j = i->second.begin(); j != i->second.end(); size += ( *j++ )->count );
            if( size < min_points_per_partition ) { discarded[ i->first - min_id ] = true; }
        }
    }
    for( List::const_iterator it = r.begin(); it != r.end(); ++it )
    {
        comma::uint32 id = it->value.id();
        if(    it->value.voxel == NULL
            || it->value.voxel->count < min_points_per_voxel
            || discarded[ it->value.voxel->id - min_id ] )
        {
            id = std::numeric_limits< comma::uint32 >::max();
            if( discard_points ) { continue; }
        }
        std::string s( it->key.length(), '\0' ); // quick and dirty
        it->key.copy( &s[0], it->key.length() );
        accumulated_helper->setBlock( s, block ); // quick and dirty, hideous
        if( csv.binary() ) // quick and dirty
        {
            std::cout.write( &s[0], s.length() ); // Binary::output( s ); // quick and dirty
            std::cout.write( ( const char* )( &id ), sizeof( comma::uint32 ) );
        }
        else
        {
            std::cout << s << csv.delimiter << id << std::endl; // quick and dirty
        }
    }
    stopwatch.stop();
    STDERR << "points-to-partitions: block: " << block << "; partitions output: elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
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
        discard_points = options.exists( "--discard-points" );
        has_flag = csv.has_field( "flag" );        
        ::tbb::filter_t< unsigned int, unsigned int > partition_filter( ::tbb::filter::serial_in_order, &partition_ );
        ::tbb::filter_t< unsigned int, void > write_filter( ::tbb::filter::serial_in_order, &write_block_ );
        #ifdef PROFILE
        ProfilerStart( "points-to-partitions.prof" ); {
        #endif
        if( discard )
        { 
            snark::tbb::bursty_reader< unsigned int > read_filter( &read_block_ );
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
    usage();
}
