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
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/sync/synchronized.h>
#include <comma/visiting/traits.h>
#include <snark/point_cloud/partition.h>
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
    std::cerr << "           supports delta input, see --delta field description" << std::endl;
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
    std::cerr << "    netcat localhost 1234 | points-to-partitions --fields=\",x,y,z,,,block\" --delta --discard > partitions.csv" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

class stopwatch_type // quick and dirty
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
static boost::mutex logMutex; // real quick and dirty
#define STDERR verbose && boost::mutex::scoped_lock( logMutex ) && std::cerr

static std::size_t min_points_per_voxel = 1;
static std::size_t min_voxels_per_partition = 1;
static std::size_t min_points_per_partition = 1;
static Eigen::Vector3d resolution;
static bool delta;
comma::csv::options csv;
static comma::uint32 minId;
static bool discard;

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
    static void setVisited( voxel& e, bool v ) { e.visited = v; }
    static comma::uint32 id( const voxel& e ) { return e.id; }
    static void setId( voxel& e, comma::uint32 id ) { e.id = id; }
};

namespace points_to_partitions {

struct point
{
    Eigen::Vector3d point;
    bool flag;
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
        v.apply( "flag", p.flag );
    }

    template < typename K, typename V > static void visit( const K&, const points_to_partitions::point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "flag", p.flag );
    }
};

} } // namespace ark { namespace visiting {

typedef Comms::Csv::detail::Delta< points_to_partitions::point >::Data pointData; // real quick and dirty
typedef Comms::Csv::Accumulated< points_to_partitions::point > accumulated_type;
static comma::uint32 listBlock;
typedef accumulated_type::List List;
static List list;
static boost::condition listReady;
static boost::mutex mutex;
static bool is_shutdown = false;
typedef comma::synchronized< boost::scoped_ptr< accumulated_type > > synchronized_type;
static synchronized_type accumulated;
static boost::scoped_ptr< accumulated_type > accumulated_helper;
static bool flag_present;

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
    const partitions& partitions = snark::equivalence_classes< It, nit_type, methods >( voxels.begin(), voxels.end(), minId );
    stopwatch.stop();
    STDERR << "points-to-partitions: block: " << block << "; partitioned; elapsed: " << stopwatch.elapsed() << " seconds" << std::endl;
    STDERR << "points-to-partitions: block: " << block << "; found " << partitions.size() << " partitions; elapsed: " << stopwatch.elapsed() << " seconds" << std::endl;
    if( partitions.empty() ) { return; }
    STDERR << "points-to-partitions: block: " << block << "; outputting partitions" << "..." << std::endl;
    stopwatch.reset();
    std::deque< bool > discarded( partitions.rbegin()->first - minId + 1, false );
    for( partitions::const_iterator i = partitions.begin(); i != partitions.end(); ++i )
    {
        if( i->second.size() < min_voxels_per_partition )
        {
            discarded[ i->first - minId ] = true;
        }
        else if( min_points_per_partition > min_voxels_per_partition * min_points_per_voxel ) // watch performance
        {
            std::size_t size = 0;
            for( set_type::const_iterator j = i->second.begin(); j != i->second.end(); size += ( *j++ )->count );
            if( size < min_points_per_partition ) { discarded[ i->first - minId ] = true; }
        }
    }
    for( List::const_iterator it = r.begin(); it != r.end(); ++it )
    {
        comma::uint32 id = it->value.id();
        if(    it->value.voxel == NULL
            || it->value.voxel->count < min_points_per_voxel
            || discarded[ it->value.voxel->id - minId ] )
        {
            id = std::numeric_limits< comma::uint32 >::max();
            if( discard ) { continue; }
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
        if( std::cout.bad() || std::cout.eof() ) { is_shutdown = true; }
    }
    stopwatch.stop();
    STDERR << "points-to-partitions: block: " << block << "; partitions output: elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
}

static void run()
{
    boost::mutex::scoped_lock lock( mutex );
    while( !is_shutdown )
    {
        listReady.wait( lock );
        if( is_shutdown ) { break; }
        partition( list, listBlock );
        list.clear();
    }
}

int main( int ac, char** av )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif

        comma::command_line_options options( ac, av );
        comma::signal_flag shutdown_flag;
        if( options.exists( "--help,-h" ) ) { usage(); }
        csv = comma::csv::options( options, "x,y,z" );
        min_points_per_voxel = options.value( "--min-points-per-voxel", 1u );
        min_voxels_per_partition = options.value( "--min-voxels-per-partition", 1u );
        min_points_per_partition = options.value( "--min-points-per-partition", 1u );
        if( min_points_per_voxel == 0 ) { std::cerr << "points-to-partitions: expected minimum number of points in a non-empty voxel, got zero" << std::endl; usage(); }
        verbose = options.exists( "--verbose,-v" );
        double r = options.value( "--resolution", double( 0.2 ) );
        resolution = Eigen::Vector3d( r, r, r );
        bool realtime = options.exists( "--discard,-d" );
        delta = options.exists( "--delta" );
        minId = options.value( "--min-id", 0 );
        discard = options.exists( "--discard-points" );
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        flag_present = std::find( v.begin(), v.end(), "flag" ) != v.end();
        if( delta && std::find( v.begin(), v.end(), "block" ) == v.end() )
        {
            std::cerr << "points-to-partitions: --delta specified, thus expected \"block\" field present, got \"" << csv.fields << "\"" << std::endl;
            usage();
        }
        {
            synchronized_type::scoped_transaction t( accumulated );
            t->reset( new accumulated_type( std::cin, csv, delta ) );
        }
        accumulated_helper.reset( new accumulated_type( std::cin, csv, delta ) ); // quick and dirty
        boost::scoped_ptr< boost::thread > thread;
        if( realtime ) { thread.reset( new boost::thread( &run ) ); }
        stopwatch stopwatch( verbose );
        // todo: refactor, using snark::ConsumerOf
        #ifdef PROFILE
        ProfilerStart( "points-to-partitions.prof" ); {
        #endif
        while( !shutdown_flag
               && std::cin.good() && !std::cin.eof()
               && std::cout.good() && !std::cout.eof() )
        {
            const List* r;
            comma::uint32 block;
            {
                synchronized_type::scoped_transaction t( accumulated );
                if( !delta ) { ( *t )->clear(); }
                STDERR << "points-to-partitions: reading block..." << std::endl;
                stopwatch.reset();
                r = ( *t )->read();
                stopwatch.stop();
                if( shutdown_flag ) { STDERR << "points-to-partitions: interrupted with signal" << std::endl; break; }
                if( !r || r->empty() ) { STDERR << "points-to-partitions: end of data" << std::endl; break; }
                block = ( *t )->block();
                STDERR << "points-to-partitions: read block " << block << " of " << r->size() << " points: elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
            }
            if( !realtime ) { partition( *r, block ); continue; }
            boost::mutex::scoped_try_lock lock( mutex );
            if( !lock )
            {
                STDERR << "points-to-partitions: skipped block " << block << " of " << r->size() << " points: elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
                continue;
            }
            STDERR << "points-to-partitions: copying block " << block << " of " << r->size() << " points..." << std::endl;
            stopwatch.reset();
            listBlock = block;
            list.clear();
            list.insert( list.end(), r->begin(), r->end() );
            stopwatch.stop();
            STDERR << "points-to-partitions: copied block " << block << " of " << r->size() << " points: elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
            listReady.notify_one();
        }
        #ifdef PROFILE
        ProfilerStop(); }
        #endif
        if( shutdown_flag ) { STDERR << "points-to-partitions: caught signal" << std::endl; }
        else { STDERR << "points-to-partitions: no more data" << std::endl; }
        if( realtime )
        {
            STDERR << "points-to-partitions: shutting down thread..." << std::endl;
            is_shutdown = true;
            listReady.notify_one();
            thread->join();
            STDERR << std::cerr << "points-to-partitions: done" << std::endl;
        }
        //if( csv->binary() ) { binary::flush(); }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-partitions: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-partitions: unknown exception" << std::endl; }
    usage();
}
