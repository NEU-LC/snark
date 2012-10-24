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

#ifdef WIN32
#include <stdio.h>
#include <fcntl.h>
#include <io.h>
#endif

#include <deque>
#include <iostream>
#include <map>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <snark/visiting/eigen.h>
#include <snark/math/interval.h>
#include <comma/visiting/traits.h>
#include <snark/Comms/Accumulated.h>
#include <snark/point_cloud/VotedTracking.h>
#include <snark/point_cloud/voxel_grid.h>

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "keep partition ids consistent in two subsequent partitioned" << std::endl;
    std::cerr << "3d point blocks (by simple voting algorithm)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: cat scans.csv | points-track-partitions [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    --resolution=<resolution>: voxel grid resolution; default 0.2" << std::endl;
    std::cerr << "    --verbose, -v: debug output on" << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << "        default fields: x,y,z,block,id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "example" << std::endl;
    std::cerr << "    cat scans.bin | points-track-partitions --fields=,x,y,z,id,block --binary=%t%3d%2ui" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

class Stopwatch // quick and dirty
{
    public:
        Stopwatch( bool verbose ) : m_verbose( verbose ) { reset(); }
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

class voxel;

struct pointWithId
{
    Eigen::Vector3d point;
    comma::uint32 block;
    comma::uint32 id;
    mutable voxel* voxel;

    pointWithId() : block( 0 ), id( 0 ), voxel( NULL ) {}
};

namespace comma { namespace visiting {

template <> struct traits< pointWithId >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const pointWithId& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, pointWithId& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
    }
};

} } // namespace comma { namespace visiting {

class voxel
{
    public:
        voxel() : size_( 0 ), sum_( 0, 0, 0 ), id_( 0 ), count_( 0 ) {}

        void add( const pointWithId& p )
        {
            unsigned int count = 1;
            Map_::iterator it = map_.find( p.id );
            if( it == map_.end() ) { map_[ p.id ] = 1; }
            else { count = ++( it->second ); }
            if( count > count_ ) { id_ = p.id; count_ = count; }
            ++size_;
            sum_ += p.point;
            p.voxel = this;
        }

        Eigen::Vector3d mean() const { return sum_ / count_; }

        comma::uint32 id() const { return id_; }

        void set( comma::uint32 v ) { id_ = v; }

    private:
        typedef std::map< comma::uint32, unsigned int > Map_;
        Map_ map_;
        unsigned int size_;
        Eigen::Vector3d sum_;
        comma::uint32 id_;
        unsigned int count_;
};
    
typedef std::deque< std::pair< voxel*, boost::optional< comma::uint32 > > > partition;
typedef std::map< comma::uint32, partition > partitionMap;
struct IdElement // quick and dirty
{
    comma::uint32 id;
    partition* partition;
    IdElement() : id( 0 ), partition( NULL ) {}
    IdElement( comma::uint32 id, partition* partition ) : id( id ), partition( partition ) {}
};
typedef std::multimap< comma::uint32, IdElement > IdMap;

static boost::optional< comma::uint32 > getPreviousId( partition::const_iterator it ) { return it->second; }

std::pair< boost::shared_ptr< snark::voxel_grid< voxel > >, boost::shared_ptr< snark::voxel_grid< voxel > > > voxels;
snark::Comms::Csv::Accumulated< pointWithId >::List list;
comma::uint32 vacant = 0;
comma::csv::options csv;

static void match() // todo: refactor this bloody mess, once voxel grid is refactored!
{
    Stopwatch stopwatch( verbose );
    verbose && std::cerr << "points-track-partitions: loading partition ids..." << std::endl;
    stopwatch.reset();
    partitionMap partitions;
    for( snark::voxel_grid< voxel >::iterator it = voxels.second->begin(); it != voxels.second->end(); ++it )
    {
        comma::uint32 currentId = it->id();
        boost::optional< comma::uint32 > previousId;
        Eigen::Vector3d mean = it->mean();
        if( voxels.first->covers( mean ) )
        {
            snark::voxel_grid< voxel >::index i = voxels.first->index_of( mean );
            voxel* voxel = voxels.first->find( i );
            if( voxel ) { previousId = voxel->id(); }
        }
        partitions[currentId].push_back( std::make_pair( &( *it ), previousId ) );
    }
    stopwatch.stop();
    verbose && std::cerr << "points-track-partitions: loaded partition ids; elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
    verbose && std::cerr << "points-track-partitions: matching partition ids..." << std::endl;
    stopwatch.reset();
    IdMap ids;
    for( partitionMap::iterator it = partitions.begin(); it != partitions.end(); ++it )
    {
        comma::uint32 id = snark::voted_tracking( it->second.begin(), it->second.end(), getPreviousId, vacant );
        if( id == vacant ) { ++vacant; }
        ids.insert( std::make_pair( id, IdElement( id, &( it->second ) ) ) );
    }
    IdMap::iterator largest;
    for( IdMap::iterator it = ids.begin(); it != ids.end(); ) // quick and dirty
    {
        largest = it++;
        for( ; it != ids.end() && it->first == largest->first; ++it )
        {
            if( largest->second.partition->size() >= it->second.partition->size() )
            {
                it->second.id = vacant++;
            }
            else
            {
                largest->second.id = vacant++;
                largest = it;
            }
        }
    }
    for( IdMap::iterator it = ids.begin(); it != ids.end(); ++it ) // quick and dirty
    {
        for( partition::iterator j = it->second.partition->begin(); j != it->second.partition->end(); ++j )
        {
            j->first->set( it->second.id );
        }
    }
    stopwatch.stop();
    verbose && std::cerr << "points-track-partitions: matched ids; elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        verbose = options.exists( "--verbose,-v" );
        double r = options.value< double >( "--resolution", 0.2 );
        Eigen::Vector3d resolution( r, r, r );
        csv = comma::csv::options( options );
        if( csv.fields == "" ) { csv.fields = "x,y,z,block,id"; }
        #ifdef WIN32
            if( csv.binary() ) { _setmode( _fileno( stdin ), _O_BINARY ); }
        #endif
        snark::Comms::Csv::Accumulated< pointWithId > accumulated( std::cin, csv, false );
        comma::csv::output_stream< pointWithId > ostream( std::cout, csv );
        comma::signal_flag isShutdown;
        Stopwatch stopwatch( verbose );
        while( !isShutdown )
        {
            verbose && std::cerr << "points-track-partitions: reading block..." << std::endl;
            stopwatch.reset();
            accumulated.read( list );
            stopwatch.stop();
            if( list.empty() ) { break; }
            verbose && std::cerr << "points-track-partitions: got block " << accumulated.block() << " of " << list.size() << " elements; elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
            verbose && std::cerr << "points-track-partitions: block " << accumulated.block() << ": filling voxel grid..." << std::endl;
            stopwatch.reset();
            snark::Extents< Eigen::Vector3d > extents;
            for( snark::Comms::Csv::Accumulated< pointWithId >::List::const_iterator it = list.begin(); it != list.end(); ++it )
            {
                if( it->value.id > vacant ) { vacant = it->value.id; }
                extents.add( it->value.point );
            }
            verbose && std::cerr << "points-track-partitions: block " << accumulated.block() << ": extents: " << extents.min().x() << "," << extents.min().y() << "," << extents.min().z() << " to " << extents.max().x() << "," << extents.max().y() << "," << extents.max().z() << std::endl;
            Eigen::Vector3d floor = extents.min() - resolution / 2;
            Eigen::Vector3d ceil = extents.min() + resolution / 2;
            extents.add( Eigen::Vector3d( snark::math::floor( floor.x() ), snark::math::floor( floor.y() ), snark::math::floor( floor.z() ) ) );
            extents.add( Eigen::Vector3d( snark::math::ceil( ceil.x() ), snark::math::ceil( ceil.y() ), snark::math::ceil( ceil.z() ) ) );
            voxels.second.reset( new snark::voxel_grid< voxel >( extents, resolution ) );
            for( snark::Comms::Csv::Accumulated< pointWithId >::List::const_iterator it = list.begin(); it != list.end(); ++it )
            {
                voxels.second->touch_at( it->value.point )->add( it->value );
            }
            stopwatch.stop();
            verbose && std::cerr << "points-track-partitions: block " << accumulated.block() << ": filled voxel grid; elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
            if( voxels.first ) { match(); }
            verbose && std::cerr << "points-track-partitions: block " << accumulated.block() << ": outputting..." << std::endl;
            stopwatch.reset();
            for( snark::Comms::Csv::Accumulated< pointWithId >::List::iterator it = list.begin(); it != list.end(); ++it )
            {
                it->value.block = accumulated.block();
                it->value.id = it->value.voxel->id();
                ostream.write( it->value, it->key );
            }
            stopwatch.stop();
            verbose && std::cerr << "points-track-partitions: block " << accumulated.block() << ": output done; elapsed " << stopwatch.elapsed() << " seconds" << std::endl;
            voxels.first.reset();
            voxels.first = voxels.second;
            voxels.second.reset();
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-track-partitions: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-track-partitions: unknown exception" << std::endl; }
    usage();
}

