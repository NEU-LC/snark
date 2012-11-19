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
#include <boost/shared_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>
#include <snark/math/interval.h>
#include <snark/point_cloud/voted_tracking.h>
#include <snark/point_cloud/voxel_map.h>
#include <snark/visiting/eigen.h>

/// @author vsevolod vlaskine

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "keep partition ids consistent in two subsequent partitioned" << std::endl;
    std::cerr << "3d point blocks (by simple voting algorithm)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: cat scans.csv | points-track-partitions [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    --origin=<origin>: voxel grid origin; default: 0,0,0" << std::endl;
    std::cerr << "    --resolution=<resolution>: voxel grid resolution; default: 0.2" << std::endl;
    std::cerr << "    --verbose, -v: debug output on" << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << "        default fields: x,y,z,block,id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat scans.xyz_id_block.csv | points-track-partitions > scans.tracked.csv" << std::endl;
    std::cerr << "    cat scans.t_xyz_id_block.bin | points-track-partitions --fields=,x,y,z,id,block --binary=%t%3d%2ui > scans.tracked.bin" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

class voxel;

struct input_t
{
    Eigen::Vector3d point;
    comma::uint32 block;
    comma::uint32 id;
    mutable ::voxel* voxel;

    input_t() : block( 0 ), id( 0 ), voxel( NULL ) {}
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const input_t& p, Visitor& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
    }
    
    template < typename Key, class Visitor >
    static void visit( const Key&, input_t& p, Visitor& v )
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

        void add( const input_t& p )
        {
            unsigned int count = 1;
            map_t_::iterator it = map_.find( p.id );
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
        typedef std::map< comma::uint32, unsigned int > map_t_;
        map_t_ map_;
        unsigned int size_;
        Eigen::Vector3d sum_;
        comma::uint32 id_;
        unsigned int count_;
};

typedef std::deque< std::pair< voxel*, boost::optional< comma::uint32 > > > partition_t;
typedef std::map< comma::uint32, partition_t > partition_map;
struct id_element // quick and dirty
{
    comma::uint32 id;
    partition_t* partition;
    id_element() : id( 0 ), partition( NULL ) {}
    id_element( comma::uint32 id, partition_t* partition ) : id( id ), partition( partition ) {}
};
typedef std::multimap< comma::uint32, id_element > id_map;

static boost::optional< comma::uint32 > get_previous_id( partition_t::const_iterator it ) { return it->second; }

std::pair< boost::shared_ptr< snark::voxel_map< voxel, 3 > >, boost::shared_ptr< snark::voxel_map< voxel, 3 > > > voxels;
typedef std::pair< input_t, std::string > pair_t;
typedef std::deque< pair_t > points_t;
static points_t points;
static comma::uint32 vacant = 0;
static comma::csv::options csv;
static bool verbose;
static Eigen::Vector3d origin;
static Eigen::Vector3d resolution;
static comma::signal_flag is_shutdown;

static void match() // todo: refactor this bloody mess, once voxel grid is refactored!
{
    partition_map partitions;
    for( snark::voxel_map< voxel, 3 >::iterator it = voxels.second->begin(); it != voxels.second->end(); ++it )
    {
        comma::uint32 current_id = it->second.id();
        boost::optional< comma::uint32 > previous_id;
        snark::voxel_map< voxel, 3 >::const_iterator v = voxels.first->find( it->second.mean() );
        if( v != voxels.first->end() ) { previous_id = v->second.id(); }
        partitions[ current_id ].push_back( std::make_pair( &it->second, previous_id ) );
    }
    id_map ids;
    for( partition_map::iterator it = partitions.begin(); it != partitions.end(); ++it )
    {
        comma::uint32 id = snark::voted_tracking( it->second.begin(), it->second.end(), get_previous_id, vacant );
        if( id == vacant ) { ++vacant; }
        ids.insert( std::make_pair( id, id_element( id, &( it->second ) ) ) );
    }
    id_map::iterator largest;
    for( id_map::iterator it = ids.begin(); it != ids.end(); ) // quick and dirty
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
    for( id_map::iterator it = ids.begin(); it != ids.end(); ++it ) // quick and dirty
    {
        for( partition_t::iterator j = it->second.partition->begin(); j != it->second.partition->end(); ++j )
        {
            j->first->set( it->second.id );
        }
    }
}

static void read_block_() // todo: implement generic reading block
{
    points.clear();
    voxels.second.reset( new snark::voxel_map< voxel, 3 >( origin, resolution ) );
    static boost::optional< pair_t > last;
    static comma::uint32 block_id = 0;
    static comma::csv::input_stream< input_t > istream( std::cin, csv );
    while( true )
    {
        if( last )
        {
            block_id = last->first.block;
            snark::voxel_map< voxel, 3 >::iterator it = voxels.second->touch_at( last->first.point );
            it->second.add( last->first );
            last->first.voxel = &it->second;
            points.push_back( *last );
            last.reset();
        }
        if( is_shutdown || std::cout.bad() || std::cin.bad() || std::cin.eof() ) { break; }
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
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        verbose = options.exists( "--verbose,-v" );
        origin = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--origin", "0,0,0" ) );
        double r = options.value< double >( "--resolution", 0.2 );
        resolution = Eigen::Vector3d( r, r, r );
        csv = comma::csv::options( options );
        if( csv.fields == "" ) { csv.fields = "x,y,z,block,id"; }
        if( !csv.has_field( "block" ) ) { std::cerr << "points-track-partitions: expected field 'block'" << std::endl; return 1; }
        if( !csv.has_field( "id" ) ) { std::cerr << "points-track-partitions: expected field 'id'" << std::endl; return 1; }
        comma::csv::output_stream< input_t > ostream( std::cout, csv );
        while( !is_shutdown && std::cin.good() && !std::cin.eof() && std::cout.good() )
        {
            read_block_();
            if( is_shutdown ) { break; }
            if( voxels.first ) { match(); }
            for( points_t::iterator it = points.begin(); it != points.end(); ++it )
            {
                it->first.id = it->first.voxel->id();
                ostream.write( it->first, it->second );
            }
            std::swap( voxels.first, voxels.second );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-track-partitions: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-track-partitions: unknown exception" << std::endl; }
}
