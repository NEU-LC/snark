// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <deque>
#include <fstream>
#include <sstream>
#include <Eigen/Core>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../../point_cloud/voxel_map.h"
#include "../../../visiting/eigen.h"
#include "visit.h"

namespace snark { namespace points_calc { namespace visit {
 
std::string traits::usage()
{
    std::ostringstream oss;
    oss
        << "    visit: ...\n"
        << "        default input fields: \"x,y,z\"\n"
        << "        options\n"
        << "            --at=<filename>[;<csv-options>]; todo\n"
        << "            --radius=[<meters>]; todo\n"
        << "            --trajectory=<filename>[;<csv-options>]; todo\n"
        << "            --trajectory-field-of-view,--trajectory-fov,--fov=<angle>; default=3.14159265359; todo\n"
        << "            --trajectory-smoothen=<steps>; default=1; todo\n"
        << "            --unvisited-id,--unvisited=<id>; default=0\n"
        << "            --visited-id,--visited=<id>; default=1\n"
        << std::endl
        << "        examples\n"
        << "            todo\n"
        << std::endl;
    return oss.str();
}

struct input
{
    Eigen::Vector3d point;
    comma::uint32 block;
    comma::uint32 id;
    
    input(): point( 0, 0, 0 ), block( 0 ), id( 0 ) {}
};

struct record
{
    visit::input input;
    std::string buffer;
    comma::uint32 visited_id;
    
    record( const visit::input& input = visit::input(), const std::string& buffer = std::string(), comma::uint32 visited_id = 0 ): input( input ), buffer( buffer ), visited_id( visited_id ) {}
};

} } } // namespace snark { namespace points_calc { namespace visit {

namespace comma { namespace visiting {
    
template <> struct traits< snark::points_calc::visit::input >
{
    template < typename K, typename V > static void visit( const K&, snark::points_calc::visit::input& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
    }

    template < typename K, typename V > static void visit( const K&, const snark::points_calc::visit::input& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace points_calc { namespace visit {

std::string traits::input_fields() { return comma::join( comma::csv::names< snark::points_calc::visit::input >( false ), ',' ); }

std::string traits::input_format() { return comma::csv::format::value< snark::points_calc::visit::input >(); }

std::string traits::output_fields() { return "visited/id"; }

std::string traits::output_format() { return "ui"; }

typedef std::deque< visit::record* > voxel_t; // todo: watch performance

int traits::run( const comma::command_line_options& options )
{
    comma::csv::options csv( options, "x,y,z" );
    csv.full_xpath = false;
    if( csv.has_field( "block" ) ) { COMMA_THROW( comma::exception, "block field support: todo" ); }
    comma::csv::input_stream< input > istream( std::cin, csv );
    double radius = options.value< double >( "--radius" );
    std::deque< record > records;
    typedef snark::voxel_map< voxel_t, 3 > voxels_t;
    voxels_t voxels( Eigen::Vector3d( radius, radius, radius ) );
    std::deque< input > seeds; // todo
    unsigned int seed_index = 0;
    std::deque< input* > queue; // todo
    bool match_id = csv.has_field( "id" );
    comma::uint32 unvisited_id = options.value( "--unvisited-id,--unvisited", 0 );
    comma::uint32 visited_id = options.value( "--visited-id,--visited", 1 ); // todo: set as default in seed feed
    auto output_block = [&]()
    {
        for( const auto& r: records )
        {
            std::cout.write( &r.buffer[0], r.buffer.size() );
            if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &r.visited_id ), 4 ); }
            else { std::cout << csv.delimiter << r.visited_id << std::endl; }
        }
        if( csv.flush ) { std::cout.flush(); }
    };
    auto visit_record = [&]( record& r, const visit::input& i )
    {
        if( r.visited_id != unvisited_id ) { return; }
        if( match_id && r.input.id != i.id ) { return; }            
            
        // todo: trajectory direction
        // todo: trajectory fov
        
        r.input.id = r.visited_id = i.id;
        queue.push_back( &r.input );
    };
    auto visit_at = [&]( const visit::input& input )
    {
        auto it = voxels.touch_at( input.point ); // quick and dirty
        voxels_t::index_type i;
        for( i[0] = it->first[0] - 1; i[0] < it->first[0] + 2; ++i[0] )
        {
            for( i[1] = it->first[1] - 1; i[1] < it->first[1] + 2; ++i[1] )
            {
                for( i[2] = it->first[2] - 1; i[2] < it->first[2] + 2; ++i[2] )
                {
                    auto git = voxels.find( i );
                    if( git != voxels.end() ) { for( auto& r: git->second ) { visit_record( *r, input ); } }
                }
            }
        }
    };
    auto handle_block = [&]()
    {
        for( seed_index = 0; seed_index < seeds.size(); ++seed_index )
        {
            queue.push_back( &seeds[ seed_index ] );
            while( !queue.empty() )
            {
                visit_at( *queue.front() );
                queue.pop_front();
            }
        }
        output_block();
        voxels.clear();
        records.clear();
        seeds.clear();
    };
    while( istream.ready() || std::cin.good() )
    {
        const input* p = istream.read();
        if( !p || ( !records.empty() && records[0].input.block != p->block ) ) { handle_block(); }
        if( !p ) { break; }
        records.push_back( record( *p, istream.last(), unvisited_id ) );
        auto voxel = voxels.touch_at( records.back().input.point );
        voxel->second.push_back( &records.back() );
    }
    return 0;
}

} } } // namespace snark { namespace points_calc { namespace visit {
