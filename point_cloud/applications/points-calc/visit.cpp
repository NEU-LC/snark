// Copyright (c) 2020 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <cmath>
#include <deque>
#include <fstream>
#include <sstream>
#include <tuple>
#include <Eigen/Core>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "../../../point_cloud/voxel_map.h"
#include "../../../visiting/eigen.h"
#include "visit.h"

namespace snark { namespace points_calc { namespace visit {
 
std::string traits::usage()
{
    std::ostringstream oss;
    oss
        << "    visit: todo...\n"
        << "        default input fields: \"x,y,z\"\n"
        << "        options\n"
        << "            --at=<filename>[;<csv-options>]; seed points from which visiting starts\n"
        << "            --radius=[<meters>]; how far to visit\n"
        << "            --field-of-view,--fov=<radians>; default=3.14159265359; at which fields of view to visit\n"
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
    Eigen::Vector3d normal;
    comma::uint32 block;
    comma::uint32 id;
    input( const Eigen::Vector3d& point = Eigen::Vector3d( 0, 0, 0 ), const Eigen::Vector3d& normal = Eigen::Vector3d( 0, 0, 0 ), comma::uint32 block = 0, comma::uint32 id = 0 ): point( point ), normal( normal ), block( block ), id( id ) {}
    input( comma::uint32 id ): point( 0, 0, 0 ), normal( 0, 0, 0 ), block( 0 ), id( id ) {}
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
        v.apply( "normal", p.normal );
        v.apply( "block", p.block );
        v.apply( "id", p.id );
    }

    template < typename K, typename V > static void visit( const K&, const snark::points_calc::visit::input& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
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
    comma::uint32 unvisited_id = options.value( "--unvisited-id,--unvisited", 0 );
    comma::uint32 visited_id = options.value( "--visited-id,--visited", 1 );
    auto handle_fields = [&]( const std::string& fields ) -> std::pair< std::string, bool >
    {
        if( fields.empty() ) { return std::make_pair( std::string( "point/x,point/y,point/z" ), false ); }
        auto v = comma::split( fields, ',' );
        bool has_normal = false;
        for( auto& f: v )
        {
            if( f == "normal" || f == "normal/x" || f == "normal/y" || f == "normal/z" ) { has_normal = true; }
            else if( f == "x" || f == "y" || f == "z" ) { f = "point/" + f; }
        }
        return std::make_pair( comma::join( v, ',' ), has_normal );
    };
    comma::csv::options csv( options );
    bool stdin_has_normal = false;
    std::tie( csv.fields, stdin_has_normal ) = handle_fields( csv.fields );
    comma::csv::input_stream< input > istream( std::cin, csv, input( unvisited_id ) );
    double radius = options.value< double >( "--radius" );
    std::deque< record > records;
    typedef snark::voxel_map< voxel_t, 3 > voxels_t;
    voxels_t voxels( Eigen::Vector3d( radius, radius, radius ) );
    std::deque< const input* > queue; // todo
    bool match_id = csv.has_field( "id" );
    std::string at_options = options.value< std::string >( "--at" );
    comma::csv::options at_csv = comma::name_value::parser( "filename", ';', '=', false ).get< comma::csv::options >( at_options );
    bool at_has_normal = false;
    std::tie( at_csv.fields, stdin_has_normal ) = handle_fields( at_csv.fields );
    comma::io::istream is( at_csv.filename, at_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
    comma::csv::input_stream< input > at_stream( *is, at_csv, input( visited_id ) );
    double fov = options.value( "--field-of-view,--fov", M_PI );
    input origin;
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
        //if( i.point -  )
        
        
        // todo: fov, radius
        
        
        
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
        static boost::optional< input > last;
        auto visit_all_at = [&]( const visit::input& input )
        {
            visit_at( input );
            while( !queue.empty() )
            {
                visit_at( *queue.front() );
                queue.pop_front();
            }
        };
        if( last ) { visit_all_at( *last ); }
        while( at_stream.ready() || is->good() )
        {
            const input* p = at_stream.read();
            if( !p ) { break; }
            if( p->block != records[0].input.block ) { last = *p; break; } // todo: matching block management
            visit_all_at( *p );
        }
        output_block();
        voxels.clear();
        records.clear();
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
