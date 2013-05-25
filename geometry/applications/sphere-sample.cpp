#include <fstream>
#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/parser.h>
#include "aero/geometry/sample.h"
#include "aero/geometry/circle.h"
#include "aero/geometry/ellipse.h"
#include "aero/geometry/polygon.h"
#include "aero/geometry/region.h"
#include "aero/geometry/traits.h"

#include <snark/math/range_bearing_elevation.h>
#include <Eigen/Geometry>

static void usage( bool more = false )
{
    std::cerr << std::endl;
    std::cerr << "make filter or sample of points in a region on a sphere" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: sphere-sample <options> [<output csv options>] > sample.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --more-help,--long-help: more help output" << std::endl;
    std::cerr << "    --region=<what><region options>" << std::endl;
    std::cerr << "        <what>" << std::endl;
    std::cerr << "            circle;<centre lat>,<centre long>,<radius>" << std::endl;
    std::cerr << "            ellipse;<focus1 lat>,<focus1 long>,<focus2 lat>,<focus2 long>,<major axis>" << std::endl;
    std::cerr << "            polygon;<point inside: lat>,<point inside: long>;<filename>[;<csv options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "sampling options" << std::endl;
    std::cerr << "    if no options given, take latitude,longitude points on stdin" << std::endl;
    std::cerr << "     and output them to stdout, if they are inside of the region" << std::endl;
    std::cerr << "    --resolution,--radius,-r=<degrees>: sample will be taken on grid of given resolution" << std::endl;
    std::cerr << "                                        which gives slightly more uniform distribution" << std::endl;
    std::cerr << "    --size,-s: if no --resolution given, sample size (number of random)" << std::endl;
    std::cerr << "               if --resolution given, sample size per each cell; default: 1" << std::endl;
    std::cerr << std::endl;
    if( more )
    {
        std::cerr << std::endl;
        std::cerr << "csv options" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    10000 random points in given polygon and a point inside with lat/lon 5,5 in degrees, e.g:" << std::endl;
    std::cerr << "    cat polygon.csv" << std::endl;
    std::cerr << "    0,0" << std::endl;
    std::cerr << "    0,10" << std::endl;
    std::cerr << "    10,10" << std::endl;
    std::cerr << "    20,0" << std::endl;
    std::cerr << "    sphere-sample --region=\"polygon;5,5;polygon.csv\" --size=10000" << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

using namespace acfr;

static aero::region make_regions_( const std::vector< std::string >& regions )
{
    aero::region r;
    if( regions.size() > 1 ) { COMMA_THROW( comma::exception, "sampling multiple regions: todo" ); }
    for( std::size_t i = 0; i < regions.size(); ++i )
    {
        const std::vector< std::string > v = comma::split( regions[i], ';' );
        if( v[0] == "circle" )
        {
            if( v.size() == 1 ) { COMMA_THROW( comma::exception, "expected circle parameters" ); }
            r = aero::region( comma::csv::ascii< aero::circle >( "", ',', true ).get( v[1] ) );
        }
        else if( v[0] == "ellipse" )
        {
            if( v.size() == 1 ) { COMMA_THROW( comma::exception, "expected ellipse parameters" ); }
            aero::ellipse e = comma::csv::ascii< aero::ellipse >( "", ',', true ).get( v[1] );
            if( !e.valid() ) { COMMA_THROW( comma::exception, "invalid ellipse: " << v[1] ); }
            r = aero::region( e );
        }
        else if( v[0] == "polygon" )
        {
            if( v.size() < 3 ) { COMMA_THROW( comma::exception, "expected polygon filename" ); }
            comma::csv::options csv = comma::name_value::parser( "shape,point-inside,filename", ';' ).get< comma::csv::options >( regions[i] );
            aero::coordinates point_inside = comma::csv::ascii< aero::coordinates >().get( v[1] );
            std::ifstream ifs( csv.filename.c_str() );
            if( !ifs.is_open() ) { COMMA_THROW( comma::exception, "polygon: failed to open file \"" << v[1] << "\"" ); }
            comma::csv::input_stream< aero::coordinates > istream( ifs, csv );
            std::vector< aero::coordinates > corners;
            while( istream.ready() || ( !ifs.eof() && ifs.good() ) )
            {
                const aero::coordinates* c = istream.read();
                if( !c ) { break; }
                corners.push_back( *c );
            }
            ifs.close();
            aero::polygon p( corners, point_inside );
            r = aero::region( p );
        }
        else
        {
            COMMA_THROW( comma::exception, "expected region, got \"" << v[0] << "\"" );
        }
    }
    return r;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        if( options.exists( "--more-help,--long-help" ) ) { usage( true ); }
        const std::vector< std::string >& regions = options.values< std::string >( "--region,-r" );
        const aero::region& r = make_regions_( regions );
        comma::csv::options csv( options );
        comma::csv::output_stream< aero::coordinates > ostream( std::cout, csv );
        comma::signal_flag is_shutdown;
        if( options.exists( "--resolution,--radius,-r" ) )
        {
            double radius = options.value< double >( "--resolution,--radius,-r" ) * M_PI / 180;
            double half_radius = radius / 2;
            std::size_t size = options.value< std::size_t >( "--size,-s", 1 );
            aero::coordinates from( r.containing_circle().centre.latitude - r.containing_circle().radius + half_radius
                                  , r.containing_circle().centre.longitude - r.containing_circle().radius + half_radius );
            aero::coordinates to( r.containing_circle().centre.latitude + r.containing_circle().radius - half_radius
                                , r.containing_circle().centre.longitude + r.containing_circle().radius - half_radius );
            for( double e = from.latitude; !is_shutdown && e < to.latitude; e += radius )
            {
                for( double b = from.longitude; !is_shutdown && b < to.longitude; b += radius )
                {
                    for( std::size_t i = 0; !is_shutdown && i < size; ++i )
                    {
                        const aero::coordinates& c = aero::pretty_uniform_sample( aero::coordinates( e, b ), half_radius );
                        if( r.includes( c ) ) { ostream.write( c ); }
                    }
                }
            }
        }
        else if( options.exists( "--size,-s" ) )
        {
            std::size_t size = options.value< std::size_t >( "--size,-s" );
            for( std::size_t i = 0; !is_shutdown && i < size; ++i )
            {
                static const unsigned int max_attempts = 10000;
                unsigned int attempts = max_attempts;
                while( attempts-- )
                {
                    const aero::coordinates& c = aero::pretty_uniform_sample( r.containing_circle() );
                    if( r.includes( c ) ) { ostream.write( c ); break; }
                }
                if( !attempts ) { std::cerr << "sphere-sample: failed to get random sample after " << max_attempts << std::endl; return 1; }
            }
        }
        else
        {
            comma::csv::input_stream< aero::coordinates > istream( std::cin, csv );
            while( !is_shutdown && ( istream.ready() || ( std::cin.good() && !std::cin.eof() ) ) )
            {
                const aero::coordinates* c = istream.read();
                if( !c ) { break; }
                if( !r.includes( *c ) ) { continue; }
                if( csv.binary() ) { std::cout.write( istream.binary().last(), istream.binary().binary().format().size() ); }
                else { std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << std::endl; }
            }
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "sphere-sample: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "sphere-sample: unknown exception" << std::endl;
    }
}
