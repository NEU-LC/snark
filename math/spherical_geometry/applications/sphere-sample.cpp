// Copyright (c) 2013-2016. This code was produced by the
// Australian Centre for Field Robotics, The University of Sydney under
// the Future Flight Planning project, University Reference 13996, contract
// NSW-CPS-2011-015264, Work Orders 5, 7 and 8. The intellectual property
// ownership is as set out in these contracts, as registered with
// Commercial Development and Industry Partnerships.

#include <iostream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>
#include "../../../math/range_bearing_elevation.h"
#include "../coordinates.h"
#include "../sample.h"
#include "../traits.h"

static const std::string app_name = "sphere-calc";

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "generates a sample ever the whole sphere" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: sphere-calc <options> > sample.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    -h|--help                       Show this help (-v|--verbose to show csv options)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "sample type options" << std::endl;
    std::cerr << "    --random                        Random uniform sample" << std::endl;
    std::cerr << "    --regular                       Regular grid sample" << std::endl;
    std::cerr << "    --regular-uniform,--uniform     Regular, but pretty uniform" << std::endl;
    std::cerr << std::endl;
    std::cerr << "random seed options" << std::endl;
    std::cerr << "    -s|--seed=<random-seed>         seed for the random sample" << std::endl;
    std::cerr << "    -t|--seed-time                  use current time as random number seed" << std::endl;
    std::cerr << std::endl;
    std::cerr << "sample options" << std::endl;
    std::cerr << "    --begin,--from=<latitude,longitude>; default: -90, -180" << std::endl;
    std::cerr << "    --end,--to=<latitude,longitude>; end not included, default: 90.00000001, 180" << std::endl;
    std::cerr << "    --resolution,-r=<value>         Sample resolution in degrees" << std::endl;
    if( verbose ) { std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    exit( 0 );
}

template< typename T > T get_option( comma::command_line_options &options, const std::string option_name, const T &default_val )
{
    if( !options.exists( option_name ) ) { return default_val; }
    T result = default_val;
    result = comma::csv::ascii< T >().get( options.value< std::string >( option_name ) );
    return result;
}

using namespace snark;

int main( int ac, char** av )
{
    try
    {
        using snark::spherical::coordinates;
        
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--verbose,-v" ) ) { std::cerr << "sphere-calc: called as: " << options.string() << std::endl; }
        double resolution = options.value< double >( "--resolution,-r" ) * M_PI / 180;
        comma::csv::output_stream< coordinates > ostream( std::cout, comma::csv::options( options ) );
        options.assert_mutually_exclusive( "--random,--regular" );
        bool regular = options.exists( "--regular" );
        bool random = options.exists( "--random" );
        bool regular_uniform = options.exists( "--regular-uniform,--uniform" );
        if( !regular && !random && !regular_uniform ) { std::cerr << "sphere-calc: expected sample type (--random, --regular, or --uniform)" << std::endl; return 1; }
        coordinates begin = get_option< coordinates >( options, "--begin", coordinates( -M_PI / 2, -M_PI ) );
        static const double epsilon = 0.00000001;
        coordinates end = get_option< coordinates >( options, "--end", coordinates( M_PI / 2 + epsilon, M_PI - epsilon ) );
        if( begin.latitude < -M_PI / 2 ) { begin.latitude = -M_PI / 2; }
        if( begin.longitude < -M_PI ) { begin.longitude = -M_PI; }
        if( end.latitude > M_PI / 2 ) { end.latitude = M_PI / 2; }
        if( end.longitude > M_PI ) { end.longitude = M_PI; }
        if( regular )
        {
            for( coordinates c = begin; comma::math::less( c.latitude, end.latitude ); c.latitude += resolution )
            {
                for( c.longitude = begin.longitude; comma::math::less( c.longitude, end.longitude ); c.longitude += resolution )
                {
                    ostream.write( c );
                }
            }
        }
        else if( random || regular_uniform )
        {
            options.assert_mutually_exclusive( "--seed,-s,--seed-time,-t" );
            boost::optional< unsigned long > seed = options.optional< unsigned long >( "--seed,-s" );
            if( options.exists( "--seed-time,-t" ) ) { seed = static_cast< unsigned long >( std::time( 0 ) ); }
            boost::mt19937 generator;
            if( seed ) { generator.seed( *seed ); }
            boost::uniform_real< double > distribution( 0, 1 );
            boost::variate_generator< boost::mt19937&, boost::uniform_real< double > > r( generator, distribution );
            for( coordinates c = begin; comma::math::less( c.latitude, end.latitude ); c.latitude += resolution )
            {
                double radius = std::abs( std::cos( c.latitude ) );
                if( radius == 0 ) { continue; }
                double step = resolution / radius;
                double offset = random ? ( r() * 2 - 1 ) * step : 0.0;
                for( c.longitude = begin.longitude + offset; comma::math::less( c.longitude, end.longitude + offset ); c.longitude += step )
                {
                    ostream.write( regular_uniform ? c : pretty_uniform_sample( c, resolution / 2 ) );
                }
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "sphere-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "sphere-calc: unknown exception" << std::endl; }
    return 1;
}
