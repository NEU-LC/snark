#include <iostream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/visiting/traits.h>
#include <snark/math/range_bearing_elevation.h>
#include <ctime>
#include "../sample.h"
#include "../coordinates.h"
#include "../traits.h"

static std::string app_name = "";

static void usage( bool verbose )
{
    std::cerr
        << "Usage: sphere-sample"
        <<      " [-h|--help [-v|--verbose]]"
        <<      " (--random|--regular|--regular-uniform|--uniform)"
        <<      " -r|--resolution=<value>"
        <<      " [-t|--seed-time] "
        <<      " [-s|--seed=<random-seed>]\n"
        <<      " --shift-by=<latitude,longitude>]\n"
        <<      " --latitude-range=<latitude range>]\n"
        <<      " --longitude-range=<longitude range>]\n"
        << "\n"
        << "Generates a sample ever the whole sphere. Option --random or --regular[-uniform] must be specified,"
        << " as well as option -r|--resolution.\n"
        << "\n"
        << "Options:\n"
        << "\n"
        << "    -h|--help                       Show this help (-v|--verbose to show csv options)\n"
        << "    --random                        Random uniform sample\n"
        << "    --regular                       Regular grid sample\n"
        << "    --regular-uniform,--uniform     Regular, but pretty uniform\n"
        << "    --shift-by                      Shifts samples by shift coordinates\n"
        << "    --latitude-range                Min,Max latitude range (default -90 to 90)\n"
        << "    --longitude-range               Min,Max longitude range (default -180 to 180)\n"
        << "    -r|--resolution=<value>         Sample resolution in degrees\n"
        << "    -t|--seed-time                  Use current time as randum number seed\n"
        << "    -s|--seed=<random-seed>         Specify randum number seed (an integer)\n";

    if( verbose )
    {
        std::cerr << "\nCsv options:\n\n" << comma::csv::options::usage() << std::endl;
    }

    std::cerr << std::endl;
    exit( 1 );
}

template< typename T >
T get_option(comma::command_line_options &options, const std::string option_name, const T &default_val)
{
    if (!options.exists( option_name ))
    { return default_val; }
    
    T result = default_val;
    try { result = comma::csv::ascii< T >().get( options.value< std::string >( option_name ) ); }
    catch ( ... )
    {
        std::cerr << app_name << ": Error: invalid value for " << option_name << "\n";
        exit( 1 );
    }
    return result;
}

using namespace acfr;

int main( int ac, char** av )
{
    app_name = av[0];
    try
    {
        if (ac <= 1) { usage(false); }
        
        comma::command_line_options options( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--help,-h" ) ) { usage( verbose ); }

        double resolution = options.value< double >( "--resolution,-r" ) * M_PI / 180;
        comma::csv::output_stream< aero::coordinates > ostream( std::cout, comma::csv::options( options ) );
        options.assert_mutually_exclusive( "--random,--regular" );
        bool regular = options.exists( "--regular" );
        bool random = options.exists( "--random" );
        bool regular_uniform = options.exists( "--regular-uniform,--uniform" );
        if( !regular && !random && !regular_uniform ) { std::cerr << "sphere-sample: expected sample type (--random or --regular)" << std::endl; return 1; }
        
        auto shift = get_option<aero::coordinates>(options, "--shift-by", aero::coordinates(0,0) );
        auto latitude_range = get_option< std::pair<double, double> >(options, "--latitude-range", std::pair<double, double>(-90, 90) );
        auto longitude_range = get_option< std::pair<double, double> >(options, "--longitude-range", std::pair<double, double>(-180, 180) );
        
        double rad_adj = M_PI/180;
        aero::coordinates from( latitude_range.first*rad_adj, longitude_range.first*rad_adj );
        aero::coordinates to( latitude_range.second*rad_adj, longitude_range.second*rad_adj );
        
        if( regular )
        {
            for( aero::coordinates c( from ); c.latitude < to.latitude; c.latitude += resolution )
            {
                for( c.longitude = from.longitude; c.longitude < to.longitude; c.longitude += resolution )
                {
                    ostream.write( c + shift );
                }
            }
        }
        else if( random || regular_uniform )
        {
            boost::mt19937 generator;
            boost::optional< unsigned long > seed;

            if( options.exists( "--seed,-s" ) )
            {
                try { seed = options.value< unsigned long >( "--seed,-s" ); }
                catch ( ... )
                {
                    std::cerr << av[0] << ": expected integer value for --seed\n";
                    exit( 1 );
                }
            }
            else
            {
                if ( options.exists( "--seed-time,-t" ) ) { seed = (unsigned long) std::time( 0 ); }
            }

            if ( seed ) { generator.seed( *seed ); }
            boost::uniform_real< double > distribution( 0, 1 );
            boost::variate_generator< boost::mt19937&, boost::uniform_real< double > > r( generator, distribution );

            for( aero::coordinates c( from ); c.latitude < to.latitude; c.latitude += resolution )
            {
                double radius = std::abs( std::cos( c.latitude ) );
                if( radius == 0 ) { continue; }
                double step = resolution / radius;
                double offset = random ? ( r() * 2 - 1 ) * step : 0.0;
                for( c.longitude = from.longitude + offset; c.longitude < to.longitude + offset; c.longitude += step )
                {
                    ostream.write( regular_uniform ? c+shift : aero::pretty_uniform_sample( c, resolution / 2 )+shift );
                }
            }
        }
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
