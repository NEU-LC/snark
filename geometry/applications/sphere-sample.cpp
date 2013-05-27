#include <iostream>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <snark/math/range_bearing_elevation.h>
#include "aero/geometry/sample.h"
#include "aero/geometry/traits.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "generate a sample on the whole sphere" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: sphere-sample <options> > sample.csv"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --random: random uniform sample" << std::endl;
    std::cerr << "    --regular: regular grid sample" << std::endl;
    std::cerr << "    --resolution,-r=<value>: regular sample resolution in degrees" << std::endl;
    if( verbose )
    {
        std::cerr << std::endl;
        std::cerr << "csv options" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    std::cerr << std::endl;
    exit( 1 );
}

using namespace acfr;

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--help,-h" ) ) { usage( verbose ); }
        comma::signal_flag is_shutdown;
        double resolution = options.value< double >( "--resolution,-r" ) * M_PI / 180;
        comma::csv::output_stream< aero::coordinates > ostream( std::cout, comma::csv::options( options ) );
        options.assert_mutually_exclusive( "--random,--regular" );
        bool regular = options.exists( "--regular" );
        bool random = options.exists( "--random" );
        if( !regular && !random ) { std::cerr << "sphere-sample: expected sample type (--random or --regular)" << std::endl; return 1; }
        if( regular )
        {
            aero::coordinates from( -M_PI / 2, -M_PI );
            aero::coordinates to( M_PI / 2, M_PI );
            for( aero::coordinates c( from ); c.latitude < to.latitude; c.latitude += resolution )
            {
                for( c.longitude = from.longitude; c.longitude < to.longitude; c.longitude += resolution )
                {
                    ostream.write( c );
                }
            }
        }
        if( random )
        {
            boost::mt19937 generator;
            boost::uniform_real< double > distribution( 0, 1 );
            boost::variate_generator< boost::mt19937&, boost::uniform_real< double > > r( generator, distribution );
            aero::coordinates from( -M_PI / 2, -M_PI );
            aero::coordinates to( M_PI / 2, M_PI );
            for( aero::coordinates c( from ); !is_shutdown && c.latitude < to.latitude; c.latitude += resolution )
            {
                double step = resolution / std::abs( std::cos( c.latitude ) );
                std::size_t size = M_PI * 2 / step;
                for( std::size_t i = 0; i < size; ++i )
                {
                    ostream.write( aero::pretty_uniform_sample( aero::coordinates( c.latitude, ( r() * 2 - 1 ) * M_PI ), resolution ) );
                }
            }


//             for( std::size_t i = 0; i < size; ++i )
//             {
//                 double z = r() * 2 - 1;
//                 double s = density * std::sqrt( 1 - z * z );
//                 double n = M_PI * 2.0 / s;
//                 double q =
//                 double a = ( r() / s * 2 - 1 ) * M_PI;
//                 snark::range_bearing_elevation rbe( Eigen::Vector3d( std::cos( a ), std::sin( a ), z ) );
//                 ostream.write( aero::coordinates( rbe.e(), rbe.b() ) );
//                 //snark::range_bearing_elevation rbe( Eigen::Vector3d( r() * 2 - 1, r() * 2 - 1, r() * 2 - 1 ) );
//                 //ostream.write( aero::coordinates( rbe.e(), rbe.b() ) );
//             }
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
