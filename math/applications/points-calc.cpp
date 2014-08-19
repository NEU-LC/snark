#include <iostream>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <snark/visiting/eigen.h>
#include <math.h>
#include <comma/math/compare.h>

typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > point_pair_t;

static comma::csv::options csv;
static comma::csv::ascii< Eigen::Vector3d > ascii;

static void usage( bool more = false )
{
    std::cerr << std::endl;
    std::cerr << "take coordinates from stdin, perform calculations, and output result to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    cat points.csv | points-calc distance > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc cumulative-distance > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc thin --resolution <resolution> > results.csv" << std::endl;
    std::cerr << "    cat points.csv | points-calc discretise --step <step> > results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations: distance, cumulative-distance, thin, discretise" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance: distance between subsequent points or, if input is pairs, between the points of the same record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cumulative-distance: cumulative distance between subsequent points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    thin: read input data and thin them down by the given --resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    discretise: read input data and discretise intervals between adjacent points with --step" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

static void calculate_distance( bool cumulative )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    double distance = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        double norm = last ? ( *p - *last ).norm() : 0;
        distance = cumulative ? distance + norm : norm;
        last = *p;
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &distance ), sizeof( double ) );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << distance << std::endl;
        }
    }
}

static void calculate_distance_for_pairs()
{
    comma::csv::input_stream< point_pair_t > istream( std::cin, csv );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const point_pair_t* p = istream.read();
        if( !p ) { break; }
        double norm = ( p->first - p->second ).norm();
        if( csv.binary() )
        {
            std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            std::cout.write( reinterpret_cast< const char* >( &norm ), sizeof( double ) );
        }
        else
        {
            std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << norm << std::endl;
        }
    }
}

static void thin( double resolution )
{
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > last;
    double distance = 0;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* p = istream.read();
        if( !p ) { break; }
        distance += last ? ( *p - *last ).norm() : 0;
        if( !last || distance >= resolution )
        {
            distance = 0;
            if( csv.binary() )
            {
                std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
            }
            else
            {
                std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << std::endl;
            }
        }
        last = *p;
    }
}

void output_points(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    if( csv.binary() )
    {
        std::cout.write( reinterpret_cast< const char* >( &p1 ), sizeof( double ) * 3 );
        std::cout.write( reinterpret_cast< const char* >( &p2 ), sizeof( double ) * 3 );
        std::cout.flush();
    }
    else 
    {
        std::cout << ascii.put( p1 ) << csv.delimiter << ascii.put( p2 ) << std::endl; 
    }
}

static void discretise( double step )
{
    BOOST_STATIC_ASSERT( sizeof( Eigen::Vector3d ) == sizeof( double ) * 3 );
    comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
    boost::optional< Eigen::Vector3d > previous_point;
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const Eigen::Vector3d* current_point = istream.read();
        if( !current_point ) { break; }
        if( previous_point )
        {
            output_points( *previous_point, *previous_point );
            double distance = ( *previous_point - *current_point ).norm();
            if( comma::math::less( step, distance ) )
            {
                Eigen::ParametrizedLine< double, 3 > line = Eigen::ParametrizedLine< double, 3 >::Through( *previous_point, *current_point );
                for( double t = step; comma::math::less( t, distance ); t += step )
                {
                    Eigen::Vector3d point = line.pointAt( t );
                    output_points( *previous_point, point );
                }
            }
        }
        previous_point.reset( *current_point );
    }
    output_points( *previous_point, *previous_point );
}


int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--help,-h" ) ) { usage( verbose ); }
        csv = comma::csv::options( options );
        csv.full_xpath = true;
        ascii = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", csv.delimiter );
        const std::vector< std::string >& operations = options.unnamed( "--verbose,-v", "-.*" );
        if( operations.size() != 1 ) { std::cerr << "points-calc: expected one operation, got " << operations.size() << std::endl; return 1; }
        const std::string& operation = operations[0];
        if( operation == "distance" )
        {
            if(    csv.has_field( "first" )   || csv.has_field( "second" )
                || csv.has_field( "first/x" ) || csv.has_field( "second/x" )
                || csv.has_field( "first/y" ) || csv.has_field( "second/y" )
                || csv.has_field( "first/z" ) || csv.has_field( "second/z" ) )
            {
                calculate_distance_for_pairs();
                return 0;
            }
            calculate_distance( false );
            return 0;
        }
        if( operation == "cumulative-distance" )
        {
            calculate_distance( true );
            return 0;
        }
        if( operation == "thin" )
        {
            if( !options.exists( "--resolution" ) ) { std::cerr << "points-calc: --resolution is not specified " << std::endl; return 1; }
            double resolution = options.value( "--resolution" , 0.0 );
            thin( resolution );
            return 0;
        }
        if( operation == "discretise" )
        {
            if( !options.exists( "--step" ) ) { std::cerr << "points-calc: --step is not specified " << std::endl; return 1; }
            double step = options.value( "--step" , 0.0 );
            if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
            discretise( step );
            return 0;
        }
        std::cerr << "points-calc: please specify an operation" << std::endl;
        return 1;
    }
    catch( std::exception& ex )
    {
        std::cerr << "points-calc: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "points-calc: unknown exception" << std::endl;
    }
    return 1;
}
