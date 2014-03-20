#include <iostream>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <snark/visiting/eigen.h>

typedef std::pair< Eigen::Vector3d, Eigen::Vector3d > pair_t;

static void usage( bool more = false )
{
    std::cerr << "todo" << std::endl;
    exit( 1 );
    std::cerr << std::endl;
    std::cerr << "take coordinates in degrees on sphere from stdin, perform calculations, append result and output to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    cat points.csv | sphere-arc distance > results.csv" << std::endl;
    std::cerr << "    cat pairs.csv | sphere-arc distance > results.csv" << std::endl;
    std::cerr << "    cat points.csv | sphere-arc cumulative-distance > results.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations: distance, cumulative-distance" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance: distance between subsequent points or, if input is pairs, between the points of the same record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "            " << comma::join( comma::csv::names< pair_t >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cumulative-distance: cumulative distance between subsequent points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

static void calculate_distance( const comma::csv::options& csv, bool cumulative )
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

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--help,-h" ) ) { usage( verbose ); }
        comma::csv::options csv( options );
        csv.full_xpath = true;
        const std::vector< std::string >& operations = options.unnamed( "--verbose,-v,--degrees", "-.*" );
        if( operations.size() != 1 ) { std::cerr << "points-calc: expected one operation, got " << operations.size() << std::endl; return 1; }
        const std::string& operation = operations[0];
        if( operation == "distance" )
        {
            if(    csv.has_field( "first" )   || csv.has_field( "second" )
                || csv.has_field( "first/x" ) || csv.has_field( "second/x" )
                || csv.has_field( "first/y" ) || csv.has_field( "second/y" )
                || csv.has_field( "first/z" ) || csv.has_field( "second/z" ) )
            {
                comma::csv::input_stream< pair_t > istream( std::cin, csv );
                while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
                {
                    const pair_t* p = istream.read();
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
                return 0;
            }
            calculate_distance( csv, false );
            return 0;
        }
        if( operation == "cumulative-distance" )
        {
            calculate_distance( csv, true );
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
