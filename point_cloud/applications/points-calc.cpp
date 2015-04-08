#include <cmath>
#include <deque>
#include <iostream>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/math/compare.h>
#include <snark/point_cloud/voxel_grid.h> // real quick and dirty
#include <snark/visiting/eigen.h>

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
    std::cerr << "operations: distance, cumulative-distance, thin, discretise, nearest, local-min, local-max" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance: distance between subsequent points or, if input is pairs, between the points of the same record" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << "                      " << comma::join( comma::csv::names< point_pair_t >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    cumulative-distance: cumulative distance between subsequent points" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    local-min: output local minimums inside of given radius" << std::endl;
    std::cerr << "    local-max: output local maximums inside of given radius" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: x,y,z,scalar" << std::endl;
    std::cerr << "        example: get local height maxima in the radius of 5 metres:" << std::endl;
    std::cerr << "            cat xyz.csv | points-calc local-max --fields=x,y,scalar --radius=5" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    nearest: find point nearest to the given point" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "        options:" << std::endl;
    std::cerr << "            --point,--to=<x>,<y>,<z>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    thin: read input data and thin them down by the given --resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "        input fields" << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "    discretise, discretize: read input data and discretise intervals between adjacent points with --step" << std::endl;
    std::cerr << "        skip discretised points that are closer to the end of the interval than --tolerance (default: --tolerance=0)" << std::endl;
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

static void discretise( double step, double tolerance )
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
                for( double t = step; comma::math::less( t + tolerance, distance ); t += step )
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

namespace local_operation {

struct point
{
    Eigen::Vector3d coordinates;
    double scalar;
    
    point() : coordinates( 0, 0, 0 ), scalar( 0 ) {}
    point( const Eigen::Vector3d& coordinates, double scalar ) : coordinates( coordinates ), scalar( scalar ) {}
};

struct record
{
    local_operation::point point;
    std::string line;
    bool rejected;
    
    record() : rejected( false ) {}
    record( const local_operation::point& p, const std::string& line ) : point( p ), line( line ), rejected( false ) {}
};

static void evaluate_local_extremum( record* i, record* j, double radius, double sign )
{
    if( i == j || i->rejected ) { return; }
    if( ( i->point.coordinates - j->point.coordinates ).squaredNorm() > radius * radius ) { return; }
    i->rejected = comma::math::less( ( i->point.scalar - j->point.scalar ) * sign, 0 );
    if( !i->rejected ) { j->rejected = true; }
}

} // namespace local_operation {

namespace comma { namespace visiting {

template <> struct traits< local_operation::point >
{
    template< typename K, typename V > static void visit( const K&, const local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
    }
    
    template< typename K, typename V > static void visit( const K&, local_operation::point& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "scalar", t.scalar );
    }
};

} } // namespace comma { namespace visiting {

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
        if( operation == "nearest" )
        {
            Eigen::Vector3d point = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--point,--to" ) );
            comma::csv::input_stream< Eigen::Vector3d > istream( std::cin, csv );
            std::string record;
            double min_distance = std::numeric_limits< double >::max();
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                double d = ( *p - point ).norm();
                if( d >= min_distance ) { continue; }
                min_distance = d;
                record = csv.binary() ? std::string( istream.binary().last(), csv.format().size() ) : comma::join( istream.ascii().last(), csv.delimiter );
            }
            if( !record.empty() ) { std::cout << record; }
            if( csv.binary() ) { std::cout.write( reinterpret_cast< const char* >( &min_distance ), sizeof( double ) ); }
            else { std::cout << csv.delimiter << min_distance << std::endl; }
            return 0;
        }
        if( operation == "thin" )
        {
            if( !options.exists( "--resolution" ) ) { std::cerr << "points-calc: --resolution is not specified " << std::endl; return 1; }
            double resolution = options.value( "--resolution" , 0.0 );
            thin( resolution );
            return 0;
        }
        if( operation == "discretise" || operation == "discretize" )
        {
            if( !options.exists( "--step" ) ) { std::cerr << "points-calc: --step is not specified " << std::endl; return 1; }
            double step = options.value( "--step" , 0.0 );
            if( step <= 0 ) { std::cerr << "points-calc: expected positive step, got " << step << std::endl; return 1; }
            // the last discretised point can be very close to the end of the interval, in which case the last two points can be identical in the output since ascii.put uses 12 digits by default
            // setting --tolerance=1e-12 will not allow the last discretised point to be too close to the end of the interval and therefore the output will have two distinct points at the end
            double tolerance = options.value( "--tolerance" , 0.0 ); 
            if( tolerance < 0 ) { std::cerr << "points-calc: expected non-negative tolerance, got " << tolerance << std::endl; return 1; }
            discretise( step, tolerance );
            return 0;
        }
        if( operation == "local-max" || operation == "local-min" ) // todo: if( operation == "local-calc" ? )
        {
            double sign = operation == "local-max" ? 1 : -1;
            if( csv.fields.empty() ) { csv.fields = "x,y,z,scalar"; } // todo: better use x,y,scalar as a point
            csv.full_xpath = false;
            comma::csv::input_stream< local_operation::point > istream( std::cin, csv );
            std::deque< local_operation::record > records;
            double radius = options.value< double >( "--radius" );
            Eigen::Vector3d resolution( radius, radius, radius );
            snark::math::closed_interval< double, 3 > extents;
            while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const local_operation::point* p = istream.read();
                if( !p ) { break; }
                std::string line;
                if( csv.binary() ) // quick and dirty
                {
                    line.resize( csv.format().size() );
                    ::memcpy( &line[0], istream.binary().last(), csv.format().size() );
                }
                else
                {
                    line = comma::join( istream.ascii().last(), csv.delimiter );
                }
                records.push_back( local_operation::record( *p, line ) );
                extents.set_hull( p->coordinates );
            }
            typedef std::vector< local_operation::record* > voxel_t; // todo: is vector a good container? use deque
            typedef snark::voxel_grid< voxel_t > grid_t;
            grid_t grid( extents, resolution );
            for( std::size_t i = 0; i < records.size(); ++i ) { ( grid.touch_at( records[i].point.coordinates ) )->push_back( &records[i] ); }
            for( grid_t::iterator it = grid.begin(); it != grid.end(); ++it )
            {
                for( voxel_t::iterator vit = it->begin(); vit != it->end(); ++vit )
                {
                    for( voxel_t::iterator wit = it->begin(); !( *vit )->rejected && wit != it->end(); ++wit )
                    {
                        local_operation::evaluate_local_extremum( *vit, *wit, radius, sign );
                    }
                    for( grid_t::neighbourhood_iterator nit = grid_t::neighbourhood_iterator::begin( it ); nit != grid_t::neighbourhood_iterator::end( it ) && !( *vit )->rejected; ++nit )
                    {
                        for( voxel_t::iterator wit = nit->begin(); wit != nit->end() && !( *vit )->rejected; ++wit )
                        {
                            local_operation::evaluate_local_extremum( *vit, *wit, radius, sign );
                        }
                    }
                }
            }
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            std::string endl = csv.binary() ? "" : "\n";
            for( std::size_t i = 0; i < records.size(); ++i )
            {
                if( records[i].rejected ) { continue; }
                std::cout.write( &records[i].line[0], records[i].line.size() );
                std::cout.write( &endl[0], endl.size() );
            }
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
