#include <deque>
#include <limits>
#include <boost/circular_buffer.hpp>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <snark/visiting/traits.h>

namespace {


void bash_completion( unsigned const ac, char const * const * av )
{
    static char const* arguments[] =
    {
        " turn decelerate",
        " --binary -b --delimiter -d --fields -f --flush --format --full-xpath --help -h --quote --precision --verbose -v",
        " --input-fields --output-fields --output-format",
        " --deceleration -c",
        " --max-acceleration",
        " --approach-speed -s",
        " --sharp-turn-angle -t",
        " --stop-on-sharp-turn --pivot -p"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

void operations( unsigned const indent_count = 0 )
{
    auto const indent = std::string( indent_count, ' ' );
    std::cerr << indent << "decelerate; moderate sudden decreases in speed with given deceleration." << std::endl;
    std::cerr << indent << "turn; set the speed on curves in the plan based on given centripetal acceleration." << std::endl;
}

void usage( bool const verbose )
{
    static const char* const indent="    ";

    std::cerr << std::endl;
    std::cerr << "control the speed of plan." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " <operation> [<options>...]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Operations:" << std::endl;
    operations(4);
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "    common:" << std::endl;
    std::cerr << "        --binary, -b=<fields>; input format." << std::endl;
    std::cerr << "        --fields, -f=<fields>; default=x,y,speed; input fields." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    decelerate:" << std::endl;
    std::cerr << "        --deceleration, -c=<deceleration>; default=0.5; deceleration by which to moderate sudden decreases in speed." << std::endl;
    std::cerr << std::endl;
    std::cerr << "    turn:" << std::endl;
    std::cerr << "        --approach-speed,-a=<speed>; default=0.3; approach with this speed for waypoints with sharp turns." << std::endl;
    std::cerr << "        --max-acceleration, -c=<acceleration>; default=1; maximum centripetal acceleration allowed." << std::endl;
    std::cerr << "        --sharp-turn-angle,-t=<angle>; default=1.178097245; assign 'approach-speed' for waypoints having turn angle greater than this." << std::endl;
    std::cerr << "        --speed,-s=<speed>; use this speed if it is not in input fields." << std::endl;
    std::cerr << "        --stop-on-sharp-turn,--pivot,-p; on sharp turns (i.e. waypoints set with 'approach-speed'), output extra point with relative heading and no speed." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Info:" << std::endl;
    std::cerr << "    --input-fields; print input fields to stdout and exit." << std::endl;
    std::cerr << "    --output-fields; print output fields to stdout and exit." << std::endl;
    std::cerr << "    --output-format; print output fields to stdout and exit." << std::endl;
    if( verbose ) { std::cerr << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    std::cerr << "Examples:" << std::endl;
    std::cerr << "    cat plan.csv |" << std::endl;
    std::cerr << "    control-speed turn --fields=x,y,z,speed --max-acceleration=0.3 --min-speed=0.3 |" << std::endl;
    std::cerr << "    control-speed decelerate --fields=x,y,z,,speed --deceleration=0.2" << std::endl;
    std::cerr << std::endl;
}

namespace generic
{
struct input_t
{
    Eigen::Vector3d coordinates;
    double speed;

    input_t( double sp = 0.0 ) : coordinates( Eigen::Vector3d::Zero() ), speed( sp ) {}
};
}

namespace turn
{
struct output_t
{
    double speed;
    double heading;

    output_t() : speed( 0.0 ), heading( 0.0 ) {}
};

struct record_t
{
    generic::input_t input;
    output_t output;
    std::string line;
    //...debug...
    //double angle;
    //double radius;

    record_t( generic::input_t const& in, comma::csv::input_stream< generic::input_t > const& istrm, comma::csv::options const& csv )
        : input( in )
        , line()
        //...debug...
        //, angle( 0 )
        //, radius( 0 )
    {
        if( csv.binary() )
        {
            line.resize( csv.format().size() );
            ::memcpy( &line[ 0 ], istrm.binary().last(), csv.format().size() );
        }
        else
        {
            line = comma::join( istrm.ascii().last(), csv.delimiter );
        }
    }

    void write( comma::csv::output_stream< output_t >& ostrm, comma::csv::options const& csv )
    {
        ostrm.append( line, output );
        if( csv.binary() && csv.flush ) { std::cout.flush(); }
        //...debug...
        //std::cerr << line << csv.delimiter << speed << csv.delimiter << angle << csv.delimiter << radius << std::endl;
    }
};
}

namespace deceleration
{
struct record_t
{
    generic::input_t input;
    std::string line;
    //...debug...
    //double angle;
    //double radius;

    record_t( generic::input_t const& in, comma::csv::input_stream< generic::input_t > const& istrm, comma::csv::options const& csv )
        : input( in )
        , line()
        //...debug...
        //, angle( 0 )
        //, radius( 0 )
    {
        if( csv.binary() )
        {
            line.resize( csv.format().size() );
            ::memcpy( &line[ 0 ], istrm.binary().last(), csv.format().size() );
        }
        else { line = comma::join( istrm.ascii().last(), csv.delimiter ); }
    }

    void write( comma::csv::output_stream< generic::input_t >& ostrm, comma::csv::options const& csv )
    {
        ostrm.write( input, line );
        if( csv.binary() && csv.flush ) { std::cout.flush(); }
        //...debug...
        //std::cerr << line << csv.delimiter << speed << csv.delimiter << angle << csv.delimiter << radius << std::endl;
    }
};
}

void handle_info_options( comma::command_line_options const& options )
{
    if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< generic::input_t >( false ), ',' ) << std::endl; exit( 0 ); }
}

double radius_calc( Eigen::Vector3d const& p1, Eigen::Vector3d const& p2, Eigen::Vector3d const& p3 )
{
    auto const side1 = ( p2 - p1 ).norm();
    auto const side2 = ( p3 - p2 ).norm();
    auto const side3 = ( p3 - p1 ).norm();
    auto const perimeter_half = ( side1 + side2 + side3 ) / 2;
    auto area = perimeter_half * ( perimeter_half - side1 ) * ( perimeter_half - side2 ) * ( perimeter_half - side3 ); // Heron's Equation

    if( comma::math::less( 0.0, area ) )
    {
        area = std::sqrt( area );
        return ( side1 * side2 * side3 ) / ( 4.0 * area );
    }
    else
    {
        return comma::math::less( side1, side3 ) && comma::math::less( side2, side3 ) ? std::numeric_limits< double >::max() : std::numeric_limits< double >::min();
    }
}

double angle_calc( Eigen::Vector3d const& p1, Eigen::Vector3d const& p2, Eigen::Vector3d const& p3 )
{
    auto angle_axis = Eigen::AngleAxis< double >( Eigen::Quaternion< double >::FromTwoVectors( p2 - p1, p3 - p2 ) );
    //...debug...
    //std::cerr << std::setprecision(16)
    //          << p1.x() << ',' << p1.y() << ',' << p1.z() << ','
    //          << p2.x() << ',' << p2.y() << ',' << p2.z() << ','
    //          << p3.x() << ',' << p3.y() << ',' << p3.z() << ','
    //          << angle_axis.angle() << ',' << angle_axis.axis().x() << ',' << angle_axis.axis().y() << ',' << angle_axis.axis().z() << std::endl;
    return angle_axis.angle() * ( comma::math::less( angle_axis.axis().z(), 0.0 ) ? -1 : 1 );
}

}

namespace comma { namespace visiting {

template <> struct traits< generic::input_t >
{
    template < typename K, typename V > static void visit ( const K&, generic::input_t& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "speed", t.speed );
    }

    template < typename K, typename V > static void visit ( const K&, const generic::input_t& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "speed", t.speed );
    }
};

template <> struct traits< turn::output_t >
{
    template < typename K, typename V > static void visit ( const K&, turn::output_t& t, V& v )
    {
        v.apply( "moderated_speed", t.speed );
        v.apply( "relative_heading", t.heading );
    }

    template < typename K, typename V > static void visit ( const K&, const turn::output_t& t, V& v )
    {
        v.apply( "moderated_speed", t.speed );
        v.apply( "relative_heading", t.heading );
    }
};

}}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        handle_info_options( options );

        std::vector< std::string > operations = options.unnamed( "","--binary,-b,--delimiter,-d,--fields,-f,--flush,--format,--full-xpath,--help,-h,--quote,--precision,--verbose,-v,--input-fields,--output-fields,--output-format,--deceleration,--max-acceleration,-c,--approach-speed,-a,--sharp-turn-angle,-t,--speed,-s,--stop-on-sharp-turn,--pivot,-p" );
        if( operations.size() != 1 ) { std::cerr << comma::verbose.app_name() << ": expected one operation, got " << operations.size() << ": " << comma::join( operations, ' ' ) << std::endl; return 1; }

        comma::csv::options csv( options );
        if( csv.fields.empty() ) { csv.fields = "x,y,speed"; }
        csv.full_xpath = false;

        if( "turn" == operations[ 0 ] )
        {
            if( !( csv.has_field( "speed" ) || options.exists( "--speed,-s" ) ) )
            { std::cerr << comma::verbose.app_name() << ": speed must be present in input fields or command line options." << std::endl; return 1; }

            comma::csv::options out_csv( csv );
            auto const pivot = options.exists( "--stop-on-sharp-turn,--pivot,-p" );
            out_csv.fields = "moderated_speed"; if( pivot ) { out_csv.fields.append( ",relative_heading" ); }
            if( options.exists( "--output-fields" ) ) { std::cout << out_csv.fields << std::endl; return 0; }
            if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< turn::output_t >( out_csv.fields, true ) << std::endl; return 0; }

            auto const speed = options.value< double >( "--speed,-s", 0.0 );
            auto const max_acceleration = options.value< double >( "--max-acceleration,-c", 1 );
            auto const approach_speed = options.value< double >( "--approach-speed,-a", 0.3 );
            auto const sharp_turn_angle = options.value< double >( "--sharp-turn-angle,-t", 3 * M_PI / 8 );
            auto const spot_turn_radius = approach_speed * approach_speed / max_acceleration;
            auto const points_min_separation = 1e-6; // arbitrary

            auto unique_points = 0U;
            std::deque< turn::record_t > que;

            comma::csv::input_stream< generic::input_t > istrm( std::cin, csv, generic::input_t( speed ) );
            comma::csv::output_stream< turn::output_t > ostrm( std::cout, out_csv );


            while( istrm.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                auto input = istrm.read(); if( !input ) break;
                if( que.empty() || ( input->coordinates - que.back().input.coordinates ).norm() >= points_min_separation ) { unique_points++; }
                que.emplace_back( *input, istrm, csv );

                if( 3U == unique_points )
                {
                    auto relevant_index = que.size() - 2U;
                    auto const angle = angle_calc( que.front().input.coordinates, que[ relevant_index ].input.coordinates, que.back().input.coordinates );
                    auto const radius = radius_calc( que.front().input.coordinates, que[ relevant_index ].input.coordinates, que.back().input.coordinates );

                    if( comma::math::less( std::fabs( angle ), sharp_turn_angle ) && comma::math::less( spot_turn_radius, radius ) )
                    {
                        auto updated_speed = std::sqrt( max_acceleration * radius );
                        if( comma::math::less( updated_speed, approach_speed ) ) { std::cerr << "Error: speed calculated is less than min speed." << std::endl; return 1; }
                        que[ relevant_index ].output.speed = std::max( approach_speed, std::min( que[ relevant_index ].input.speed, updated_speed ) );
                    }
                    else
                    {
                        que[ relevant_index ].output.speed = std::min( que[ relevant_index ].input.speed, approach_speed );
                        if( pivot )
                        {
                            auto temp = que.back();
                            que.back() = que[ relevant_index ];
                            que.back().output.speed = 0;
                            que.back().output.heading = angle;
                            que.push_back( temp );
                        }
                    }
                    while( 2 < que.size() ) { que.front().write( ostrm, csv ); que.pop_front(); }
                    unique_points = 2;
                }
            }
            for( auto ii = 0U; ii < que.size(); ii++ ) { que[ ii ].write( ostrm, csv ); }
        }
        else if( "decelerate" == operations[ 0 ] )
        {
            comma::csv::input_stream< generic::input_t > istrm( std::cin, csv );

            comma::csv::output_stream< generic::input_t > ostrm( std::cout, csv );
            std::deque< deceleration::record_t > que;

            auto const deceleration = options.value< double >( "--deceleration,-c", 0.5 );
            auto max_speed = 0.0;
            while( istrm.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                auto input = istrm.read(); if( !input ) break;
                que.emplace_back( *input, istrm, csv );

                max_speed = std::max( que.back().input.speed, max_speed );
                auto const affected_distance = max_speed * max_speed / ( 2 * deceleration ); // kinematic equation with min_speed = 0.0

                auto curri = que.rbegin();
                auto previ = curri + 1;
                double updated_speed, distance;

                for( ; previ != que.rend()
                         && comma::math::less( 0.0, distance = ( curri->input.coordinates - previ->input.coordinates ).norm() )
                         && comma::math::less( updated_speed = std::sqrt( curri->input.speed * curri->input.speed + 2 * deceleration * distance )
                                             , previ->input.speed )
                     ; previ++, curri++ )
                {
                    previ->input.speed = updated_speed;
                }
                while(( que.back().input.coordinates - que.front().input.coordinates ).norm() > affected_distance )
                {
                    que.front().write( ostrm, csv );
                    que.pop_front();
                }
            }
            for( auto ii = 0U; ii < que.size(); ii++ ) { que[ ii ].write( ostrm, csv ); }
        }
        else { std::cerr << comma::verbose.app_name() << ": unknown operation: " <<  operations[ 0 ] << std::endl; }

    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 0;
}

