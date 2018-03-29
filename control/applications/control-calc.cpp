// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2018 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <iostream>
#include <boost/optional.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/format.h>
#include <comma/csv/names.h>
#include <comma/csv/traits.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/name_value/name_value.h>
#include "../../math/roll_pitch_yaw.h"
#include "../../timing/timestamped.h"
#include "../../timing/traits.h"
#include "../../visiting/traits.h"
#include "../wayline.h"

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " carrot"
        " --input-fields --input-format --output-fields --output-format"
        " --nav-fields --nav-format --nav --max-error --max-look-ahead --search --stuck-timeout"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nvarious control-related operations";
    std::cerr << "\n";
    std::cerr << "\nusage:";
    std::cerr << "\n    <plan> | control-calc <operation>";
    std::cerr << "\n";
    std::cerr << "\nwhere <operation> is one of \"carrot\"";
    std::cerr << "\n";
    std::cerr << "\ngeneral options:";
    std::cerr << "\n    --input-fields; show input fields and exit";
    std::cerr << "\n    --input-format; show input format and exit";
    std::cerr << "\n    --output-fields; show output fields and exit";
    std::cerr << "\n    --output-format; show output format and exit";
    std::cerr << "\n    --verbose,-v; more output";
    std::cerr << "\n";
    std::cerr << "\ncarrot operation options:";
    std::cerr << "\n    --max-error=<ratio>; maximum error between full and direct path";
    std::cerr << "\n    --max-look-ahead=[<metres>]; maximum distance to look ahead";
    std::cerr << "\n    --nav=<host>:<port>; navigation feed";
    std::cerr << "\n    --nav-fields; show navigation fields and exit";
    std::cerr << "\n    --nav-format; show navigation format and exit";
    std::cerr << "\n    --search=[<metres>]; distance from current point to search for nearest point";
    std::cerr << "\n    --stuck-timeout=[<seconds>]; timeout before trying to become unstuck";
    std::cerr << "\n";
    std::cerr << "\n    --search is used to limit the distance along the path which is searched for";
    std::cerr << "\n    the nearest point. This is useful if the path crosses over itself or later";
    std::cerr << "\n    parts of the path run close to earlier parts. e.g. any normal up-and-back";
    std::cerr << "\n    pattern. Without this option if the robot was pushed off the path it would";
    std::cerr << "\n    likely pick-up the geographically near, but path-wise distant, point as";
    std::cerr << "\n    it's nearest point.";
    std::cerr << "\n";
    std::cerr << "\n    If the nearest point doesn't advance for --stuck-timeout seconds the robot";
    std::cerr << "\n    is considered to be stuck. To become unstuck the nearest point is pushed";
    std::cerr << "\n    forward by one space. If the robot remains stuck for further --stuck-timeout";
    std::cerr << "\n    periods the procedure is repeated.";
    std::cerr << "\n    No action is taken if --stuck-timeout is unset.";
    std::cerr << "\n";
    std::cerr << "\nexamples:";
    std::cerr << "\n    cat plan.csv | control-calc carrot --nav tcp:localhost:12345 --max-error=1.005";
    std::cerr << "\n" << std::endl;
    exit( 1 );
}

template< typename T > std::string field_names( bool full_xpath = false )
{
    return comma::join( comma::csv::names< T >( full_xpath ), ',' );
}

template< typename T > std::string format( bool full_xpath = false, const std::string& fields = "" )
{
    return comma::csv::format::value< T >( !fields.empty()
                                         ? fields
                                         : field_names< T >( full_xpath ), full_xpath );
}

namespace snark { namespace control_calc {

class carrot_op
{
public:
    typedef snark::control::wayline::position_t position_t;

    struct record_t
    {
        position_t coordinates;
        std::string line;
    
        record_t() {}
        record_t( const position_t& coordinates, const std::string& line )
            : coordinates( coordinates )
            , line( line )
        {}

        void output( const comma::csv::options& csv ) const
        {
            std::cout.write( &line[0], line.size() );
            if( !csv.binary() ) { std::cout << "\n"; }
            std::cout.flush();
        }
    };

    struct nav_data
    {
        boost::posix_time::ptime t;
        Eigen::Vector3d position;
        snark::roll_pitch_yaw orientation;
    };

    struct point_carrot
    {
        record_t point;
        record_t carrot;

        point_carrot() {}
        point_carrot( record_t point, record_t carrot ): point( point ), carrot( carrot ) {}
    };

    carrot_op( const comma::command_line_options& options )
        : options( options )
        , csv( options )
        , nearest_point_index( 0 )
        , last_nearest_point_index( 0 )
        , most_advanced_point_index( 0 )
    {
        csv.full_xpath = false;
        last_advanced_time = boost::posix_time::microsec_clock::universal_time();
    }

    int run()
    {
        if( options.exists( "--input-fields" ) ) { std::cout << field_names< position_t >() << std::endl; return 0; }
        if( options.exists( "--input-format" ) ) { std::cout << format< position_t >() << std::endl; return 0; }
        if( options.exists( "--nav-fields" ) ) { std::cout << field_names< nav_data >() << std::endl; return 0; }
        if( options.exists( "--nav-format" ) ) { std::cout << format< nav_data >() << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { std::cout << ( !csv.fields.empty() ? csv.fields : field_names< position_t >() ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << ( csv.binary() ? csv.format().string() : format< position_t >() ) << std::endl; return 0; }

        double max_error = options.value< double >( "--max-error" );
        boost::optional< double > max_look_ahead = boost::make_optional( options.exists( "--max-look-ahead" )
                                                                       , options.value( "--max-look-ahead", double() ));
        boost::optional< double > search_distance = boost::make_optional( options.exists( "--search" )
                                                                        , options.value( "--search", double() ));
        boost::optional< int > stuck_timeout_ms = boost::make_optional( options.exists( "--stuck-timeout" )
                                                                      , int( options.value( "--stuck-timeout", double() ) * 1000 ));

        comma::csv::input_stream< position_t > input_stream( std::cin, csv, position_t() );

        std::string nav_option = options.value< std::string >( "--nav" );
        comma::csv::options nav_csv = comma::name_value::parser( "filename", ';', '=', false ).get< comma::csv::options >( nav_option );
        nav_csv.full_xpath = false;
        if( nav_csv.fields.empty() ) { nav_csv.fields = field_names< nav_data >(); }
        comma::io::istream nav_in( nav_csv.filename, nav_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
        comma::csv::input_stream< nav_data > nav_stream( *nav_in, nav_csv );
        comma::io::select select;
        select.read().add( nav_in );

        // Wait for nav to be ready
        if( select.wait( boost::posix_time::seconds( 5 )))
        {
            const nav_data* nav = nav_stream.read();
            if( !nav ) { std::cerr << "control-calc: nav stream error" << std::endl; return 1; }
        }
        else
        {
            std::cerr << "control-calc: nav is not publishing data" << std::endl;
            return 1;
        }
        select.read().add( comma::io::stdin_fd );

        record_t record;
        if( csv.binary() ) { record.line.resize( csv.format().size() ); }

        comma::signal_flag is_shutdown;
        while( !is_shutdown )
        {
            if( !input_stream.ready() && !nav_stream.ready() ) { select.wait( boost::posix_time::milliseconds( 10 )); }

            while( !is_shutdown && ( input_stream.ready() || ( select.check() && select.read().ready( comma::io::stdin_fd ))))
            {
                const position_t* p = input_stream.read();
                if( !p )
                {
                    if( comma::verbose )
                    {
                        std::cerr << "control-calc: end of input stream" << std::endl;
                        for( const point_carrot& pc : point_carrots ) { std::cerr << "control-calc: point " << pc.point.line << " to carrot " << pc.carrot.line << std::endl; }
                    }
                    select.read().remove( comma::io::stdin_fd ); break;
                }
                record.coordinates = *p;
                if( csv.binary() ) { ::memcpy( &record.line[0], input_stream.binary().last(), record.line.size() ); }
                else { record.line = comma::join( input_stream.ascii().last(), csv.delimiter ); }
                point_carrots.push_back( point_carrot( record, record ));
                update_point_carrots( max_error, max_look_ahead );
            }
            while( !is_shutdown && ( nav_stream.ready() || ( select.check() && select.read().ready( nav_in ))))
            {
                const nav_data* p = nav_stream.read();
                if( !p ) { comma::verbose << "end of nav stream" << std::endl; return 1; }
                carrot( p->position[0], p->position[1], search_distance, stuck_timeout_ms ).output( csv );
            }
        }
        return 0;
    }

private:
    // find the nearest point on the path to the current point
    // and return the associated carrot
    record_t carrot( double x, double y
                   , const boost::optional< double >& search_distance
                   , const boost::optional< int >& stuck_timeout_ms )
    {
        position_t current_point( x, y );

        // limit the search if desired
        size_t first_search_point = 0;
        size_t last_search_point = point_carrots.size() - 1;
        if( search_distance )
        {
            size_t i = nearest_point_index;
            double distance = 0;
            for( i = nearest_point_index; i > 0; i-- )
            {
                distance += ( point_carrots[i].point.coordinates - point_carrots[i-1].point.coordinates ).norm();
                if( distance > *search_distance ) { break; }
            }
            first_search_point = i;

            i = nearest_point_index;
            distance = 0;
            for( i = nearest_point_index; i < point_carrots.size() - 1; i++ )
            {
                distance += ( point_carrots[i].point.coordinates - point_carrots[i+1].point.coordinates ).norm();
                if( distance > *search_distance ) { break; }
            }
            last_search_point = i;
        }

        // find nearest point
        double min_distance = std::numeric_limits< double >::max();
        for( size_t i = first_search_point; i <= last_search_point; i++ )
        {
            double d = ( point_carrots[i].point.coordinates - current_point ).norm();
            if( d < min_distance ) { min_distance = d; nearest_point_index = i; }
        }

        if( nearest_point_index != last_nearest_point_index )
        {
            last_nearest_point_index = nearest_point_index;
            comma::verbose << "moved nearest to " << point_carrots[nearest_point_index].point.coordinates[0] << "," << point_carrots[nearest_point_index].point.coordinates[1] << std::endl;
            if( nearest_point_index > most_advanced_point_index )
            {
                most_advanced_point_index = nearest_point_index;
                last_advanced_time = boost::posix_time::microsec_clock::universal_time();
            }
        }

        size_t nudged_nearest_point_index = nearest_point_index;
        if( stuck_timeout_ms )
        {
            // for how many "timeout" periods have we not advanced?
            unsigned int num_stuck_timeouts = ( boost::posix_time::microsec_clock::universal_time() - last_advanced_time ).total_milliseconds() / *stuck_timeout_ms;
            if( num_stuck_timeouts > 0 )
            {
                nudged_nearest_point_index = nearest_point_index + num_stuck_timeouts;
                if( nudged_nearest_point_index >= point_carrots.size() ) { nudged_nearest_point_index = point_carrots.size() - 1; }
                comma::verbose << "nudging nearest point by " << num_stuck_timeouts << " to " << point_carrots[nudged_nearest_point_index].point.coordinates[0] << "," << point_carrots[nudged_nearest_point_index].point.coordinates[1] << std::endl;
            }
        }

        return point_carrots[nudged_nearest_point_index].carrot;
    }

    // look backwards along the path from the point we've just added and
    // record this as the target point (carrot) for any point close enough
    void update_point_carrots( double max_error, boost::optional< double > max_look_ahead )
    {
        double cumulative_distance = 0;
        position_t* prev_coordinates = nullptr;
        const record_t& last_point = point_carrots.back().point;
        for( auto it = point_carrots.rbegin(); it != point_carrots.rend(); ++it )
        {
            if( prev_coordinates )
            {
                double straight_line_distance = ( last_point.coordinates - it->point.coordinates ).norm();
                cumulative_distance += ( it->point.coordinates - *prev_coordinates ).norm();
                if( cumulative_distance <= straight_line_distance * max_error &&
                  !( max_look_ahead && cumulative_distance > *max_look_ahead ))
                {
                    it->carrot = last_point;
                }
            }
            prev_coordinates = &it->point.coordinates;
        }
    }

    comma::command_line_options options;
    comma::csv::options csv;
    std::deque< point_carrot > point_carrots;
    size_t nearest_point_index;
    size_t last_nearest_point_index;
    size_t most_advanced_point_index;
    boost::posix_time::ptime last_advanced_time;
};

} } // namespace snark { namespace control_calc {

namespace comma { namespace visiting {
    
template <> struct traits< snark::control_calc::carrot_op::nav_data >
{
    template < typename Key, class Visitor > static void visit( const Key&, snark::control_calc::carrot_op::nav_data& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }

    template < typename Key, class Visitor > static void visit( const Key&, const snark::control_calc::carrot_op::nav_data& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        comma::csv::options input_csv( options );
        std::vector< std::string > unnamed = options.unnamed( "--help,-h,--verbose,-v,--input-fields,--input-format,--nav-fields,--nav-format,--output-fields,--output-format", "-.*" );
        if( unnamed.empty() ) { std::cerr << "control-calc: operation required" << std::endl; return 1; }
        const std::string& operation = unnamed[0];
        if( operation == "carrot" ) { return snark::control_calc::carrot_op( options ).run(); }
        std::cerr << "control-calc: expected operation, got: \"" << operation << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "control-calc: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "control-calc: unknown exception" << std::endl; }
    return 1;
}
