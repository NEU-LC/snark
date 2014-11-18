// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#include <stdint.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/numeric/conversion/cast.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/date_time/posix_time/posix_time_duration.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/thread.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/math/compare.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include "../traits.h"
#include "../commands.h"
#include "../commands_handler.h"
#include "../inputs.h"
#include "../units.h"
#include "../tilt_sweep.h"
#include "../waypoints_follower.h"
#include "../../../sensors/hokuyo/traits.h"
extern "C" {
    #include "../simulink/Arm_controller_v2.h"
}
#include "../simulink/traits.h"

/* External inputs (root inport signals with auto storage) */
extern ExtU_Arm_controller_v2_T Arm_controller_v2_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_Arm_controller_v2_T Arm_controller_v2_Y;

static const char* name() { return "ur-arm-control: "; }

using snark::ur::handlers::input_primitive;
using snark::ur::handlers::result;

void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << std::endl;
    std::cerr << "example: socat tcp-listen:9999,reuseaddr EXEC:\"snark-ur10-control --id 7 -ip 192.168.0.10 -p 8888\" " << name() << " " << std::endl;
    std::cerr << "          Listens for commands from TCP port 9999, process command and send control string to 192.168.0.10:8888" << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:            show this message" << std::endl;
    std::cerr << "    --versbose,-v:        show messages to the robot arm - angles are changed to degrees." << std::endl;
    std::cerr << "*   --id=:                ID to identify commands, eg. ><ID>,999,set_pos,home;" << std::endl;
    std::cerr << "*   --robot-arm-host=:    Host name or IP of the robot arm." << std::endl;
    std::cerr << "*   --robot-arm-port=:    TCP Port number of the robot arm." << std::endl;
    std::cerr << "*   --feedback-host=:     Host name or IP of the robot arm's feedback." << std::endl;
    std::cerr << "*   --feedback-port=:     TCP Port number of the robot arm's feedback." << std::endl;
    std::cerr << "    --sleep=:             Loop sleep value in seconds, default is 0.2s if not specified." << std::endl;
    std::cerr << "*   --config=:            Config file for robot arm, see --output-config." << std::endl;
    std::cerr << "*   --scan-forwarding-port=|-P=:" << std::endl;
    std::cerr << "                          Broadcast scanned data using TCP on this port." << std::endl;
    std::cerr << "    --output-config=:     Print config format in json." << std::endl;
    // std::cerr << "    --init-force-limit,-ifl:" << std::endl;
    // std::cerr << "                          Force (Newtons) limit when auto initializing, if exceeded then stop auto init." << std::endl;
    exit ( code );
}


template < typename T > 
comma::csv::ascii< T >& ascii( )
{
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

void output( const std::string& msg, std::ostream& os=std::cout )
{
    os << msg << std::endl;
}

static snark::ur::status_t arm_status; 
/// Stream to command robot arm
namespace ip = boost::asio::ip;
typedef snark::ur::handlers::commands_handler commands_handler_t;
typedef boost::shared_ptr< commands_handler_t > commands_handler_shared;
static commands_handler_shared commands_handler;
static bool verbose = false;
static snark::ur::config config;

template < typename C >
std::string handle( const std::vector< std::string >& line, std::ostream& os )
{
    C c;
    try
    {
        c = C::ascii().get( line );
    }
    catch( boost::bad_lexical_cast& le ) {
        std::ostringstream ss;
        ss << '<' << comma::join( line, ',' ) << ',' << boost::lexical_cast< std::string >( snark::ur::errors::format_error )
           << ",\"command format error, wrong field type/s, fields: " << c.names() << " - types: "  << c.serialise() << "\";";
        return ss.str();
    }
    catch( comma::exception& ce ) {
        std::ostringstream ss;
        ss << '<' << comma::join( line, ',' ) << ',' << boost::lexical_cast< std::string >( snark::ur::errors::format_error )
           << ",\"command format error, wrong field/s or field type/s, fields: " << c.names() << " - types: "  << c.serialise() << "\";";
        return ss.str();
    }
    catch( ... ) { COMMA_THROW( comma::exception, "unknown error is parsing: " + comma::join( line , ',' ) ); }
       
    comma::dispatch::handler& h_ref( *commands_handler );
    comma::dispatch::dispatched_base& ref( c );
    ref.dispatch_to( h_ref );
    // perform action
    // result ret = snark::ur::action< C >::run( c, os );
    std::ostringstream ss;
    ss << '<' << c.serialise() << ',' << commands_handler->ret.get_message() << ';';
    return ss.str();
}

void process_command( const std::vector< std::string >& v, std::ostream& os )
{
    if( boost::iequals( v[2], "move_cam" ) )         { output( handle< snark::ur::move_cam >( v, os ) ); }
    else if( boost::iequals( v[2], "move_effector" )){ output( handle< snark::ur::move_effector >( v, os ) ); }
    else if( boost::iequals( v[2], "pan_tilt" ) )    { output( handle< snark::ur::pan_tilt >( v, os ) ); }
    else if( boost::iequals( v[2], "set_pos" ) )     { output( handle< snark::ur::set_position >( v, os ) ); }
    else if( boost::iequals( v[2], "set_home" ) )    { output( handle< snark::ur::set_home >( v, os ) ); }
    else if( boost::iequals( v[2], "power" ) )       { output( handle< snark::ur::power >( v, os )); }  
//     else if( boost::iequals( v[2], "scan" ) )        
//     {
//         if( v.size() >= snark::ur::sweep_cam::fields )     { output( handle< snark::ur::sweep_cam >( v, os )); }
//         else 
//         {
//             // If the user did not enter a sweep angle as last field, use the default
//             // You can either do this or create another scan command with the extra sweep_angle field
//             static std::string default_sweep = boost::lexical_cast< std::string >( config.continuum.scan.sweep_angle.value() );
//             std::vector< std::string > items = v; 
//             items.push_back( default_sweep );
//             output( handle< snark::ur::sweep_cam >( items, os ) );
//         } 
//     }  
    else if( boost::iequals( v[2], "brakes" ) || boost::iequals( v[2], "stop" ) ) { output( handle< snark::ur::brakes >( v, os )); }  
    else if( boost::iequals( v[2], "cancel" ) ) { } /// No need to do anything, used to cancel other running commands e.g. auto_init or scan
    else if( boost::iequals( v[2], "auto_init" ) )  
    { 
        if( v.size() == snark::ur::auto_init_force::fields ) { output( handle< snark::ur::auto_init_force >( v, os )); }
        else { output( handle< snark::ur::auto_init >( v, os )); } 
    }  
    else if( boost::iequals( v[2], "initj" ) ) { output( handle< snark::ur::joint_move >( v, os )); }  
    else { output( comma::join( v, v.size(), ',' ) + ',' + boost::lexical_cast< std::string >( snark::ur::errors::unknown_command ) + ",\"unknown command found: '" + v[2] + "'\"" ); return; }
}

/// Return null if no status
void read_status( comma::csv::binary_input_stream< snark::ur::status_t >& iss, comma::io::istream& stream,
    comma::io::select& select, const comma::io::file_descriptor& fd )
{
    /// Within 100ms, we are guranteed a new status, there may already be many statuses waiting to be read
    static const boost::posix_time::milliseconds timeout( 0.1 * 1000000u );
    select.wait( timeout );
    if( !select.read().ready( fd ) ) { 
        std::cerr << "no status received within timeout of " << timeout.total_milliseconds() << "ms" << std::endl; 
        COMMA_THROW( comma::exception, "no status received within timeout of " << timeout.total_milliseconds() << "ms" ); 
    }
    arm_status = *( iss.read() );
    while( stream->rdbuf()->in_avail() > 0 )  { arm_status = *( iss.read() ); }

    if( arm_status.length != snark::ur::fixed_status::size ) {
        std::cerr << name() << "status data alignment check failed" << std::endl; 
        COMMA_THROW( comma::exception, "status data alignment check failed" ); 
    }
}

class stop_on_exit
{
    std::ostream& os_;
public:
    stop_on_exit( std::ostream& oss ) : os_(oss) {}
    ~stop_on_exit() 
    {
        std::cerr << name() << "powering off on exit." << std::endl;
        os_ << "stopj([0.1,0.1,0.1,0.1,0.1,0.1])\n";
        os_ << "power off\n"; // power off too
        os_.flush();
    }

};

void load_config( const std::string& filepath )
{
    std::ifstream config_ifs( filepath.c_str() );
    if( !config_ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open file: " + filepath ); }

    boost::property_tree::ptree t;
    comma::from_ptree( t, true );
    boost::property_tree::read_json( config_ifs, t );
    comma::from_ptree from_ptree( t, true );
    comma::visiting::apply( from_ptree ).to( config );
}

/// Create a home position file if arm is running and is in home position
void home_position_check( const snark::ur::status_t& status, const std::string& homefile )
{
    static const boost::filesystem::path path( homefile );
    if( status.is_running() )
    {
        if( status.check_pose( config.continuum.home_position ) ) { std::ofstream( homefile.c_str(), std::ios::out | std::ios::trunc ); } // create
        else { boost::filesystem::remove( path ); } // remove
    }
}

bool should_stop( snark::ur::inputs& in )
{
    static const boost::posix_time::seconds timeout( 0 );
    in.read( timeout );
    return ( !in.is_empty() );
}

struct ttt : public boost::array< double, 6 > {};

namespace comma { namespace visiting {
    
// Commands
template < > struct traits< ttt >
{
    template< typename K, typename V > static void visit( const K& k, ttt& t, V& v )
    {
        v.apply( "angles", (boost::array< double, 6 >&) t );
    }
    template< typename K, typename V > static void visit( const K& k, const ttt& t, V& v )
    {
        v.apply( "angles", (const boost::array< double, 6 >&) t );
    }
};

}}


int main( int ac, char** av )
{
    comma::signal_flag signaled;
    comma::command_line_options options( ac, av );
    if( options.exists( "-h,--help" ) ) { usage( 0 ); }
    if( options.exists( "--output-config") )
    {
        boost::property_tree::ptree t;
        comma::to_ptree to_ptree( t );
        comma::visiting::apply( to_ptree ).to( config );
        boost::property_tree::write_json( std::cout, t );  
    }
    
    using boost::posix_time::microsec_clock;
    using boost::posix_time::seconds;
    using boost::posix_time::ptime;
    using boost::asio::ip::tcp;

    double acc = 0.5;
    double vel = 0.1;

    std::cerr << name() << "started" << std::endl;
    try
    {
        /// Convert simulink output into arm's command
        snark::ur::handlers::arm_output output( acc * snark::ur::angular_acceleration_t::unit_type(), vel * snark::ur::angular_velocity_t::unit_type(),
                       Arm_controller_v2_Y );
    
        comma::uint16 rover_id = options.value< comma::uint16 >( "--id" );
        double sleep = options.value< double >( "--sleep", 0.06 );  // seconds

        verbose = options.exists( "--verbose,-v" );

        std::string config_file = options.value< std::string >( "--config" );
        load_config( config_file );
        
        /// home position file
        const snark::ur::continuum_t& continuum = config.continuum;
        if( continuum.work_directory.empty() ) { std::cerr << name() << "cannot find home position directory! exiting!" <<std::endl; return 1; }
        boost::filesystem::path dir( continuum.work_directory );
        if( !boost::filesystem::exists( dir ) || !boost::filesystem::is_directory( dir ) ) { std::cerr << name() << "work_directory must exists: " << continuum.work_directory << std::endl; return 1; }

        for( std::size_t j=0; j<snark::ur::joints_num; ++j )
        {
            std::cerr << name() << "home joint " << j << " - " << continuum.home_position[j].value() << '"' << std::endl;
        }

        std::string arm_conn_host = options.value< std::string >( "--robot-arm-host" );
        std::string arm_conn_port = options.value< std::string >( "--robot-arm-port" );
        std::string arm_feedback_host = options.value< std::string >( "--feedback-host" );
        std::string arm_feedback_port = options.value< std::string >( "--feedback-port" );
        std::string tcp_scan_forwarding = "tcp:" + boost::lexical_cast< std::string >( continuum.lidar.scan_forwarding_port );
        comma::io::publisher scan_broadcast( tcp_scan_forwarding, comma::io::mode::binary );
        
        boost::scoped_ptr< comma::io::ostream > poss;
        try
        {
            const std::string cmd_str = "tcp:" + arm_conn_host + ':' + arm_conn_port;
            std::cerr << name() << "connecting to the robotic arm command channel: " << cmd_str << std::endl;
            poss.reset( new comma::io::ostream( cmd_str, comma::io::mode::ascii, comma::io::mode::non_blocking ) );
        }
        catch( comma::exception& e )
        {
            std::cerr << name() << "failed to connect to tcp:" << arm_conn_host << ':' << arm_conn_port << std::endl;
            return 1;
        }
        comma::io::ostream& robot_arm = *poss;

        /// For reading  commands from stdin with specific rover id as filter
        snark::ur::inputs inputs( rover_id );

        typedef std::vector< std::string > command_vector;
        const comma::uint32 usec( sleep * 1000000u );
        
        std::string status_conn = "tcp:" + arm_feedback_host + ':' + arm_feedback_port;
        std::cerr << name() << "status connection to feedback status: " << status_conn << std::endl;
        comma::io::istream status_stream( status_conn, comma::io::mode::binary );
        /// Status input is always expected to be binary format
        comma::csv::options csv_in;
        csv_in.full_xpath = true;
        csv_in.format( comma::csv::format::value< snark::ur::status_t >( "", true ) );
        comma::csv::binary_input_stream< snark::ur::status_t > istream( *status_stream, csv_in );
        /// For reading status input, if failed to wait then status stream is dead
        comma::io::select select;
        select.read().add( status_stream.fd() );


        {
            stop_on_exit on_exit( *robot_arm );

            /// Create the handler for auto init.
            snark::ur::handlers::auto_initialization auto_init( arm_status, *robot_arm,
                    boost::bind( read_status, boost::ref(istream), boost::ref( status_stream ), select, status_stream.fd() ),
                    signaled, 
                    boost::bind( should_stop, boost::ref( inputs ) ),
                    continuum.work_directory );
            auto_init.set_app_name( name() );
            /// This is the handler for following waypoints given by Simulink code
            snark::ur::handlers::waypoints_follower waypoints_follower( output, 
                    boost::bind( read_status, boost::ref(istream), boost::ref( status_stream ), select, status_stream.fd() ),
                    arm_status,
                    boost::bind( should_stop, boost::ref( inputs ) ),
                    signaled );
            waypoints_follower.name( name() );
            
            // This is the infomation and generic function for recording some data
            // It records data for scan command starting from waypoint 2 and ends at waypoint 3
            snark::ur::handlers::commands_handler::optional_recording_t record_info;

            /// This is the command handler for all commands
            commands_handler.reset( new commands_handler_t( Arm_controller_v2_U, output, arm_status, *robot_arm, 
                                                            auto_init, waypoints_follower, record_info, 
                                                            std::cout, continuum ) );
        

            boost::posix_time::microseconds timeout( usec );
            while( !signaled && std::cin.good() )
            {
                if( !status_stream->good() ) { 
                    std::cerr << name() << "status connection to robot-arm failed" << std::endl;
                    COMMA_THROW( comma::exception, "status connection to robot arm failed." ); 
                }
                // Read and update the latest status from the robot arm, put it into arm_status
                read_status( istream, status_stream, select, status_stream.fd() ); 
                // Checks for home position and create the home file if true, else remove home file.
                home_position_check( arm_status, auto_init.home_filepath() );
                
                /// Also act as sleep, reads commands from stdin
                inputs.read( timeout );
                // Process commands into inputs into the system
                if( !inputs.is_empty() )
                {
                    const command_vector v = inputs.front();
                    inputs.pop();
                    process_command( v, *robot_arm );
    
                }
                
                usleep( 1000 );
            }
            std::cerr << name() << "exiting" << std::endl;
            *robot_arm << "power off\n";
            robot_arm->flush();
        }
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
