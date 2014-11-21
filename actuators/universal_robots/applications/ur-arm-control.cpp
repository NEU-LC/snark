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

static const char* name() { return "ur-arm-control: "; }

using snark::ur::handlers::result;

void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << std::endl;
    std::cerr << "example: socat tcp-listen:9999,reuseaddr EXEC:\"ur-arm-control --config=<config file> --id 7 --robot-arm-host=<arm ip address> --robot-arm-port=30002 --feedback-host=localhost --feedback-port=9998 \" " << name() << " " << std::endl;
    std::cerr << "          Listens for commands from TCP port 9999, process command and send feedback to localhost:9998" << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:            show this message" << std::endl;
    std::cerr << "    --versbose,-v:        show messages to the robot arm - angles are changed to degrees." << std::endl;
    std::cerr << "*   --robot-arm-host=:    Host name or IP of the robot arm." << std::endl;
    std::cerr << "*   --robot-arm-port=:    TCP Port number of the robot arm." << std::endl;
    std::cerr << "*   --feedback-host=:     Host name or IP of the robot arm's feedback." << std::endl;
    std::cerr << "*   --feedback-port=:     TCP Port number of the robot arm's feedback." << std::endl;
    std::cerr << "    --sleep=:             Loop sleep value in seconds, default is 0.2s if not specified." << std::endl;
    std::cerr << "*   --config=:            Config file for robot arm, see --output-config." << std::endl;
    std::cerr << "    --output-config=:     Print config format in json." << std::endl;
    exit ( code );
}

static snark::ur::status_t arm_status; 
typedef snark::ur::handlers::commands_handler commands_handler_t;
typedef boost::shared_ptr< commands_handler_t > commands_handler_shared;
static commands_handler_shared commands_handler;
static bool verbose = false;
static snark::ur::config_t config;

template < typename T >
std::string handle( const std::vector< std::string >& line, std::ostream& os )
{
    static comma::csv::ascii< T > ascii;
    T command;
    try { command = ascii.get( line ); }
    catch( boost::bad_lexical_cast& le ) { std::ostringstream ss; ss << comma::join( line, ',' ) << ":\"format error, wrong field type/s, expected fields: " << command.names(); return ss.str(); }
    catch( comma::exception& ce ) { std::ostringstream ss; ss << comma::join( line, ',' ) << ":\"format error, wrong field/s or field type/s, expected fields: " << command.names(); return ss.str(); }
    catch( ... ) { COMMA_THROW( comma::exception, "unknown error is parsing: " + comma::join( line , ',' ) ); }       
    comma::dispatch::handler& h_ref( *commands_handler );
    comma::dispatch::dispatched_base& ref( command );
    ref.dispatch_to( h_ref );
    std::ostringstream ss; ss << command.serialise() << ':' << commands_handler->ret.get_message() << ';'; return ss.str();
}

void process_command( const std::vector< std::string >& v, std::ostream& os )
{
    if( boost::iequals( v[0], "power" ) )       { std::cout << handle< snark::ur::power >( v, os ) << std::endl; }  
    else if( boost::iequals( v[0], "brakes" ) || boost::iequals( v[2], "stop" ) ) { std::cout << handle< snark::ur::brakes >( v, os ) << std::endl; }  
    else if( boost::iequals( v[0], "cancel" ) ) { } /// No need to do anything, used to cancel other running commands e.g. auto_init
    else if( boost::iequals( v[0], "auto_init" ) ) { std::cout << handle< snark::ur::auto_init >( v, os ) << std::endl; }
    else if( boost::iequals( v[0], "initj" ) ) { std::cout << handle< snark::ur::joint_move >( v, os ) << std::endl; }
    else { std::cout << comma::join( v, v.size(), ',' ) << ":\"unknown command\"" << std::endl; return; }
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
public:
    stop_on_exit( std::ostream& oss ) : os_(oss) {}
    ~stop_on_exit() 
    {
        std::cerr << name() << "powering off on exit." << std::endl;
        os_ << "stopj([0.1,0.1,0.1,0.1,0.1,0.1])\n";
        os_ << "power off\n"; // power off too
        os_.flush();
    }
private:
    std::ostream& os_;
};

void load_config( const std::string& filepath )
{
    std::ifstream config_ifs( filepath.c_str() );
    if( !config_ifs.is_open() ) { COMMA_THROW( comma::exception, "failed to open file: " + filepath ); }
    boost::property_tree::ptree t;
    boost::property_tree::read_json( config_ifs, t );
    comma::from_ptree from_ptree( t, true );
    comma::visiting::apply( from_ptree ).to( config );
}

bool should_stop( snark::ur::inputs& in )
{
    static const boost::posix_time::seconds timeout( 0 );
    in.read( timeout );
    return ( !in.is_empty() );
}

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

    std::cerr << name() << "started" << std::endl;
    try
    {
        double sleep = options.value< double >( "--sleep", 0.06 );
        verbose = options.exists( "--verbose,-v" );
        std::string config_file = options.value< std::string >( "--config" );
        load_config( config_file );
        if( config.work_directory.empty() ) { std::cerr << name() << "expected to find work_directory in config file, got empty string" <<std::endl; return 1; }
        boost::filesystem::path dir( config.work_directory );
        if( !boost::filesystem::exists( dir ) || !boost::filesystem::is_directory( dir ) ) { std::cerr << name() << "work_directory must exists: " << config.work_directory << std::endl; return 1; }
        for( std::size_t j=0; j<snark::ur::joints_num; ++j )
        {
            std::cerr << name() << "home joint " << j << " - " << config.home_position[j].value() << '"' << std::endl;
        }
        std::string arm_conn_host = options.value< std::string >( "--robot-arm-host" );
        std::string arm_conn_port = options.value< std::string >( "--robot-arm-port" );
        std::string arm_feedback_host = options.value< std::string >( "--feedback-host" );
        std::string arm_feedback_port = options.value< std::string >( "--feedback-port" );
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
        snark::ur::inputs inputs; /// For reading  commands from stdin with specific robot id as filter
        typedef std::vector< std::string > command_vector;
        const comma::uint32 usec( sleep * 1000000u );
        std::string status_conn = "tcp:" + arm_feedback_host + ':' + arm_feedback_port;
        std::cerr << name() << "status connection to feedback status: " << status_conn << std::endl;
        comma::io::istream status_stream( status_conn, comma::io::mode::binary );
        comma::csv::options csv_in;
        csv_in.full_xpath = true;
        csv_in.format( comma::csv::format::value< snark::ur::status_t >( "", true ) );
        comma::csv::binary_input_stream< snark::ur::status_t > istream( *status_stream, csv_in );
        comma::io::select select; /// For reading status input, if failed to wait then status stream is dead
        select.read().add( status_stream.fd() );
        {
            stop_on_exit on_exit( *robot_arm );
            snark::ur::handlers::auto_initialization auto_init( arm_status, *robot_arm, boost::bind( read_status, boost::ref(istream), boost::ref( status_stream ), select, status_stream.fd() ),
                signaled, boost::bind( should_stop, boost::ref( inputs ) ) );
            auto_init.set_app_name( name() );
            commands_handler.reset( new commands_handler_t( arm_status, *robot_arm, auto_init, std::cout, config ) );
            boost::posix_time::microseconds timeout( usec );
            while( !signaled && std::cin.good() )
            {
                if( !status_stream->good() ) { COMMA_THROW( comma::exception, name() << "status connection to robot arm failed." ); }
                read_status( istream, status_stream, select, status_stream.fd() ); // Read and update the latest status from the robot arm, put it into arm_status
                inputs.read( timeout ); // Also act as sleep, reads commands from stdin
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
