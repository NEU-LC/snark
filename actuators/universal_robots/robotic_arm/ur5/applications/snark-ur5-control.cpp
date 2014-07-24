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
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/convenience.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include "../../traits.h"
#include "../../commands.h"
#include "../../commands_handler.h"
#include "../../inputs.h"
extern "C" {
    #include "../../simulink/Arm_Controller.h"
}
#include "../../simulink/traits.h"

/* External inputs (root inport signals with auto storage) */
extern ExtU_Arm_Controller_T Arm_Controller_U;

/* External outputs (root outports fed by signals with auto storage) */
extern ExtY_Arm_Controller_T Arm_Controller_Y;

static const char* name() {
    return "robot-arm-daemon: ";
}

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
} // namespace impl_ {


namespace arm = snark::ur::robotic_arm;
using arm::handlers::input_primitive;
using arm::handlers::result;

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
    std::cerr << "*   --status-port=|-sp=:  TCP service port the statuses will be broadcasted on. See below." << std::endl;
    std::cerr << "*   --robot-arm-host=:    Host name or IP of the robot arm." << std::endl;
    std::cerr << "*   --robot-arm-port=:    TCP Port number of the robot arm." << std::endl;
    std::cerr << "*   --feedback-host=:     Host name or IP of the robot arm's feedback." << std::endl;
    std::cerr << "*   --feedback-port=:     TCP Port number of the robot arm's feedback." << std::endl;
    std::cerr << "    --sleep=:             Loop sleep value in seconds, default is 0.2s if not specified." << std::endl;
    std::cerr << "*   --config=:            Config file for robot arm." << std::endl;
    typedef arm::current_positions current_positions_t;
    comma::csv::binary< current_positions_t > binary;
    std::cerr << "UR10's status:" << std::endl;
    std::cerr << "   format: " << binary.format().string() << " total size is " << binary.format().size() << " bytes" << std::endl;
    std::vector< std::string > names = comma::csv::names< current_positions_t >();
    std::cerr << "   fields: " << comma::join( names, ','  ) << " number of fields: " << names.size() << std::endl;
    std::cerr << std::endl;
    exit ( code );
}


template < typename T > 
comma::csv::ascii< T >& ascii( )
{
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

typedef boost::units::quantity< boost::units::si::angular_acceleration > angular_acceleration_t;
typedef boost::units::quantity< boost::units::si::angular_velocity > angular_velocity_t;


class arm_output
{
public:
    typedef arm::current_positions current_positions_t;
private:
    angular_acceleration_t acceleration;
    angular_velocity_t velocity;
    ExtY_Arm_Controller_T& joints;
    current_positions_t& current_positions;
public:
    arm_output( const angular_acceleration_t& ac, const angular_velocity_t& vel,
                ExtY_Arm_Controller_T& output ) : 
                acceleration( ac ), velocity( vel ), joints( output ), 
                current_positions( static_cast< current_positions_t& >( output ) ) 
                {
                    Arm_Controller_initialize();
                }
   ~arm_output() 
    { 
        Arm_Controller_terminate(); 
        std::cout.flush(); 
        close(STDIN_FILENO);
        close(STDOUT_FILENO);
        close(STDERR_FILENO);
    }
                
   std::string debug_in_degrees() const
   {
       std::ostringstream ss;
       ss << "debug: movej([";
       for(std::size_t i=0; i<6u; ++i) 
       {
          ss << static_cast< arm::plane_angle_degrees_t >( joints.joint_angle_vector[i] * arm::radian ).value();
          if( i < 5 ) { ss << ','; }
       }
       ss << "],a=" << acceleration.value() << ','
          << "v=" << velocity.value() << ')';
          
          
       return ss.str();
   }
   std::string serialise() const
   {
       static std::string tmp;
       std::ostringstream ss;
       ss << "movej([" << ascii< ExtY_Arm_Controller_T >( ).put( joints, tmp )
          << "],a=" << acceleration.value() << ','
          << "v=" << velocity.value() << ')';
       return ss.str();
   }
   
   void write_arm_status( comma::io::publisher& publisher )
   {
       // write a byte indicating the status, and joint positions
       static comma::csv::binary< current_positions_t > binary("","",true, current_positions );
       static std::vector<char> line( binary.format().size() );
       binary.put( current_positions, line.data() );
       publisher.write( line.data(), line.size());
   }
                
};


void output( const std::string& msg, std::ostream& os=std::cout )
{
    os << msg << std::endl;
}

static arm::fixed_status arm_status; 
/// Stream to command robot arm
namespace ip = boost::asio::ip;
typedef arm::handlers::commands_handler commands_handler_t;
typedef boost::shared_ptr< commands_handler_t > commands_handler_shared;
static commands_handler_shared commands_handler;

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
        ss << '<' << comma::join( line, ',' ) << ',' << impl_::str( arm::errors::format_error )
           << ",\"command format error, wrong field type/s, fields: " << c.names() << " - types: "  << c.serialise() << "\";";
        return ss.str();
    }
    catch( comma::exception& ce ) {
        std::ostringstream ss;
        ss << '<' << comma::join( line, ',' ) << ',' << impl_::str( arm::errors::format_error )
           << ",\"command format error, wrong field/s or field type/s, fields: " << c.names() << " - types: "  << c.serialise() << "\";";
        return ss.str();
    }
    catch( ... ) { COMMA_THROW( comma::exception, "unknown error is parsing: " + comma::join( line , ',' ) ); }
       
    comma::dispatch::handler& h_ref( *commands_handler );
    comma::dispatch::dispatched_base& ref( c );
    ref.dispatch_to( h_ref );
    // perform action
    // result ret = arm::action< C >::run( c, os );
    std::ostringstream ss;
    ss << '<' << c.serialise() << ',' << commands_handler->ret.get_message() << ';';
    return ss.str();
}

void process_command( const std::vector< std::string >& v, std::ostream& os )
{
    if( boost::iequals( v[2], "move_cam" ) )        { output( handle< arm::move_cam >( v, os ) ); }
    else if( boost::iequals( v[2], "set_pos" ) )  
    {
        if( v.size() == arm::set_position::fields ) 
             { output( handle< arm::set_position >( v, os ) ); } 
        else { output( handle< arm::set_position_giraffe >( v, os ) ); } 
    }
    else if( boost::iequals( v[2], "set_home" ) )   { output( handle< arm::set_home >( v, os ) ); }
    else if( boost::iequals( v[2], "power" ) )      { output( handle< arm::power >( v, os )); }  
    else if( boost::iequals( v[2], "brakes" ) || 
             boost::iequals( v[2], "stop" ) )       { output( handle< arm::brakes >( v, os )); }  
    else if( boost::iequals( v[2], "auto_init" ) )  { output( handle< arm::auto_init >( v, os )); }  
    else if( boost::iequals( v[2], "initj" ) )      { output( handle< arm::joint_move >( v, os )); }  
    // else if( boost::iequals( v[2], "movej" ) )      { output( handle< arm::move_joints >( v, os ) ); }
    else { output( comma::join( v, v.size(), ',' ) + ',' + 
        impl_::str( arm::errors::unknown_command ) + ",\"unknown command found: '" + v[2] + "'\"" ); return; }
}

// /// Connect to the TCP server within the allowed timeout
// /// Needed because comma::io::iostream is not available
// bool tcp_connect( const std::string& host, const std::string& port, 
//                   const boost::posix_time::time_duration& timeout, ip::tcp::iostream& io )
// {
//     using boost::asio::ip::tcp;
//     boost::asio::io_service service;
//     tcp::resolver resolver( service );
//     tcp::resolver::query query( host == "localhost" ? "127.0.0.1" : host, port );
//     tcp::resolver::iterator it = resolver.resolve( query );
//     // Connect and find out if successful or not quickly using timeout
//     io.expires_from_now( timeout );
//     io.connect( it->endpoint() );
//     io.expires_at( boost::posix_time::pos_infin );
//     
//     return io.error() == 0;
// } 

bool ready( comma::io::istream& is )
{
    // std::cerr << "avail: " << is->rdbuf()->in_avail() << std::endl;
    return is->rdbuf()->in_avail() > 0;
}

/// Return null if no status
bool read_status( comma::io::istream& iss )
{
    // std::cerr << "rdbuf" << std::endl;
    iss->read( arm_status.data(), arm::fixed_status::size );
    if( !iss->good() ) {
        // std::cerr  << "not good" << std::endl;
        return false;
    }
    // Keep reading to get latest data
    while( ready( iss ) )
    {
        iss->read( arm_status.data(), arm::fixed_status::size );
        // std::cerr << "ready again read" << std::endl;
    }
    return true;
}

static arm::config config;

class stop_on_exit
{
    ip::tcp::iostream& os_;
public:
    stop_on_exit( ip::tcp::iostream& oss ) : os_(oss) {}
    ~stop_on_exit() 
    {
        os_ << "stopj([0.1,0.1,0.1,0.1,0.1,0.1])\n";
        os_.flush();
        os_.close();
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

namespace fs = boost::filesystem;

int main( int ac, char** av )
{
    
    comma::signal_flag signaled;
    
    comma::command_line_options options( ac, av );
    if( options.exists( "-h,--help" ) ) { usage( 0 ); }
    
    using boost::posix_time::microsec_clock;
    using boost::posix_time::seconds;
    using boost::posix_time::ptime;
    using boost::asio::ip::tcp;

    double acc = 0.5;
    double vel = 0.1;

    std::cerr << name() << "started" << std::endl;
    try
    {
        arm_output output( acc * angular_acceleration_t::unit_type(), vel * angular_velocity_t::unit_type(),
                       Arm_Controller_Y );
    
        comma::uint16 rover_id = options.value< comma::uint16 >( "--id" );
        double sleep = 0.1; // seconds
        if( options.exists( "--sleep" ) ) { sleep = options.value< double >( "--sleep" ); };

        comma::uint32 listen_port = options.value< comma::uint32 >( "--status-port,-sp" );
        
        bool verbose = options.exists( "--verbose,-v" );

        std::string config_file = options.value< std::string >( "--config" );
        load_config( config_file );

        std::string arm_conn_host = options.value< std::string >( "--robot-arm-host" );
        std::string arm_conn_port = options.value< std::string >( "--robot-arm-port" );
        std::string arm_feedback_host = options.value< std::string >( "--feedback-host" );
        std::string arm_feedback_port = options.value< std::string >( "--feedback-port" );
        
        comma::io::ostream robot_arm( "tcp:" + arm_conn_host + ':' + arm_conn_port, 
                                      comma::io::mode::ascii, comma::io::mode::non_blocking );

        // create tcp server for broadcasting status
        std::ostringstream ss;
        ss << "tcp:" << listen_port;
        comma::io::publisher publisher( ss.str(), comma::io::mode::binary );

        arm::inputs inputs( rover_id );

        typedef std::vector< std::string > command_vector;
        const comma::uint32 usec( sleep * 1000000u );
        
        std::string status_conn = "tcp:" + arm_feedback_host + ':' + arm_feedback_port;
        std::cerr << name() << "status connection to feedback status: " << status_conn << std::endl;
        comma::io::istream status_stream( status_conn, comma::io::mode::binary );
        comma::io::select select;
        select.read().add( status_stream.fd() );

        commands_handler.reset( new commands_handler_t( Arm_Controller_U, arm_status, *robot_arm, status_stream, select, signaled ) );

        while( !signaled && std::cin.good() )
        {

            select.check();
            if( ready( status_stream ) || select.read().ready( status_stream.fd() ) ) 
            {
                if( read_status( status_stream ) )
                { 
                    // std::cerr << name() << "robotmode: " << arm_status.robot_mode() << std::endl;
                    // boost::property_tree::ptree t;
                    // comma::to_ptree to_ptree( t );
                    // comma::visiting::apply( to_ptree ).to( arm_status );
                    // boost::property_tree::write_json( std::cerr, t, false );    
                }
            }
            
            try { inputs.read(); }
            catch(...) { COMMA_THROW( comma::exception, "reading from stdcin failed." ); }
            // Process commands into inputs into the system
            if( !inputs.is_empty() )
            {
                const command_vector& v = inputs.front();
                //std::cerr << name() << " got " << comma::join( v, ',' ) << std::endl;
                process_command( v, *robot_arm );

                inputs.pop();
            }
            // Run simulink code
            Arm_Controller_step();
            
            // We we need to send command to arm
            if( Arm_Controller_Y.command_flag > 0 )
            {
                if( verbose ) { std::cerr << name() << output.debug_in_degrees() << std::endl; }
                *robot_arm << output.serialise() << std::endl;
                robot_arm->flush();
                Arm_Controller_U.motion_primitive = real_T( input_primitive::no_action );
            }
            
            // reset inputs
            memset( &Arm_Controller_U, 0, sizeof( ExtU_Arm_Controller_T ) );
            // send out arm's current status: code and joint positions
            output.write_arm_status( publisher );

            if( sleep > 0 ) usleep( usec );
        }

        std::cerr << name() << "exiting" << std::endl;
        *robot_arm << "stopj([0.1,0.1,0.1,0.1,0.1,0.1])\n";
        robot_arm->flush();
        publisher.close();
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
