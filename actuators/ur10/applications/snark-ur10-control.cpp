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
#include <boost/units/systems/si/angular_velocity.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <boost/units/quantity.hpp>
#include <boost/asio.hpp>
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
#include "../traits.h"
#include "../commands.h"
#include "../inputs.h"
extern "C" {
    #include "../simulink/Arm_Controller.h"
}
#include "../simulink/traits.h"

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


void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << std::endl;
    std::cerr << "example: socat tcp-listen:9999,reuseaddr EXEC:\"robot-arm-daemon --id 7 -ip 127.0.0.1 -p 8888\" " << name() << " " << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "*   --id=: ID to identify commands, eg. ><ID>,999,set_pos,home;" << std::endl;
    std::cerr << "*   --address=|-ip=: TCP IP address of the robot arm." << std::endl;
    std::cerr << "*   --status-port=|-sp=: TCP service port the robot arm status will be broadcasted on. Binary, one byte of -1, 0 or 1 value." << std::endl;
    std::cerr << "*   --port=|-p=: TCP port number of robot arm" << std::endl;
    std::cerr << "    --sleep=: loop sleep value in seconds, default to 0.2 if not specified." << std::endl;
    typedef snark::robot_arm::current_positions current_positions_t;
    comma::csv::binary< current_positions_t > binary;
    std::cerr << "UR10's status:" << std::endl;
    std::cerr << "   format: " << binary.format().string() << " total size is " << binary.format().size() << " bytes" << std::endl;
    std::vector< std::string > names = comma::csv::names< current_positions_t >();
    std::cerr << "   fields: " << comma::join( names, ','  ) << " number of fields: " << names.size() << std::endl;
    std::cerr << std::endl;
    exit ( code );
}

namespace arm = snark::robot_arm;

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
    
    angular_acceleration_t acceleration;
    angular_velocity_t velocity;
    ExtY_Arm_Controller_T& joints;
public:
    arm_output( const angular_acceleration_t& ac, const angular_velocity_t& vel,
                ExtY_Arm_Controller_T& output ) : 
                acceleration( ac ), velocity( vel ), joints( output ) {}
                
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
                
};


void output( const std::string& msg, std::ostream& os=std::cout )
{
    os << msg << std::endl;
}

struct input_primitive
{
    enum {
        no_action = 0,
        move_cam = 1,
        set_position = 2,
        set_home=3,      // define home position, internal usage
        movej=4
    };  
};

struct result
{
    struct error { enum { success=0, invalid_input=1 }; };
    int code;
    std::string message;
    
    result( const std::string& msg, int code_ ) : code( code_ ), message( msg ) {}
    result() : code( error::success ), message( "success" ) {}
    
    std::string get_message() const 
    {
        std::ostringstream ss;
        ss << code << ',' << '"' << message << '"';
        return ss.str();
    }
    bool is_success() const { return code == error::success; }
};

template < typename T > struct action;

template < > struct action< arm::move_cam > {
    static result run( const arm::move_cam& cam )
    {
#ifdef MAMMOTH_VERBOSE
        std::cerr << name() << " running " << cam.serialise() << std::endl; 
#endif        
        static const arm::plane_angle_degrees_t max_pan = 45.0 * arm::degree;
        static const arm::plane_angle_degrees_t min_pan = -45.0 * arm::degree;
        static const arm::plane_angle_degrees_t max_tilt = 90.0 * arm::degree;
        static const arm::plane_angle_degrees_t min_tilt = -90.0 * arm::degree;
        static const arm::length_t min_height = 0.2 * arm::meter;
        static const arm::length_t max_height = 0.5 * arm::meter;
        
        
        if( cam.pan < min_pan ) { return result( "pan angle is below minimum limit of -45.0", result::error::invalid_input ); }
        if( cam.pan > max_pan ) { return result( "pan angle is above minimum limit of 45.0", result::error::invalid_input ); }
        if( cam.tilt < min_tilt ) { return result( "tilt angle is below minimum limit of -90.0", result::error::invalid_input ); }
        if( cam.tilt > max_tilt ) { return result( "tilt angle is above minimum limit of 90.0", result::error::invalid_input ); }
        if( cam.height < min_height ) { return result( "height value is below minimum limit of 0.1m", result::error::invalid_input ); }
        if( cam.height > max_height ) { return result( "height value is above minimum limit of 0.5m", result::error::invalid_input ); }
        
        static double zero_tilt = 90.0;
        Arm_Controller_U.motion_primitive = real_T( input_primitive::move_cam );
        Arm_Controller_U.Input_1 = cam.pan.value();
        Arm_Controller_U.Input_2 = zero_tilt - cam.tilt.value();
        Arm_Controller_U.Input_3 = cam.height.value();
        
        
        return result();
    }  
};

template < > struct action< arm::move_joints > {
    static result run( const arm::move_joints& joints )
    {
#ifdef MAMMOTH_VERBOSE
        std::cerr << name() << " running " << joints.serialise() << std::endl; 
#endif
        static const arm::plane_angle_degrees_t min = 0.0 * arm::degree;
        static const arm::plane_angle_degrees_t max = 360.0 * arm::degree;
        for( std::size_t i=0; i<joints.joints.size(); ++i )
        {
            if( joints.joints[i] < min || joints.joints[0] > max ) { return result( "joint angle must be 0-360 degrees", result::error::invalid_input ); }
        }
        
        Arm_Controller_U.motion_primitive = real_T( input_primitive::movej );
        Arm_Controller_U.Input_1 = joints.joints[0].value();
        Arm_Controller_U.Input_2 = joints.joints[1].value();
        Arm_Controller_U.Input_3 = joints.joints[2].value();
        Arm_Controller_U.Input_4 = joints.joints[3].value();
        Arm_Controller_U.Input_5 = joints.joints[4].value();
        Arm_Controller_U.Input_6 = joints.joints[5].value();
        
        
        
        return result();
    }  
};

template < > struct action< arm::set_position > {
    static result run( const arm::set_position& pos )
    {
        Arm_Controller_U.motion_primitive = input_primitive::set_position;
        
        Arm_Controller_U.Input_1 = pos.position == "giraffe" ? 
                arm::set_position::giraffe : arm::set_position::home;
                
        if( pos.position == "giraffe" ) { Arm_Controller_U.Input_1 = arm::set_position::giraffe; }
        else if( pos.position == "home" ) { Arm_Controller_U.Input_1 = arm::set_position::home; }
        else { return result("unknown position type", int(result::error::invalid_input) ); }
//         std::cerr << name() << " running " << pos.serialise()  << " pos_input: " << Arm_Controller_U.Input_1 
//             << " tag: " << pos.position << std::endl; 
        return result();
    }  
};

template < > struct action< arm::set_home > {
    static result run( const arm::set_home& h )
    {
        Arm_Controller_U.motion_primitive = input_primitive::set_home;
        return result();
    }  
};

template < typename C >
std::string handle( const std::vector< std::string >& line )
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
       
    // perform action
    result ret = action< C >::run( c );
    std::ostringstream ss;
    ss << '<' << c.serialise() << ',' << ret.get_message() << ';';
    return ss.str();
}

void process_command( const std::vector< std::string >& v )
{
    if( boost::iequals( v[2], "move_cam" ) )    { output( handle< arm::move_cam >( v ) ); }
    else if( boost::iequals( v[2], "set_pos" ) )  { output( handle< arm::set_position >( v ) ); }
    else if( boost::iequals( v[2], "set_home" ) )  { output( handle< arm::set_home >( v ) ); }
    else if( boost::iequals( v[2], "movej" ) )  { output( handle< arm::move_joints >( v ) ); }
    else { output( comma::join( v, v.size(), ',' ) + ',' + 
        impl_::str( arm::errors::unknown_command ) + ",\"unknown command found: '" + v[2] + "'\"" ); return; }
}

namespace ip = boost::asio::ip;
/// Connect to the TCP server within the allowed timeout
/// Needed because comma::io::iostream is not available
bool tcp_connect( const std::string& conn_str, 
                  const boost::posix_time::time_duration& timeout, ip::tcp::iostream& io )
{
    using boost::asio::ip::tcp;
    std::vector< std::string > v = comma::split( conn_str, ':' );
    boost::asio::io_service service;
    tcp::resolver resolver( service );
    tcp::resolver::query query( v[0] == "localhost" ? "127.0.0.1" : v[0], v[1] );
    tcp::resolver::iterator it = resolver.resolve( query );
    
    io.expires_from_now( timeout );
    io.connect( it->endpoint() );
    
    io.expires_at( boost::posix_time::pos_infin );
    
    return io.error() == 0;
} 

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
    arm_output output( acc * angular_acceleration_t::unit_type(), vel * angular_velocity_t::unit_type(),
                       Arm_Controller_Y );

    std::cerr << name() << "started" << std::endl;
    try
    {
        comma::uint16 rover_id = options.value< comma::uint16 >( "--id" );
        double sleep = 0.2; // seconds
        if( options.exists( "--sleep" ) ) { sleep = options.value< double >( "--sleep" ); };

        comma::uint32 listen_port = options.value< comma::uint32 >( "--status-port,-sp" );
        
        std::string arm_conn = options.value< std::string >( "--robot-arm" );
        tcp::iostream robot_arm;
        if( !tcp_connect( arm_conn, boost::posix_time::seconds(1), robot_arm ) ) 
        {
            std::cerr << name() << "failed to connect to robot arm at " 
                      << arm_conn << " - " << robot_arm.error().message() << std::endl;
            exit( 1 );
        }

        // create tcp server for broadcasting status
        std::ostringstream ss;
        ss << "tcp:" << listen_port;
        comma::io::publisher publisher( ss.str(), comma::io::mode::binary );

        arm::inputs inputs( rover_id );

        typedef std::vector< std::string > command_vector;

        typedef snark::robot_arm::current_positions current_positions_t;
        current_positions_t& current_positions = static_cast< current_positions_t& >( Arm_Controller_Y );
            
        const comma::uint32 usec( sleep * 1000000u );
        while( !signaled && std::cin.good() )
        {
            inputs.read();
            if( !inputs.is_empty() )
            {
                const command_vector& v = inputs.front();
                //std::cerr << name() << " got " << comma::join( v, ',' ) << std::endl;
                process_command( v );

                inputs.pop();
            }
            
            Arm_Controller_step();
            // We we need to send command to arm
            if( Arm_Controller_Y.command_flag > 0 )
            {
                std::cerr << name() << output.debug_in_degrees() << std::endl;
                robot_arm << output.serialise() << std::endl;
                robot_arm.flush();
                Arm_Controller_U.motion_primitive = real_T( input_primitive::no_action );
            }
            // reset inputs
            memset( &Arm_Controller_U, 0, sizeof( ExtU_Arm_Controller_T ) );
            // write a byte indicating the status, and joint positions
            static comma::csv::binary< current_positions_t > binary("","",true, current_positions );
            static std::vector<char> line( binary.format().size() );
            binary.put( current_positions, line.data() );
            publisher.write( line.data(), line.size());

            usleep( usec );
        }


        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
