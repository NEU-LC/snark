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
#include <boost/asio.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/pthread/mutex.hpp>
#include <comma/io/stream.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/base/types.h>
#include <comma/visiting/apply.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/parser.h>
#include <comma/io/stream.h>
#include <comma/io/publisher.h>
#include <comma/csv/stream.h>
#include <comma/csv/binary.h>
#include <comma/string/string.h>
#include <comma/application/signal_flag.h>
#include "../../traits.h"
#include "../../data.h"
#include "../../units.h"

const char* name() { return "snark-ur10-from-console: "; }

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
} // namespace impl_ {

namespace arm = snark::ur::robotic_arm;
typedef arm::fixed_status status_t;

void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << " Reads keyboard input to command the initialisation of robotic arm's joints - one at a time." << std::endl;
    std::cerr << "         Produces robot arm commands to move the joint a small angle, feed the output to robot arm's connection on port 30002." << std::endl;
    std::cerr << "example: io-console | snark-ur10-from-console --feedback-host robot-arm --feedback-port 30003 | socat -u - tcp:robot-arm:30002 " << name() << " " << std::endl;
    std::cerr << "         io-console can alternatively be replaced with 'socat -u STDIN,raw,escape=81 STDOUT'." << std::endl;
    std::cerr << "Interactive:" << std::endl;
    std::cerr << "    Input char 0-5 for switching joint to initialise, defaults to joint index 5 (6th joint) on startup." << std::endl;
    std::cerr << "    Press/hold char k for positive velocity." << std::endl;
    std::cerr << "    Press/hold char j for negative velocity." << std::endl;
    std::cerr << "    Press/hold ' ' or any other key to stop any movement." << std::endl;
    std::cerr << "    A stop is sent when application closes." << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:            show this message" << std::endl;
    std::cerr << "*   --feedback-host:      Hostname for real time feedback of robotic-arm's status." << std::endl;
    std::cerr << "    --feedback-port:      Port for real time feed back of robotic-arm's status, defaults is 30003." << std::endl;
    std::cerr << "    --sleep:              Loop sleep in seconds, defaults to 0.02s, this should ALWAYS be smaller than --time-step." << std::endl;
    std::cerr << "    --versbose,-v:        show messages to the robot arm - angles are changed to degrees." << std::endl;
    std::cerr << "    --velocity,-v:        Radian per second velocity for movement." << std::endl;
    std::cerr << "    --acceleration,-a:    Radian per second squared acceleration for movement." << std::endl;
    std::cerr << "    --time-step,-s:       Seconds e.g. 0.1 for 100ms, how long to move arm per key press." << std::endl;
    std::cerr << "    --no-exit:            Allows moving each joint when the arm is already initialised." << std::endl;
    exit ( code );
}


template < typename T > 
comma::csv::ascii< T >& ascii( )
{
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

std::ostream& ostream = std::cout;

struct is_in_initialise
{
    bool operator()( const arm::jointmode::mode& state ) { return state == arm::jointmode::initializing; }
};

struct is_not_in_running
{
    bool operator()( const arm::jointmode::mode& state ) { return state != arm::jointmode::running; }
};

/// Stores the current joint being initialised, it handles key press to move the single joint.
class current_joint
{
    char current_;
    arm::angular_velocity_t velocity_;          // velocity for every key press
    arm::angular_acceleration_t acceleration_;  // accelaration for every key press
    boost::posix_time::time_duration time_;           // how long to move arm per key press
    static const char min = 0;
    static const char max = 5;
    
    struct data {
        data( ) : speeds( max+1, 0.0 ) {}
        data( char joint_index, double vel ) : speeds( max+1, 0.0 )
        {
            speeds[ joint_index ] = vel;
        }
        std::vector< double > speeds;
    };
public:
    current_joint( char num, 
                   const arm::angular_velocity_t& vel, 
                   const arm::angular_acceleration_t& acc, 
                   const boost::posix_time::time_duration& duration=boost::posix_time::seconds(0.1) ) : current_( num ), 
        velocity_( vel ), acceleration_( acc ), time_( duration ) {
            status();
        }
    ~current_joint() { stop(); }
    
    char index() const { return current_; }
    void set_current( char id )
    {
        if( id > (arm::joints_num-1) || id < 0 ) { COMMA_THROW( comma::exception, "joint index out of range" ); }
        current_ = id;
        
        status();
    }
    
    void handle( char c )
    {
        double sign=1;
        switch( c )
        {
            case 'k':
            case char(39):
                break;
            case 'j':
            case char(37):
                sign = -1;
                break;
            case '0':
            case '1':
            case '2':
            case '3':
            case '4':
            case '5':
                current_ = boost::lexical_cast< comma::uint16 >( c );
                // TODO: check if joint is in init mode
                status();
                return;
            case ' ':    
            default:
                stop();
                return;
        }
        
        double vel = sign*velocity_.value();
        if( current_ <= 2 ) { vel /= 2.0; }
        current_joint::data data( current_, vel );
        static std::string line;

        double time = (time_.total_milliseconds()/1000.0) * ( (current_+1)/3.0 ); // scale time from 1/3 to 2.0 for joint 0-5
        ostream << "speedj_init([" << ascii< current_joint::data >().put( data, line )
                << "]," << acceleration_.value() << ',' << time << ')' << std::endl;
        ostream.flush();
    }
    
    void status() const
    {
        std::cerr << name() << "Initialising joint (" << int(current_) << ')' << std::endl; 
        std::cerr.flush();
    }
    void stop()
    {
        static std::string stop = "speedj_init([0,0,0,0,0,0],0.1,0.1)";
        ostream << stop << std::endl;
        ostream.flush();
        std::cerr << name() << "sent stop command." << std::endl;
        std::cerr.flush();
    }
};

namespace comma { namespace visiting {
    
template < > struct traits< current_joint::data >
{
    template< typename K, typename V > static void visit( const K& k, const current_joint::data& t, V& v )
    {
        v.apply( "speeds", t.speeds );
    }
};

} } //namespace comma { namespace visiting {

void get_status( arm::status_t& state )
{
    static comma::csv::binary_input_stream< arm::status_t > iss( std::cin );
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
    
    const char start_joint = 5;
    arm::angular_acceleration_t acceleration = 0.05 * arm::rad_per_s2;
    arm::angular_velocity_t velocity = 0.1 * arm::rad_per_sec;
    boost::posix_time::millisec duration_step( 20u );
    
    double sleep = 0.01;
    if( options.exists("--sleep") ) { sleep = options.value< double >("--sleep");  }
    const comma::int64 usec = 1000000u * sleep;

    bool allow_in_run_mode = options.exists( "--no-exit" );
    
    if( options.exists("-v,--velocity") ) { velocity = options.value< double >("-v,--velocity") * arm::rad_per_sec;  }
    if( options.exists("-a,--acceleration") ) { acceleration = options.value< double >("-a,--acceleration") * arm::rad_per_s2;  }
    if( options.exists("-s,--time-step") ) { duration_step = boost::posix_time::millisec(  std::size_t(options.value< double >("-s,--time-step") * 1000u) );  }
    
    std::string feedback_host = options.value< std::string >( "--feedback-host" );
    std::string feedback_port = options.value< std::string >( "--feedback-port", "30003" );
    
    std::string status_conn = "tcp:" + feedback_host + ':' + feedback_port;
    std::cerr << name() << "status connection to feedback status: " << status_conn << std::endl;
    comma::io::istream status_stream( status_conn, comma::io::mode::binary );
    comma::io::select select;
    select.read().add( status_stream.fd() );
   
    if( usec > duration_step.total_microseconds() ) { COMMA_THROW( comma::exception, "--sleep must be smaller or equals to --time-step" ); }
    
    std::cerr << name() << "duration step: " << duration_step.total_milliseconds() << "ms" << std::endl;
    std::cerr << name() << "velocity: " << velocity.value() << "rad/ms"<< std::endl;
    std::cerr << name() << "acceleration: " << acceleration.value() << "rad/ms^2"<< std::endl;

    std::cerr << name() << "started" << std::endl;
    std::cerr.flush();
    
    // Listens for commands.
    select.read().add( 0 );
    
    // arm's status
    arm::fixed_status arm_status; 
    arm::status_t state;

    usleep( 0.5 * 1000000u );
    
    try
    {
        comma::csv::options csv_in;
        csv_in.full_xpath = true;
        csv_in.format( comma::csv::format::value< arm::status_t >( "", true ) );
        comma::csv::binary_input_stream< arm::status_t > iss( *status_stream, csv_in );
        
        current_joint joint( start_joint, velocity, acceleration, duration_step );
        while( !signaled && std::cin.good() )
        {
            select.check();
            // If we have status data, read till the latest data
            if( select.read().ready( status_stream.fd() ) )
            {
                state = *(iss.read());
                if( !status_stream->good() ) { COMMA_THROW( comma::exception, "failure on connection/read for robotic arm's status" ); } 
                while( status_stream->rdbuf()->in_avail() > 0 )
                {
                    state = *(iss.read());
                    if( !status_stream->good() ) { COMMA_THROW( comma::exception, "failure on connection/read for robotic arm's status" ); } 
                }
            }

            int joint_inited = -1;
            // find first joint in initialization state, descending order joint 5-0
            if( allow_in_run_mode && state.joint_modes[ joint.index() ] != arm::jointmode::running ) {}
            else if( state.joint_modes[ joint.index() ] != arm::jointmode::initializing ) 
            {
                typedef arm::status_t::array_jointmodes_t::const_reverse_iterator reverse_iter;
                typedef arm::status_t::array_jointmodes_t::const_iterator const_iter;
                reverse_iter iter = std::find_if( state.joint_modes.crbegin(), 
                                                  state.joint_modes.crend(), is_in_initialise() );
                if( iter == state.joint_modes.crend() ) // no more in initialize state
                {
                    const_iter irun = std::find_if( state.joint_modes.cbegin(), 
                                                    state.joint_modes.cend(), is_not_in_running() );
                    
                    if( irun == state.joint_modes.cend() ) 
                    { 
                        std::cerr << name() << "finished - initialisation completed for all joints." << std::endl;  
                        return 0; 
                    } // all initialised 
                    else 
                    { 
                        std::cerr << name() << "error - initialisation completed with joint/s not in running state, joint: "
                                  << ( irun - state.joint_modes.cbegin() ) << " mode: " << arm::jointmode_str(*irun) << std::endl; 
                        // boost::property_tree::ptree t;
                        // comma::to_ptree to_ptree( t );
                        // comma::visiting::apply( to_ptree ).to( state );
                        // boost::property_tree::write_json( std::cerr, t, false );    
                        return 1; 
                    }
                }
                else 
                { 
                    /// This works but is a bit dangerous as it move to next joint but the user pressed many movement for previous joint
                    // go to next joint in initializing state
                    //joint.set_current ( std::distance(iter, state.joint_modes.crend()) - 1 ); 

                    // This alternative way means you are moving the same joint.
                    joint_inited = std::distance(iter, state.joint_modes.crend()) - 1;
                } 
            }
            
            // std::cerr << name() << "waiting for input" << std::endl;
            if( select.read().ready( 0 ) )
            {
                char c;
                std::cin.read( &c, 1u );
                if( std::cin.gcount() != 1 ) { break; }

                if( joint_inited >= 0 )
                {
                    switch( c )
                    {
                        case '0':
                        case '1':
                        case '2':
                        case '3':
                        case '4':
                        case '5':
                            break;
                        default:
                        {
                            std::cerr << name() << "joint " << int( joint.index() ) 
                                << " is initialised, please change to joint id (" 
                                << joint_inited << ')' << std::endl;
                            break;
                        }
                    }
                }

                joint.handle( c );
            }

            // This is to not flood the robot arm controller.
            usleep( usec );
        }
        
    }
    catch( comma::exception& ce ) { std::cerr << name() << ": exception thrown: " << ce.what() << std::endl; return 1; }
    catch( std::exception& e ) { std::cerr << name() << ": unknown exception caught: " << e.what() << std::endl; return 1; }
    catch(...) { std::cerr << name() << ": unknown exception." << std::endl; return 1; }
    
}
