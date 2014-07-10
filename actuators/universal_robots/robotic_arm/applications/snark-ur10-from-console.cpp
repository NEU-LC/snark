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
#include "../traits.h"
#include "../data.h"
#include "../units.h"

const char* name() { return "snark-ur10-from-console: "; }

namespace impl_ {

template < typename T >
std::string str(T t) { return boost::lexical_cast< std::string > ( t ); }
    
} // namespace impl_ {

namespace arm = snark::robot_arm;
typedef arm::fixed_status status_t;

void usage(int code=1)
{
    std::cerr << std::endl;
    std::cerr << name() << std::endl;
    std::cerr << "example: socat -u STDIN,raw STDOUT | snark-ur10-from-console --host robot-arm --status-port 30003 | socat -u - tcp:robot-arm:30002 " << name() << " " << std::endl;
    std::cerr << "         The program expects raw binary input hence 'STDIN,raw', when used in ascii mode a CR must be hit after each keypress." << std::endl;
    std::cerr << "Interactive:" << std::endl;
    std::cerr << "    Input char 0-5 for switching joint to initialise, defaults to joint index 5 (6th joint) on startup." << std::endl;
    std::cerr << "    Press/hold char k for positive velocity." << std::endl;
    std::cerr << "    Press/hold char j for negative velocity." << std::endl;
    std::cerr << "    Press/hold ' ' or any other key to stop any movement." << std::endl;
    std::cerr << "    A stop is sent when application closes." << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h:            show this message" << std::endl;
    std::cerr << "    --sleep:              Loop sleep in seconds, defaults to 0.02s, this should ALWAYS be smaller than --time-step." << std::endl;
    std::cerr << "    --host:               Robot arm's hostname or IP." << std::endl;
    std::cerr << "    --port,-p:            Robot arm's port for fixed size binary status, defaults is 30003." << std::endl;
    std::cerr << "    --versbose,-v:        show messages to the robot arm - angles are changed to degrees." << std::endl;
    std::cerr << "    --velocity,-v:        Radian per second velocity for movement." << std::endl;
    std::cerr << "    --acceleration,-a:    Radian per second squared acceleration for movement." << std::endl;
    std::cerr << "    --time-step,-s:       Seconds e.g. 0.1 for 100ms, how long to move arm per key press." << std::endl;
    exit ( code );
}


template < typename T > 
comma::csv::ascii< T >& ascii( )
{
    static comma::csv::ascii< T > ascii_;
    return ascii_;
}

std::ostream& ostream = std::cout;

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
        velocity_( vel ), acceleration_( acc ), time_( duration ) {}
    ~current_joint() { stop(); }
    
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
        
        current_joint::data data( current_, sign*velocity_.value() );
        static std::string line;
        ostream << "speedj_init([" << ascii< current_joint::data >().put( data, line )
                << "]," << acceleration_.value() << ',' << (time_.total_milliseconds()/1000.0) << ')' << std::endl;
        ostream.flush();
    }
    
    void status() const
    {
        std::cerr << name() << "Initialising joint (" << int(current_) << ')' << std::endl; 
        std::cerr.flush();
    }
    void stop()
    {
        static std::string stop = "speedj_init([0,0,0,0,0,0],0,0)";
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
    
    
    if( options.exists("-v,--velocity") ) { velocity = options.value< double >("-v,--velocity") * arm::rad_per_sec;  }
    if( options.exists("-a,--acceleration") ) { acceleration = options.value< double >("-a,--acceleration") * arm::rad_per_s2;  }
    if( options.exists("-s,--time-step") ) { duration_step = boost::posix_time::millisec(  std::size_t(options.value< double >("-s,--time-step") * 1000u) );  }
    
    if( usec > duration_step.total_microseconds() ) { COMMA_THROW( comma::exception, "--sleep must be smaller or equals to --time-step" ); }
    
    std::cerr << name() << "duration step: " << duration_step.total_milliseconds() << "ms" << std::endl;
    std::cerr << name() << "velocity: " << velocity.value() << "rad/ms"<< std::endl;
    std::cerr << name() << "acceleration: " << acceleration.value() << "rad/ms^2"<< std::endl;

    std::cerr << name() << "started" << std::endl;
    std::cerr.flush();
    
    comma::io::select select;
    select.read().add( 0 );
    
    try
    {
        current_joint joint( start_joint, velocity, acceleration, duration_step );
        while( !signaled && std::cin.good() )
        {
            select.check();
            
            if( select.read().ready( 0 ) )
            {
                char c;
                std::cin.read( &c, 1u );
                if( std::cin.gcount() != 1 ) { break; }
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
