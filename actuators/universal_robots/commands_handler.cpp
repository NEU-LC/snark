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

#include "commands_handler.h"
#include <fstream>
#include <boost/bind.hpp>
#include "auto_initialization.h"
#include "traits.h"

namespace snark { namespace ur { namespace handlers {

static const char* name() { return "robot-arm-daemon: "; }

void commands_handler::handle( snark::ur::power& p )
{
    if( p.is_on && !status_.is_powered_off() ) {
        ret = result( "cannot execute power on command as current state is not 'no_power'", result::error::invalid_robot_state );
        return;
    }

    std::cerr << name() << "powering robot arm " << ( p.is_on ? "on" : "off" ) << std::endl;
    os << "power " << ( p.is_on ? "on" : "off" ) << std::endl;
    os.flush();
    ret = result();
}

void commands_handler::handle( snark::ur::brakes& b )
{
    std::cerr << name() << "running brakes: " << b.enable  << std::endl;
    if( !b.enable ) {
        os << "set robotmode run" <<std::endl;
    }
    else {
        os << "stopj([0.1,0.1,0.1,0.1,0.1,0.1])" << std::endl;
    }
    os.flush();
    ret = result();
}

void commands_handler::handle( snark::ur::joint_move& joint )
{
    if( !is_initialising() ) {
        ret = result( "cannot initialise joint as robot is not in initialisation mode", result::error::invalid_robot_state );
        return;
    }
    
    /// command can be use if in running or initialising mode
    int index = joint.joint_id;
    if( !( ( status_.robot_mode == robotmode::initializing || status_.robot_mode == robotmode::running )  && 
           ( status_.joint_modes[index] == jointmode::initializing || status_.joint_modes[index] == jointmode::running ) ) ) 
    { 
        std::ostringstream ss;
        ss << "robot and  joint (" << index << ") must be initializing or running state. However current robot mode is '" 
           << status_.mode_str() << "' and joint mode is '" << status_.jmode_str(index)  << '\'' << std::endl;
        ret = result( ss.str(), result::error::invalid_robot_state );
        return; 
    }

    static const unsigned char min_id = 0;
    static const unsigned char max_id = 5;
    std::cerr << name() << "move joint: " << int(joint.joint_id) << " dir: " << joint.dir << std::endl;
    static const angular_velocity_t velocity = 0.1 * rad_per_sec;
    static const angular_acceleration_t acceleration = 0.05 * rad_per_s2;
    static const boost::posix_time::time_duration duration = boost::posix_time::milliseconds( 20 );
    
    if( joint.joint_id < min_id || joint.joint_id > max_id ) {
        ret = result( "joint id must be 0-5", result::error::invalid_input );
        return;
    }
    
    double vel = ( joint.dir ? velocity.value() : -velocity.value() );
    if( joint.joint_id == 2 ) { vel /= 2; }
    else if( joint.joint_id == 1 ) { vel /= 2.5; }
    else if( joint.joint_id < 1 ) { vel /= 3; }
    
    std::ostringstream ss;
    ss << "speedj_init([";
    for( std::size_t i=min_id; i<=max_id; ++i )
    {
        ss << (i == joint.joint_id ? vel : 0);
        if( i != max_id ) { ss << ','; };
    }
    ss << "],"  << acceleration.value() << ',' << (duration.total_milliseconds()/1000.0) << ')' << std::endl;
    os << ss.str();
    os.flush(); 
    
    boost::filesystem::remove( home_filepath_ );
    
    ret = result();
}

template < typename C >
void movement_started( const C& c, std::ostream& oss )
{
    static comma::csv::ascii< C > ascii;
    static std::string tmp;
    oss << '<' << c.serialise() << ',' << result::error::action_started << ',' << "\"movement initiated\";" << std::endl;
    oss.flush();
}

template < typename C >
bool commands_handler::execute_waypoints( const C& command, bool record )
{
    Arm_controller_v2_step();
    if( !output_.runnable() ) { 

        if( output_.will_collide() ) {
            ret = result( "cannot run command as it will cause a collision", result::error::collision ); inputs_reset(); 
        }
        else {
            ret = result( "proposed action is not possible", result::error::invalid_input );
        }
        return false; 
    }
    
    if( verbose_ ) { 
        std::cerr << name() << output_.debug_in_degrees() << std::endl; 
        std::cerr << name() << output_.serialise() << std::endl; 
    }
    // Ok now follow the waypoints
    if( record ) { ret = waypoints_follower_.run( boost::bind( movement_started< C >, boost::cref( command ), boost::ref( this->ostream_ ) ) , this->os, recorder_setup_ ); }
    else { ret = waypoints_follower_.run( boost::bind( movement_started< C >, boost::cref( command ), boost::ref( this->ostream_ ) ) , this->os ); }

    inputs_reset();
    return ret.is_success();
}

bool commands_handler::is_initialising() const 
{
    if( status_.robot_mode != robotmode::initializing ) { return false; }
    for( std::size_t i=0; i<joints_num; ++i ) 
    {
        if( status_.jmode(i) != jointmode::initializing && 
            status_.jmode(i) != jointmode::running ) { 
            return false; 
        }
    }
    return true;
}

void commands_handler::handle( snark::ur::auto_init& a )
{
    ret = init_.run( boost::bind( movement_started< auto_init >, boost::cref( a ), boost::ref( this->ostream_ ) ), false );
}

void commands_handler::handle( snark::ur::auto_init_force& init )
{
    ret = init_.run( boost::bind( movement_started< auto_init_force >, boost::cref( init ), boost::ref( this->ostream_ ) ), init.force );
}

} } } // namespace snark { namespace ur { namespace handlers {
