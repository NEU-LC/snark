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

template < typename C >
void movement_started( const C& c, std::ostream& oss )
{
    static comma::csv::ascii< C > ascii;
    static std::string tmp;
    oss << '<' << c.serialise() << ',' << result::error::action_started << ',' << "\"movement initiated\";" << std::endl;
    oss.flush();
}

void commands_handler::handle( snark::ur::move_joints& joints )
{
    ret = result();
}

void commands_handler::handle( snark::ur::joint_move& joint )
{
    if( !is_initialising() ) {
        ret = result( "cannot initialise joint as robot is not in initialisation mode", result::error::invalid_robot_state );
        return;
    }
    move_cam_height_.reset(); // no longer in move_cam position
    
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
    if( record ) {
        ret = waypoints_follower_.run(
                boost::bind( movement_started< C >, boost::cref( command ), boost::ref( this->ostream_ ) )
                , this->os, recorder_setup_ );    
    }
    else {
        ret = waypoints_follower_.run(
                boost::bind( movement_started< C >, boost::cref( command ), boost::ref( this->ostream_ ) )
                , this->os );    
    }

    inputs_reset();
    return ret.is_success();
}

void commands_handler::handle( snark::ur::set_home& h )
{
    inputs_reset();
    set_current_position();
    inputs_.motion_primitive = input_primitive::set_home;
    ret = result();
}

void commands_handler::handle( snark::ur::set_position& pos )
{
    if( !status_.is_running() ) {
        ret = result( "cannot set position as robot is not in running mode", result::error::invalid_robot_state );
        return;
    }
    
    inputs_reset();
    set_current_position();
    inputs_.motion_primitive = input_primitive::set_position;

    if( pos.position == "giraffe" ) { inputs_.Input_1 = snark::ur::set_position::giraffe; boost::filesystem::remove( home_filepath_ ); }
    else if( pos.position == "home" ) { inputs_.Input_1 = snark::ur::set_position::home; }
    else { ret = result("unknown position type", int(result::error::invalid_input) ); return; }

    inputs_.Input_2 = 0;    // zero pan for giraffe
    inputs_.Input_3 = 0;    // zero tilt for giraffe
    
    if( execute_waypoints( pos ) )
    {
        is_move_effector = false;
        move_cam_height_.reset(); // no longer in move_cam position
    }
}

void commands_handler::handle( snark::ur::move_effector& e )
{    
    std::cerr << name() << "handling move_effector" << std::endl; 
    if( !status_.is_running() ) { ret = result( "cannot move effector as robot is not in running mode", result::error::invalid_robot_state ); return; }

    // First run simulink to calculate the waypoints
    inputs_reset();
    set_current_position();
    inputs_.motion_primitive = input_primitive::move_effector;
    inputs_.Input_1 = e.offset.x();
    inputs_.Input_2 = e.offset.y();
    inputs_.Input_3 = e.offset.z();
    std::cerr << "joint 2 is " << status_.joint_angles[2].value() << std::endl;

    if( execute_waypoints( e ) )
    {
        is_move_effector = true;
        move_cam_height_.reset();
    }
}

bool commands_handler::is_initialising() const 
{
    if( status_.robot_mode != robotmode::initializing ) { 
        // std::cerr << "robot mode " << status_.robot_mode << " expected: " << robotmode::initializing << std::endl;
        return false; 
    }

    for( std::size_t i=0; i<joints_num; ++i ) 
    {
        // std::cerr << "joint " << i << " mode " << status_.joint_modes[i] << " expected: " << jointmode::running << std::endl;
        if( status_.jmode(i) != jointmode::initializing && 
            status_.jmode(i) != jointmode::running ) { 
            return false; 
        }
    }

    return true;
}

void commands_handler::handle( snark::ur::auto_init& a )
{
    ret = init_.run( boost::bind( movement_started< auto_init >, boost::cref( a ), boost::ref( this->ostream_ ) ),
                     false );
    if( ret.is_success() ) { 
        std::cerr << name() << "going to home position." << std::endl;
        snark::ur::set_position home; 
        handle( home ); 
    } // set to home

}

void commands_handler::handle( snark::ur::auto_init_force& init )
{
    ret = init_.run( boost::bind( movement_started< auto_init_force >, boost::cref( init ), boost::ref( this->ostream_ ) ), 
                     init.force );
    if( ret.is_success() ) { 
        std::cerr << name() << "going to home position." << std::endl;
        snark::ur::set_position home; 
        handle( home ); 
    } // set to home
}

void commands_handler::inputs_reset() 
{ 
    inputs_.motion_primitive = input_primitive::no_action;
    inputs_.Input_1 = 0;
    inputs_.Input_2 = 0;
    inputs_.Input_3 = 0;
    inputs_.Input_4 = 0;
    inputs_.Input_5 = 0;
    inputs_.Input_6 = 0;
}

/// The Simulink code needs to know the current position of the arm
/// Sets it after reading the position
void commands_handler::set_current_position( )
{
    // std::cerr << name() << "setting joints, joint 2: " << status.joint_angles[2].value() << std::endl; 
    inputs_.Joint1 = status_.joint_angles[0].value();
    inputs_.Joint2 = status_.joint_angles[1].value();
    inputs_.Joint3 = status_.joint_angles[2].value();
    inputs_.Joint4 = status_.joint_angles[3].value();
    inputs_.Joint5 = status_.joint_angles[4].value();
    inputs_.Joint6 = status_.joint_angles[5].value();
}

} } } // namespace snark { namespace ur { namespace handlers {
