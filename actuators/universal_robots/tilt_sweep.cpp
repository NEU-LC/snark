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

#include "tilt_sweep.h"
#include "output.h"
#include "traits.h"
#include <fstream>
#include <boost/thread.hpp>
#include <comma/math/compare.h>
#include <comma/io/publisher.h>
#include <comma/io/stream.h>
#include "../../sensors/hokuyo/traits.h"

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

const char* tilt_sweep::lidar_filename = "lidar.bin";

namespace impl_ {

void save_lidar( const std::string& conn_str, const boost::filesystem::path& savefile,
                 const std::string& fields, 
                 double range_limit )
{
    boost::filesystem::remove( savefile );

    namespace hok = snark::hokuyo;

    comma::csv::options csv;
    csv.fields = fields;
    csv.full_xpath = true;

    try
    {
        /// TODO use input stream
        comma::io::istream iss( conn_str, comma::io::mode::binary );
        comma::csv::binary_input_stream< hok::data_point > istream( *iss ); 
        //comma::io::ostream oss( savefile.string(), comma::io::mode::binary );
        std::ofstream oss( savefile.string().c_str(), std::ios::trunc | std::ios::binary | std::ios::out );
        if( !oss.is_open() ) { COMMA_THROW( comma::exception, "failed to open output file: " << savefile.string() );  }
        comma::csv::output_stream< hok::data_point > ostream( oss, csv ); 

        comma::uint32 count = 0;
        while( 1 )
        {
            ++count;
            if( count % 10 == 0 ) { boost::this_thread::interruption_point(); }

            const hok::data_point* point = istream.read();
            if( point == NULL ) { return; }

            if( point->range <= range_limit ) { ostream.write( *point ); }
        }
    }
    catch( boost::thread_interrupted& ti )
    {
        std::cerr << "save lidar interrupted." << std::endl;
    }
    catch( std::exception& e )
    {
        std::cerr  << "save_lidar exception: " << e.what() << std::endl;
    }
    catch(...)
    {
        std::cerr << "save_lidar unknown exception."<< std::endl;
    }
}
class holder
{
public:
    holder( boost::thread& thread ) : thread_( thread ), stopped_( false ) {}
    ~holder();
    /// Interrupt and wait for running thread to stop
    void stop();

private:
    boost::thread& thread_;
    bool stopped_;
};
void holder::stop()
{
    std::cerr << "interrupting save_lidar." << std::endl;
    thread_.interrupt();
    stopped_ = true;
    thread_.join();
    std::cerr << "save_lidar ended." << std::endl;
}
holder::~holder (){ 
    if( !stopped_ ) { stop(); } 
}

} // namespace impl_ {
    
// TODO how do you cancel an actioned item, stopj and run mode? or set to current angles
// Currently it sets to current joints angles, both work
// The other method requires to be a bit of a wait for mode change
void tilt_sweep::stop_movement(std::ostream& rover)
{
    static comma::csv::ascii< status_t::array_joint_angles_t > ascii;
    static std::string tmp;
    
    std::ostringstream ss;
    status_update_();
    ss << "movej([" << ascii.put( status_.joint_angles, tmp ) 
       << "],a=" << serialiser_.acceleration().value() << ','
       << "v=" << serialiser_.velocity().value() << ')';
    const std::string stop_str = ss.str();
    rover << stop_str << std::endl;
    rover.flush();
}

result tilt_sweep::run( const length_t& height, const plane_angle_degrees_t& pan, 
                          started_reply_t start_initiated,
                          std::ostream& rover )
{
    return run( height, pan, min_, max_, start_initiated, rover );
}
    
    
result tilt_sweep::run( const length_t& height, const plane_angle_degrees_t& pan, 
                          const plane_angle_degrees_t& tilt_down, const plane_angle_degrees_t& tilt_up, 
                          started_reply_t start_initiated,
                          std::ostream& rover )
{
    
    std::cerr << name() << "scanning from " << tilt_up.value() << "\" to " << tilt_down.value() << '"' << std::endl;
    move_t move1, move2, ret;
    if( !calculate_solution( height, pan, tilt_down, tilt_up, move1, move2, ret ) )
    {
        return result( "cannot perform the proposed camera sweep because of collision", result::error::failure );
    }
    
    bool stop = interrupt_();
    if( signaled_ || stop ) { return result( "camera sweep action is cancelled", result::error::cancelled ); }
    
    /// signal start of command
    start_initiated();

    /// Starting recording thread.
    std::cerr << "starting to save lidar data into " << lidar_filepath_.string() << std::endl;
    boost::thread recorder( boost::bind( &impl_::save_lidar, "tcp:localhost:9003", lidar_filepath_, "x,y,z,intensity", 3.0 ) );

    impl_::holder thread_started( recorder );
    
    std::cerr << name() << "running action 1: " << move1.action << " target angle: " << move1.tilt.value() << std::endl;
    rover << move1.action << std::endl;
    rover.flush();
    
    /// Check that it stopped
    static comma::uint32 usec = 0.1 * 1000000u;
    
    static const arm::plane_angle_t epsilon = static_cast< arm::plane_angle_t >( 0.5 * arm::degree );
    while( !comma::math::equal( status_.joint_angles[ tilt_joint ], move1.tilt, epsilon  ) )
    {
        status_update_();
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    std::cerr << name() << "running action 2: " << move2.action << " target angle:" << move2.tilt.value() << std::endl;
    rover << move2.action << std::endl;
    rover.flush();
    while( !comma::math::equal( status_.joint_angles[ tilt_joint ], move2.tilt, epsilon  ) )
    {
        status_update_();
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    // stop recording lidar data now
    thread_started.stop();
    
    std::cerr << name() << "returning to position: " << ret.action << " target angle:" << ret.tilt.value() << std::endl;
    rover << ret.action << std::endl;
    rover.flush();
    while( !comma::math::equal( status_.joint_angles[ tilt_joint ], ret.tilt, epsilon  ) )
    {
        status_update_();
        stop = interrupt_();
        if( signaled_ || stop ) { stop_movement( rover ); return result( "camera sweep action is cancelled", result::error::cancelled ); }
        usleep( usec );
    }
    
    return result();
}

bool tilt_sweep::calculate_solution( const length_t& height, const plane_angle_degrees_t& pan, 
                                       const plane_angle_degrees_t& tilt_down, const plane_angle_degrees_t& tilt_up, 
                                       move_t& move1, move_t& move2, move_t& ret )
{
    const plane_angle_degrees_t current_tilt = static_cast< plane_angle_degrees_t >( status_.position.orientation.y() * radian ); // This is the pitch
    std::cerr << name() << "current tilt angle is " << current_tilt.value() << '"' << std::endl;
    
    inputs_reset();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = pan.value();
    inputs_.Input_2 = tilt_up.value();
    inputs_.Input_3 = height.value();
    Arm_controller_v2_step();
    
    if( !serialiser_.runnable() ) { std::cerr << name() << "failed to find move action 1, is collision: " << serialiser_.will_collide() << std::endl; return false; }
    
    /// Get commands
    move1 = move_t( serialiser_.serialise(), serialiser_.proposed_tilt_angle() );
    
    inputs_reset();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = pan.value();
    inputs_.Input_2 = tilt_down.value();
    inputs_.Input_3 = height.value();
    Arm_controller_v2_step();
    
    if( !serialiser_.runnable() ) { std::cerr << name() << "failed to find move action 2, will_collide: " << serialiser_.will_collide() << std::endl; return false; }
    
    /// Get commands
    move2 = move_t( serialiser_.serialise(), serialiser_.proposed_tilt_angle() );
    
    // return to former position
    inputs_reset();
    
    inputs_.motion_primitive = real_T( input_primitive::move_cam );
    inputs_.Input_1 = pan.value();
    inputs_.Input_2 = current_tilt.value();
    inputs_.Input_3 = height.value();
    Arm_controller_v2_step();
    
    if( !serialiser_.runnable() ) { std::cerr << name() << "failed to find move action 3, will_collide:" << serialiser_.will_collide() << std::endl; return false; }
    
    /// Get commands
    ret = move_t( serialiser_.serialise(), serialiser_.proposed_tilt_angle() );
    
    /// Done
    inputs_.motion_primitive = input_primitive::no_action;
    
    return true;
}

    
} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {
    