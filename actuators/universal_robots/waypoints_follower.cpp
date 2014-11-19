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

#include "waypoints_follower.h"
#include "output.h"
#include "traits.h"
#include <comma/math/compare.h>

namespace snark { namespace ur { namespace handlers {

namespace impl_ { 

class thread_release
{
public:
    void operator()( boost::thread* thread )
    {
        std::cerr << "interrupting save_lidar." << std::endl;
        thread->interrupt();
        thread->join();
        std::cerr << "save_lidar ended." << std::endl;
        delete thread;
    }

private:
};

} // namespace impl_ { 
    
// Currently it sets to current joints angles, both work
// The other method requires to be a bit of a wait for mode change
void waypoints_follower::stop_movement(std::ostream& robot)
{
    static comma::csv::ascii< status_t::array_joint_angles_t > ascii;
    static std::string tmp;
    
    std::ostringstream ss;
    status_update_();
    ss << "movej([" << ascii.put( status_.joint_angles, tmp ) 
       << "],a=" << serialiser_.acceleration().value() << ','
       << "v=" << serialiser_.velocity().value() << ')';
    const std::string stop_str = ss.str();
    robot << stop_str << std::endl;
    robot.flush();
}

result waypoints_follower::run( started_reply_t start_initiated, std::ostream& robot, const boost::optional< recorder_setup_t >& record_info )
{
    comma::uint32 num_of_moves = serialiser_.num_of_moves();
    if( num_of_moves > joints_num ) { 
        std::cerr << name() << "too many waypoints, got " << num_of_moves << std::endl;
        return result( "Simulink calculation error, too many waypoints.", result::error::failure ); 
    }

    bool stop = interrupt_();
    if( signaled_ || stop ) { return result( "action is cancelled", result::error::cancelled ); }
    
    /// Check that it stopped
    static comma::uint32 usec = 0.1 * 1000000u;

    /// signal start of command
    start_initiated();
    /// If recording is needed, and always release the thread regardless of function exit point
    boost::shared_ptr< boost::thread > recorder_container;
    
    std::cerr << "num_of_moves is " << num_of_moves << std::endl;

    static const plane_angle_degrees_t final_epsilon = 0.3*degree;
    static const plane_angle_degrees_t epsilon = 0.5*degree;
    for( std::size_t j=0; j< num_of_moves; ++j )
    {
        std::cerr << name() << "moving to waypoint " << (j+1) << std::endl;
        std::cerr << name() << serialiser_.serialise( j ) << std::endl;

        /// If recording, starts it and use the specified velocity
        std::string arm_command = serialiser_.serialise( j );
        if( record_info )
        {
            if( record_info->start_waypoint_ == j ) 
            { 
                arm_command = serialiser_.serialise( record_info->velocity_, j );
                recorder_container.reset( new boost::thread( record_info->recorder_ ), impl_::thread_release() ); 
            }
            if( record_info->end_waypoint_ == j ) { recorder_container.reset(); }
        }

        robot << arm_command << std::endl;
        robot.flush();

        const snark::ur::move_config_t& config = serialiser_.get_move_config( j );
        while( !status_.check_pose( config, ( j == (num_of_moves - 1 ) ? final_epsilon : epsilon )  ) )  // Only check for intermediate waypoints
        {
            // std::cerr << "not yet at pose" << std::endl;
            status_update_();
            stop = interrupt_();
            if( signaled_ || stop ) { stop_movement( robot ); return result( "action is cancelled", result::error::cancelled ); }
            usleep( usec );
        }
    }
    return result();
}
    
} } } // namespace snark { namespace ur { namespace handlers {
    