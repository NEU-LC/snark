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

#ifndef SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H
#define SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <comma/base/types.h>
#include <comma/dispatch/dispatched.h>
#include <comma/io/stream.h>
#include <comma/io/select.h>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <boost/optional.hpp>
#include "data.h"
#include "commands.h"
#include "auto_initialization.h"
#include "tilt_sweep.h"
#include "waypoints_follower.h"
#include "output.h"
#include <boost/filesystem.hpp>

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

namespace arm = robotic_arm;
namespace fs = boost::filesystem;


class commands_handler : public comma::dispatch::handler_of< power >,
                                  public comma::dispatch::handler_of< brakes >,
                                  public comma::dispatch::handler_of< set_home >,
                                  public comma::dispatch::handler_of< auto_init >,
                                  public comma::dispatch::handler_of< move_cam >,
                                  public comma::dispatch::handler_of< move_joints >,
                                  public comma::dispatch::handler_of< joint_move >,
                                  public comma::dispatch::handler_of< set_position >,
                                  public comma::dispatch::handler_of< auto_init_force >,
                                  public comma::dispatch::handler_of< sweep_cam >,
                                  public comma::dispatch::handler_of< pan_tilt >,
                                  public comma::dispatch::handler_of< move_effector >
{
public:
    void handle( power& p );
    void handle( brakes& b );
    void handle( auto_init& a );
    void handle( move_cam& c );
    void handle( move_effector& e );
    void handle( move_joints& js );
    void handle( set_home& h );
    void handle( set_position& p );
    void handle( auto_init_force& p );
    void handle( joint_move& j );
    void handle( sweep_cam& s );
    void handle( pan_tilt& p );

    typedef boost::optional< waypoints_follower::recorder_setup_t > optional_recording_t;
    
    commands_handler( ExtU_Arm_controller_v2_T& simulink_inputs, const arm_output& output,
                      arm::status_t& status, std::ostream& robot, 
                      auto_initialization& init, tilt_sweep& sweep, waypoints_follower& follower, 
                      optional_recording_t recorder,
                      std::ostream& oss, const arm::continuum_t& config ) : 
        inputs_(simulink_inputs), output_(output), 
        status_( status ), os( robot ), 
        init_(init), sweep_( sweep ), waypoints_follower_( follower ),
        recorder_setup_( recorder ),
        ostream_( oss ), config_( config ),
        verbose_(true), is_move_effector( false ),
        home_filepath_( init_.home_filepath() ), lidar_filepath_( config_.work_directory + '/' + lidar_filename )
        {}
        
    bool is_initialising() const; 
    
    result ret;  /// Indicate if command succeed
private:
    ExtU_Arm_controller_v2_T& inputs_; /// inputs into simulink engine 
    const arm_output& output_;
    status_t& status_;
    std::ostream& os;
    auto_initialization& init_;
    tilt_sweep& sweep_;
    waypoints_follower& waypoints_follower_;
    optional_recording_t recorder_setup_;
    std::ostream& ostream_;
    arm::continuum_t config_;
    bool verbose_;
    boost::optional< length_t > move_cam_height_;
    plane_angle_degrees_t move_cam_pan_;
    bool is_move_effector;
    
    /// Run the command on the controller if possible
    bool execute();
    template < typename C >
    bool execute_waypoints( const C& c, bool record=false );
    /// Sets the current position of the arm into Simulink input structure
    void set_current_position();

    inline bool is_home_position() const { return status_.check_pose( config_.home_position ); } 

    /// resets inputs to noaction
    void inputs_reset();
public:
    static const char* lidar_filename;
    const fs::path home_filepath_;
    const fs::path lidar_filepath_;
};

} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {


#endif // SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H