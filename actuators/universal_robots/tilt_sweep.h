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

#ifndef SNARKS_ACTUATORS_UR_ROBOTIC_ARM_CAMERA_SWEEP_H
#define SNARKS_ACTUATORS_UR_ROBOTIC_ARM_CAMERA_SWEEP_H
#include <string>
#include <vector>
#include <iostream>
#include <functional>
#include <comma/base/types.h>
#include <comma/dispatch/dispatched.h>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/base/exception.h>
#include <comma/application/signal_flag.h>
#include <boost/optional.hpp>
#include <boost/function.hpp>
#include <boost/filesystem.hpp>
#include "data.h"
#include "auto_initialization.h"
extern "C" {
    #include "simulink/Arm_controller_v2.h"
}
#include "simulink/traits.h"
#include "result.h"
#include "output.h"

namespace snark { namespace ur { namespace handlers {

/// class to perform the SCAN command, moves end effector down, up then back to original tilt angle.
class tilt_sweep
{
    typedef boost::function< void ( void ) > status_updater_t;
    typedef boost::function< bool ( void ) > interrupt_t;    /// A new command is received
    
    ExtU_Arm_controller_v2_T& inputs_;
    const arm_output& serialiser_;
    status_updater_t status_update_;
    const status_t& status_;
    interrupt_t interrupt_;
    comma::signal_flag& signaled_;
    continuum_t config_;
    std::string name_;
    /// The degrees to tilt to
    plane_angle_degrees_t min_; /// Should be negative - tilts down
    plane_angle_degrees_t max_; /// Should be positive - tilts up
    
    struct move_t
    {
        move_t() {};
        move_t( const std::string& m, const plane_angle_t& a ) : action( m ), tilt( a ) {}
        std::string action;
        plane_angle_t tilt;
    };
    
    bool calculate_solution( const length_t& height, const plane_angle_degrees_t& pan, 
                             const plane_angle_degrees_t& tilt_down, const plane_angle_degrees_t& tilt_up, 
                             move_t& move1, move_t& move2, move_t& ret );
    void stop_movement( std::ostream& robot );
    
    void inputs_reset() 
    { 
        inputs_.motion_primitive = input_primitive::no_action;
        inputs_.Input_1 = 0;
        inputs_.Input_2 = 0;
        inputs_.Input_3 = 0;
        inputs_.Input_4 = 0;
        inputs_.Input_5 = 0;
        inputs_.Input_6 = 0;
    }
public:
    tilt_sweep( // boost::function< bool (std::string& move1, std::string& move2 ) > f, /// caculate proposed sweep
                  ExtU_Arm_controller_v2_T& inputs, /// Simulink inputs
                  arm_output& serialiser,       /// Wrapper for Simulink outputs
                  boost::function< void ( void ) > status_updater,
                  const status_t& status,
                  interrupt_t interrupt,
                  comma::signal_flag& signaled,
                  const continuum_t config
        ) : 
                    inputs_( inputs ), serialiser_( serialiser ), 
                    status_update_( status_updater ), status_( status ), 
                    interrupt_( interrupt ), signaled_( signaled ),
                    config_( config ),
                    min_(-45.0*degree), max_(15.0*degree),
                    lidar_filepath_( config.work_directory + '/' + lidar_filename )  {}
  
    void set_min( const plane_angle_degrees_t& min ) { min_ = min; }                  
    void set_max( const plane_angle_degrees_t& max ) { max_ = max; }                  
    /// To be called to signal that the movement has started - for commands like SCAN or AUTO_INIT
    typedef boost::function< void ( void ) > started_reply_t;
    
    result run( const length_t& height, const plane_angle_degrees_t& pan,
                started_reply_t started,
                std::ostream& robot );
     result run( const length_t& height, const plane_angle_degrees_t& pan,
                const plane_angle_degrees_t& tilt_down, const plane_angle_degrees_t& tilt_up, 
                started_reply_t started,
                std::ostream& robot );
    
    const std::string& name() const { return name_; }
    void name( const std::string& name )  { name_ = name ; }

    static const char* lidar_filename;
    const boost::filesystem::path lidar_filepath_;
};
    
} } } // namespace snark { namespace ur { namespace handlers {

#endif // SNARKS_ACTUATORS_UR_ROBOTIC_ARM_CAMERA_SWEEP_H