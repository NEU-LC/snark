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
#include "data.h"
#include "auto_initialization.h"
extern "C" {
    #include "simulink/Arm_Controller.h"
}
#include "simulink/traits.h"
#include "result.h"
#include "output.h"

namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

namespace arm = robotic_arm;

    
class camera_sweep
{
    typedef boost::function< bool (std::string& move1, std::string& move2 ) > calc_func_t;
    typedef boost::function< void ( void ) > status_updater_t;
    typedef boost::function< bool ( void ) > interrupt_t;    /// A new command is received
    
    
    //calc_func_t calculate_;
    ExtU_Arm_Controller_T& inputs_;
    ExtY_Arm_Controller_T& outputs_;
    const arm_output& serialiser_;
    status_updater_t status_update_;
    const status_t& status_;
    interrupt_t interrupt_;
    comma::signal_flag& signaled_;
    std::string name_;
    
    bool calculate_solution( const length_t& height, std::string& move1, std::string& move2 );
    /// Rover is the robotic arm
    void stop_movement( std::ostream& rover );
public:
    camera_sweep( // boost::function< bool (std::string& move1, std::string& move2 ) > f, /// caculate proposed sweep
                  ExtU_Arm_Controller_T& inputs, /// Simulink inputs
                  ExtY_Arm_Controller_T& outputs, /// Simulink outputs
                  arm_output& serialiser,
                  boost::function< void ( void ) > status_updater,
                  const status_t& status,
                  interrupt_t interrupt,
		  comma::signal_flag& signaled
		) : 
                    inputs_( inputs ), outputs_(outputs), serialiser_( serialiser ), 
                    status_update_( status_updater ), status_( status ), 
                    interrupt_( interrupt ), signaled_( signaled ) {}
                    
    result run( const length_t& height, std::ostream& rover );
    
    const std::string& name() const { return name_; }
    void name( const std::string& name )  { name_ = name ; }
};
    
} } } } // namespace snark { namespace ur { namespace robotic_arm { namespace handlers {

#endif // SNARKS_ACTUATORS_UR_ROBOTIC_ARM_CAMERA_SWEEP_H