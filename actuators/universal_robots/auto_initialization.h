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

#ifndef SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H
#define SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H

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
#include "result.h"

namespace snark { namespace ur { namespace handlers {

/// This class takes over control from the main loop and do auto init,
/// because we dont want to use a second thread (simpler) and auto init is a long running action.
/// The code is put into this class, with the run member.
class auto_initialization
{
    typedef boost::function< void ( void ) > status_updater_t;
    typedef boost::function< bool ( void ) > interrupt_t;
    
public:
    typedef boost::function< void ( void ) > started_reply_t; /// To be called to signal that the movement has started - for commands like scan or auto_init   
    auto_initialization( snark::ur::status_t& status, std::ostream& robot_ostream, started_reply_t status_updater, comma::signal_flag& signaled, interrupt_t interrupt ) : 
        status_( status ), os( robot_ostream ), update_status_( status_updater ), signaled_( signaled ), interrupt_( interrupt ), force_max_( 13.0 ) {}
    
    void set_app_name( const char* name ) { name_ = name; }
    void set_force_limit( double newtons ){ force_max_ = newtons; }
    
    /// Performs auto initialisation but also listens for new commands.
    /// If a new command arrives or a signal is received run() returns immediately.
    /// result: shows whether success or failure.
    result run( started_reply_t started );
    
private:
    snark::ur::status_t& status_; /// Status to check if initialized 
    std::ostream& os;           /// output to the robot
    status_updater_t update_status_;
    comma::signal_flag& signaled_;  /// Check if signal received
    interrupt_t interrupt_;    /// A new command is received
    
    std::string name_;  // name of the executable running this
    /// This is the value of force limit on arm before failing auto initialisation.
    double force_max_; // newtons    
    const std::string& name() const { return name_; }
};

} } } // namespace snark { namespace ur { namespace handlers {

#endif // SNARKS_ACTUATORS_UR_ROBOTIC_ARM_AUTO_INITIALISATION_H