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
#include "result.h"
//#include "auto_initialization.h"

namespace snark { namespace ur { namespace handlers {

class commands_handler : public comma::dispatch::handler_of< power >,
    public comma::dispatch::handler_of< brakes >,
//    public comma::dispatch::handler_of< auto_init >,
    public comma::dispatch::handler_of< joint_move >
{
public:
    void handle( power& p );
    void handle( brakes& b );
    void handle( auto_init& a );
    void handle( joint_move& j );

    commands_handler( snark::ur::status_t& status, std::ostream& robot_ostream, const snark::ur::config_t& config ) : 
        status_( status ), os( robot_ostream ), config_( config ), verbose_( false ) {}
    bool is_initialising() const; 
    
    result ret;  /// Indicate if command succeed
private:
    status_t& status_;
    std::ostream& os;
    snark::ur::config_t config_;
    bool verbose_;    
};

} } } // namespace snark { namespace ur { namespace handlers {

#endif // SNARK_ACTUATORS_UNIVERISAL_ROBOTS_COMMANDS_HANDLER_H