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

#ifndef SNARK_ACTUATORS_UR_ROBOTIC_ARM_INPUTS_H
#define SNARK_ACTUATORS_UR_ROBOTIC_ARM_INPUTS_H

#include <vector>
#include <queue>
#include <string>
#include <iostream>
#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/shared_ptr.hpp>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/base/types.h>

namespace snark { namespace ur {
/// Buffers inputs by the user, reads from std::cin
/// It reads all user input before the robot starts to process them 
/// Allows the robot to process a small batch of commands at a time but reads all of them
///  as soon as they come in.
/// If a user sends an ESTOP/DISABLE, the buffer of commands is discarded immediately.
/// Purpose is to prevent a large backlog
class inputs
{
public:
    typedef std::vector< std::string > command_tokens;
    typedef std::queue< command_tokens > robot_commands;
    typedef std::list< command_tokens > command_list;
    typedef std::vector< command_tokens > command_vector;
    static const comma::uint16 id_index = 0; // robot's id index
    /// Command's name index
    static const comma::uint16 name_index = 2;
    static const comma::uint16 MAX_BUFFER = 2048;
    
    // A file backend for logs
    inputs( char robot_id );
    bool is_empty() const { return my_commands.empty(); }
    /// It select wait with timeout then reads the inputs
    /// Note: if timeout > 0, it does not handle signal interrupt during waiting
    void read( const boost::posix_time::time_duration& timeout = boost::posix_time::microseconds( 0 ) );
    
    command_tokens& front() { return my_commands.front(); }
    const std::string next_command_name() const { return my_commands.front()[ name_index ]; }
    void pop() { my_commands.pop(); }
    
private:
    char robot_id;
    
    comma::io::istream istream_;
    comma::io::select select_;
    
    /// The buffer of commands
    robot_commands my_commands;
};

} } // namespace snark { namespace ur {

#endif // SNARK_ACTUATORS_UR_ROBOTIC_ARM_INPUTS_H
