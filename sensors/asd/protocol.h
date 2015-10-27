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
// 3. Neither the name of the University of Sydney nor the
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

#pragma once
#include "commands.h"
#include <utility>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/io/stream.h>
#include <snark/timing/timestamped.h>

namespace snark { namespace asd {

    
class protocol
{
    comma::io::iostream ios;
    //number of acquire_data responses expected to receive
    int more;
    //throw exception on error if strict
    bool strict;
public:
    typedef snark::timestamped<snark::asd::commands::acquire_data::spectrum_data> acquire_reply_t;
    //tcp address
    protocol(const std::string& address, bool strict);
    template<typename T>
    snark::timestamped<typename T::reply> send(const std::string& command);
    acquire_reply_t send_acquire_data(const std::string& command);
    void handle_reply(const commands::reply_header& header);
    //these functions use internal state (not concurrent safe)
    void send_acquire_cmd(const std::string& command);
    bool more_acquire_data();
    acquire_reply_t get_acquire_response();
};

template<typename T>
snark::timestamped<typename T::reply> protocol::send(const std::string& command)
{
    *ios<<command<<std::flush;
    typename T::reply reply;
    ios->read(reply.data(),reply.size);
    boost::posix_time::ptime time=boost::posix_time::microsec_clock::local_time();
    //error handling
    handle_reply(reply.header);
    return snark::timestamped<typename T::reply>(time, reply);
}

} }//namespace snark { namespace asd {

