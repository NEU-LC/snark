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
#include <comma/io/select.h>
#include "../../../timing/timestamped.h"

namespace snark { namespace asd {

class protocol
{
    comma::io::iostream ios;
    comma::io::select select;
    unsigned int timeout_seconds;
    template<typename T>
    void read_packet(T& t);
public:
    typedef snark::timestamped<snark::asd::commands::acquire_data::spectrum_data> acquire_reply_t;
    //tcp address
    protocol(const std::string& address, unsigned int timeout_seconds=0);
    template<typename T>
    snark::timestamped<typename T::reply> send(const std::string& command);
    acquire_reply_t send_acquire_data(const std::string& command);
};

template<typename T>
void protocol::read_packet(T& t)
{
    if(timeout_seconds)
    {
        select.wait( timeout_seconds );
        if(!select.read().ready( ios.fd() )) { COMMA_THROW(comma::exception, "timeout waiting for read" ); }
    }
    ios->read(t.data(),t.size);
    std::streamsize read_count=ios->gcount();
    if(read_count != t.size) { COMMA_THROW(comma::exception, "read count mismatch, expected: " << t.size << " bytes; got: " << read_count );}
}

template<typename T>
snark::timestamped<typename T::reply> protocol::send(const std::string& command)
{
    *ios<<command<<std::flush;
    typename T::reply reply;
    read_packet(reply);
    boost::posix_time::ptime time=boost::posix_time::microsec_clock::universal_time();
    //error handling
    return snark::timestamped<typename T::reply>(time, reply);
}

} }//namespace snark { namespace asd {

