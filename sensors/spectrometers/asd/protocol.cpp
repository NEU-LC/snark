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

#include "protocol.h"
#include <vector>
#include <comma/io/publisher.h>
#include <comma/application/verbose.h>

namespace snark { namespace asd {

protocol::protocol(const std::string& address,unsigned int t) : ios(address), timeout_seconds(t)
{
    select.read().add( ios.fd());
    comma::verbose<<"asd::protocol: connected on "<<address<<std::endl;
    std::string str;
    if(timeout_seconds)
    {
        select.wait( timeout_seconds );
        if(!select.read().ready( ios.fd() )) { COMMA_THROW(comma::exception, "timeout waiting for read" ); }
    }
    std::getline(*ios, str);
    comma::verbose<<str<<std::endl;
    std::getline(*ios, str);
    comma::verbose<<str<<std::endl;
}

protocol::acquire_reply_t protocol::send_acquire_data(const std::string& command)
{
    *ios<<command<<std::flush;
    snark::asd::commands::acquire_data::spectrum_data reply;
    read_packet(reply);
    boost::posix_time::ptime time=boost::posix_time::microsec_clock::universal_time();
    //error handling
    return acquire_reply_t(time, reply);
}

/*
void protocol_sample_instantiate()
{
    //this adds template instances for linker
    protocol p("");
    p.send<snark::asd::commands::optimize>("");
    p.send<snark::asd::commands::instrument_gain_control>("");
    p.send<snark::asd::commands::restore>("");
    p.send<snark::asd::commands::save>("");
    p.send<snark::asd::commands::erase>("");
    p.send<snark::asd::commands::init>("");
    p.send<snark::asd::commands::version>("");
    p.send<snark::asd::commands::abort>("");
}
*/

} }//namespace snark { namespace asd {
    
