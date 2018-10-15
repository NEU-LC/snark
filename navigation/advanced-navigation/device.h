// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

/// @author Navid Pirmarzdashti

#pragma once
#include "messages.h"
#include "stream.h"
#include <vector>

namespace snark { namespace navigation { namespace advanced_navigation {

/// spatial dual device class
class device
{
    std::unique_ptr<advanced_navigation::stream> stream;
    std::vector<char> buf;
    unsigned index;
    unsigned head;
    messages::header* msg_header;
public:
    /// name is serial port or - for stdin
    device(const std::string& name,const advanced_navigation::options& options);
    virtual ~device() { }
    void process();
    void send_ntrip(std::vector<char> buf);
    void send(const messages::command command);
    comma::io::file_descriptor fd();
protected:
    virtual void handle(const messages::system_state* msg) { }
    virtual void handle(const messages::raw_sensors* msg) { }
    virtual void handle(const messages::satellites* msg) { }
    virtual void handle(const messages::position_standard_deviation* msg) { }
    virtual void handle(const messages::velocity_standard_deviation* msg) { }
    virtual void handle(const messages::orientation_standard_deviation* msg) { }
    virtual void handle(const messages::acknowledgement* msg) { }
    virtual void handle_raw(messages::header* msg_header, const char* msg_data,std::size_t msg_data_length) { }
};

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
    
