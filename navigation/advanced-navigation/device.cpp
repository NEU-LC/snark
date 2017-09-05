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

#include "device.h"

namespace snark { namespace navigation { namespace advanced_navigation {

device::device(const std::string& name,int baud_rate) : port(service,name)
{
    port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
}
void device::process()
{
    //port.read_some<>();
//     read (some) into buf
//     len>5
//         is valid head
//             calculate length
//             read to length if not already read
//             check crc
//             discard if crc fails
//             copy to message packet
//             debug log if active
//             call handler/or return if any
//         else advance buf ptr 1 byte //discarding 1 byte debug?
}
void device::read(char* data, std::size_t size)
{
    last_read = boost::asio::read( port, boost::asio::buffer( data, size ) );
//         if(debug_verbose)
//             std::cerr<<"<-["<<(int)size<<"]"<<dump(data,size)<<std::endl;
}
void device::write(const char* data, std::size_t size)
{
//         if(debug_verbose)
//             std::cerr<<"->"<<dump(data,size)<<std::endl;
    boost::asio::write(port,boost::asio::buffer(data,size));
}
int device::native()
{
    return port.native();
}

    
} } } //namespace snark { namespace navigation { namespace advanced_navigation {
    
