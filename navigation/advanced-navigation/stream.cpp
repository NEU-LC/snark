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

#include "stream.h"
#include <comma/base/exception.h>

namespace snark { namespace navigation { namespace advanced_navigation {

serial_stream::serial_stream(const std::string& name,const advanced_navigation::options& options) : 
    port(service,name)
{
    port.set_option(boost::asio::serial_port_base::baud_rate(options.baud_rate));
    port.set_option(boost::asio::serial_port_base::character_size(8));
    port.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    port.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
}

std::size_t serial_stream::read_some(char* buf,std::size_t to_read)
{
    boost::system::error_code ec;
//         unsigned read_size=boost::asio::read(port, boost::asio::buffer(&buf[index],to_read));
    return port.read_some(boost::asio::buffer(buf,to_read),ec);
}
std::size_t serial_stream::write(const char* buf,std::size_t to_write)
{
    return boost::asio::write(port, boost::asio::buffer(buf,to_write));
}

comma::io::file_descriptor serial_stream::fd()
{
    return port.native_handle();
}

io_stream::io_stream(const std::string& name,const advanced_navigation::options& options) : 
    is(name,comma::io::mode::binary,comma::io::mode::non_blocking)
{
//     std::cerr<<"io_stream::io_stream "<<name<<" "<<(bool)ios<<std::endl;
}
std::size_t io_stream::read_some(char* buf,std::size_t to_read)
{
//     std::cerr<<"io_stream::read_some "<<to_read<<" "<<(void*)buf<<std::endl;
    if(!is->good()) { throw eois_exception(std::string("end of file on istream ")+is.name()); }
//     return is->readsome(buf,to_read);
    is->read(buf,to_read);
    return is->gcount();
//     unsigned read=ios->gcount();
//     std::cerr<<"io_stream::read_some / "<<read<<std::endl;
//     return read;
}
std::size_t io_stream::write(const char* buf,std::size_t to_write)
{
    COMMA_THROW( comma::exception, "cannot write to istream");
//     ios->write(buf,to_write);
//     return ios->good()?to_write:0;
}

comma::io::file_descriptor io_stream::fd()
{
    return is.fd();
}

} } } //namespace snark { namespace navigation { namespace advanced_navigation {
    
