// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2016 The University of Sydney
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

#include <string>
#include <boost/asio.hpp>
#include <boost/concept_check.hpp>
// #include <boost/asio/serial_port.hpp>
//#include <boost/asio/read.hpp>

extern bool debug_verbose;
namespace snark { namespace hokuyo {

struct stream_base
{
    virtual ~stream_base() { }
    virtual void read(char* data, std::size_t size)=0;
    virtual void write(const char* data, std::size_t size)=0;
    virtual void flush()=0;
    stream_base& operator <<(const std::string& str)
    {
        write(str.c_str(),str.length());
        return *this;
    }
    virtual int native()=0;
    virtual void close()=0;
    virtual std::size_t bytes_read() const = 0;
};

struct tcp_stream:public stream_base
{
    boost::asio::ip::tcp::iostream ios;
    tcp_stream(){}
    virtual void read(char* data, std::size_t size)
    {
        ios.read(data,size);
    }
    virtual void write(const char* data, std::size_t size)
    {
        ios.write(data,size);
    }
    virtual void flush()
    {
        ios.flush();
    }
    virtual int native()
    {
#if (BOOST_VERSION >= 106600)
        return ios.rdbuf()->native_handle();
#else
        return ios.rdbuf()->native();
#endif
    }
    virtual void close()
    {
        ios.close();
    }
    virtual std::size_t bytes_read() const
    {
        return ios.gcount();
    }
};

struct serial_stream:public stream_base
{
    // name: port filename eg "COM1" or  "/dev/ttyS0"
    boost::asio::io_service service;
    boost::asio::serial_port port;
    std::size_t last_read;
    serial_stream(const std::string& name) : service(), port(service,name), last_read(0)
    {
    }
    serial_stream(const std::string& name,
                  int baud_rate, 
                  int char_size, 
                  boost::asio::serial_port_base::parity::type parity,
                  boost::asio::serial_port_base::stop_bits::type stop_bits
                 ) : service(), port(service,name), last_read(0)
    {
        port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
        port.set_option(boost::asio::serial_port_base::character_size(char_size));
        port.set_option(boost::asio::serial_port_base::parity(parity));
        port.set_option(boost::asio::serial_port_base::stop_bits(stop_bits));
    }
    void set_baud_rate(int baud_rate)
    {
        port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    }
    void dump_setting()
    {
        boost::asio::serial_port_base::baud_rate baud_rate;
        boost::asio::serial_port_base::flow_control flow_control;
        boost::asio::serial_port_base::parity parity;
        boost::asio::serial_port_base::stop_bits stop_bits;
        boost::asio::serial_port_base::character_size char_size;
        port.get_option(baud_rate);
        port.get_option(flow_control);
        port.get_option(parity);
        port.get_option(stop_bits);
        port.get_option(char_size);
        std::cerr<<baud_rate.value()<< "-"<<char_size.value()<< to_string(parity.value())<<to_string(stop_bits.value())<<" ctrl "<<to_string(flow_control.value())<<std::endl;
    }
    static std::string dump(const char* data, std::size_t size)
    {
        static char buf[20];
        std::string s;
        for(std::size_t i=0;i<size;i++)
        {
            if(data[i]=='\n')
                s+="\\n\n";
            else if(data[i]==0)
                s+=".";
//             else if(data[i]==-1)
//                 s+="\\-1";
            else if(data[i]<'0' || data[i]>'z')
            {
                sprintf(buf,"%x",(int)(unsigned char)data[i]);
                s+="\\x"+std::string(buf);
            }
            else
                s+=std::string(1,data[i]);
        }
        return s;
    }
    virtual void read(char* data, std::size_t size)
    {
        last_read = boost::asio::read( port, boost::asio::buffer( data, size ) );
        if(debug_verbose)
            std::cerr<<"<-["<<(int)size<<"]"<<dump(data,size)<<std::endl;
    }
    virtual void write(const char* data, std::size_t size)
    {
        if(debug_verbose)
            std::cerr<<"->"<<dump(data,size)<<std::endl;
        boost::asio::write(port,boost::asio::buffer(data,size));
    }
    virtual void flush()
    {
        //port.flush();
    }
    virtual int native()
    {
#if (BOOST_VERSION >= 106600)
        return port.native_handle();
#else
        return port.native();
#endif
    }
    virtual void close()
    {
        port.close();
    }
    virtual std::size_t bytes_read() const
    {
        return last_read;
    }
    const char* to_string(boost::asio::serial_port_base::parity::type t)
    {
        switch(t)
        {
            case boost::asio::serial_port_base::parity::none:
                return "N";
            case boost::asio::serial_port_base::parity::even:
                return "E";
            case boost::asio::serial_port_base::parity::odd:
                return "O";
        }
        return "?";
    }
    const char* to_string(boost::asio::serial_port_base::stop_bits::type t)
    {
        switch(t)
        {
            case boost::asio::serial_port_base::stop_bits::one:
                return "1";
            case boost::asio::serial_port_base::stop_bits::onepointfive:
                return "1.5";
            case boost::asio::serial_port_base::stop_bits::two:
                return "2";
        }
        return "?";
    }
    const char* to_string(boost::asio::serial_port_base::flow_control::type t)
    {
        switch(t)
        {
            case boost::asio::serial_port_base::flow_control::none:
                return "none";
            case boost::asio::serial_port_base::flow_control::software:
                return "sw";
            case boost::asio::serial_port_base::flow_control::hardware:
                return "hw";
        }
        return "?";
    }
};

} } // namespace comma { namespace visitting {
    

