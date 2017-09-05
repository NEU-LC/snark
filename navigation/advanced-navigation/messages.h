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
#include <comma/base/types.h>
#include <comma/packed/packed.h>
#include <boost/array.hpp>
#include <boost/date_time/posix_time/ptime.hpp>


namespace snark { namespace navigation { namespace advanced_navigation {

namespace messages {

struct header : public comma::packed::packed_struct<header,5>
{
    comma::packed::uint8 LRC;
    comma::packed::uint8 id;
    comma::packed::uint8 length;
    comma::packed::little_endian_uint16 crc;
    bool is_valid() const;
    //The CRC is a CRC16-CCITT. The starting value is 0xFFFF. The CRC covers only the packet data.
    bool check_crc(const char* data) const;   //length is from header
};

struct system_state_packet : public comma::packed::packed_struct<system_state_packet,100>
{
    enum { packet_id = 20 };
    boost::posix_time::ptime t() const;
    
    comma::packed::little_endian_uint16 system_status;
    comma::packed::little_endian_uint16 filter_status;
    comma::packed::little_endian_uint32 unix_time_seconds;
    comma::packed::little_endian_uint32 microseconds;
    comma::packed::little_endian_float64 latitude;  //rad
    comma::packed::little_endian_float64 longitude; //rad
    comma::packed::little_endian_float64 height;    //m
    boost::array<comma::packed::little_endian_float32,3> velocity;  // north, east, down m/s
    boost::array<comma::packed::little_endian_float32,3> body_acceleration; //x,y,z m/s/s
    comma::packed::little_endian_float32 g_force;   //g
    boost::array<comma::packed::little_endian_float32,3> orientation;   //roll,pitch,heading radians
    boost::array<comma::packed::little_endian_float32,3> angular_velocity;  //x,y,z rad/s
    boost::array<comma::packed::little_endian_float32,3> standard_deviation;    //latitude,longitude,height m
};
    
} //namespace messages {
    
} } } //namespace snark { namespace navigation { namespace advanced_navigation {
