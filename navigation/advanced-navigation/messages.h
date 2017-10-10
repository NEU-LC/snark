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
#include "../../math/spherical_geometry/coordinates.h"


namespace snark { namespace navigation { namespace advanced_navigation {

namespace messages {

/// header packet
struct header : public comma::packed::packed_struct<header,5>
{
    comma::packed::uint8 LRC;
    comma::packed::uint8 id;
    comma::packed::uint8 length;
    comma::packed::little_endian_uint16 msg_crc;
    bool is_valid() const;
    //The CRC is a CRC16-CCITT. The starting value is 0xFFFF. The CRC covers only the packet data.
    bool check_crc(const char* data) const;   //length is from header
    header();
    header(unsigned char id, unsigned char length,const char* data);
};

struct system_state : public comma::packed::packed_struct<system_state,100>
{
    enum { id = 20 };
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
    boost::array<comma::packed::little_endian_float32,3> position_stddev;    //latitude,longitude,height m
    snark::spherical::coordinates coordinates() const;
};

struct raw_sensors : public comma::packed::packed_struct<raw_sensors,48>
{
    enum { id = 28 };
    boost::array<comma::packed::little_endian_float32,3> accelerometer; //x,y,z m/s/s
    boost::array<comma::packed::little_endian_float32,3> gyroscope; //x,y,z rad/s
    boost::array<comma::packed::little_endian_float32,3> magnetometer;  //x,y,z mG
    comma::packed::little_endian_float32 imu_temperature;   //deg C
    comma::packed::little_endian_float32 pressure;  //Pascals
    comma::packed::little_endian_float32 pressure_temperature;  //deg C
};

struct satellites : public comma::packed::packed_struct<satellites,13>
{
    enum { id = 30 };
    comma::packed::little_endian_float32 hdop;
    comma::packed::little_endian_float32 vdop;
    comma::packed::uint8 gps_satellites;
    comma::packed::uint8 glonass_satellites;
    comma::packed::uint8 beidou_satellites;
    comma::packed::uint8 galileo_satellites;
    comma::packed::uint8 sbas_satellites;
};

struct rtcm_corrections : public comma::packed::packed_struct<rtcm_corrections,260>
{
    enum { id = 55 };
    messages::header header;
    boost::array<comma::packed::uint8,255> msg_data;
    rtcm_corrections() { }
    rtcm_corrections(const char* buf, unsigned size);
};

struct position_standard_deviation : public comma::packed::packed_struct<position_standard_deviation,12>
{
    enum { id = 24 };
    boost::array<comma::packed::little_endian_float32,3> stddev;
};

struct velocity_standard_deviation : public comma::packed::packed_struct<velocity_standard_deviation,12>
{
    enum { id = 25 };
    boost::array<comma::packed::little_endian_float32,3> stddev;
};

//euler_orientation_standard_deviation_packet_t
struct orientation_standard_deviation : public comma::packed::packed_struct<velocity_standard_deviation,12>
{
    enum { id = 26 };
    boost::array<comma::packed::little_endian_float32,3> stddev;
};

} //namespace messages {
    
} } } //namespace snark { namespace navigation { namespace advanced_navigation {
