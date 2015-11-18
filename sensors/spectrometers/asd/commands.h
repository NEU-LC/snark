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
#include <comma/packed/packed.h>
#include <comma/packed/big_endian.h>

namespace snark { namespace asd { namespace commands {

enum { flash_count = 200 };
    
struct reply_header : public comma::packed::packed_struct< reply_header, 8 >
{
    comma::packed::big_endian_int32 header;
    comma::packed::big_endian_int32 error;
};

struct name_value : public comma::packed::packed_struct< name_value, 38 >
{
    comma::packed::string<30> name;
    comma::packed::big_endian_float64 value;
    //comma::packed::float64 value;
};

struct version
{
    static const char* command() { return "V";  }
    static const char* name() { return "version"; }
    struct reply:public comma::packed::packed_struct<reply,50>
    {
        reply_header header;
        name_value entry;
        comma::packed::big_endian_int32 type;
        static std::string type_description(int type)
        {
            switch(type)
            {
                case 1:return "VNIR";
                case 4:return "SWIR1";
                case 5:return "VNIR/SWIR1";
                case 8:return "SWIR2";
                case 9:return "VNIR/SWIR2";
                case 12:return "SWIR1/SWIR2";
                case 13:return "VNIR/SWIR1/SWIR2";
                default: return "";
            }
        }
    };
};

struct abort
{
    static const char* command() { return "ABORT";  }
    static const char* name() { return "abort"; }
    struct reply:public comma::packed::packed_struct<reply,50>
    {
        reply_header header;
        name_value entry;
        comma::packed::big_endian_int32 count;
    };
};

struct optimize
{
    static const char* command() { return "OPT";  }
    static const char* name() { return "optimize"; }
    struct reply:public comma::packed::packed_struct<reply,28>
    {
        reply_header header;
        comma::packed::big_endian_int32 itime;
        boost::array<comma::packed::big_endian_int32,2> gain;
        boost::array<comma::packed::big_endian_int32,2> offset;
    };
};

struct restore
{
    static const char* command() { return "RESTORE";  }
    static const char* name() { return "restore"; }
    struct reply:public comma::packed::packed_struct<reply, reply_header::size + flash_count * name_value::size + 8>
    {
        reply_header header;
        boost::array<comma::packed::string<30>,flash_count> names;
        boost::array<comma::packed::big_endian_float64, flash_count> values;
        comma::packed::big_endian_int32 count;
        comma::packed::big_endian_int32 verify;
    };
};

struct init
{
    static const char* command() { return "INIT";  }
    static const char* name() { return "init"; }
    struct reply:public comma::packed::packed_struct<reply, reply_header::size + name_value::size + 4>
    {
        reply_header header;
        name_value entry;
        comma::packed::big_endian_int32 count;
    };
};

struct save
{
    static const char* command() { return "SAVE";  }
    static const char* name() { return "save"; }
    struct reply:public comma::packed::packed_struct<reply, reply_header::size + flash_count * name_value::size + 8>
    {
        reply_header header;
        boost::array<comma::packed::string<30>,flash_count> names;
        boost::array<comma::packed::big_endian_float64, flash_count> values;
        comma::packed::big_endian_int32 count;
        comma::packed::big_endian_int32 verify;
    };
};

struct erase
{
    static const char* command() { return "ERASE";  }
    static const char* name() { return "erase"; }
    struct reply:public comma::packed::packed_struct<reply, reply_header::size + flash_count * name_value::size + 8>
    {
        reply_header header;
        boost::array<comma::packed::string<30>,flash_count> names;
        boost::array<comma::packed::big_endian_float64, flash_count> values;
        comma::packed::big_endian_int32 count;
        comma::packed::big_endian_int32 verify;
    };
};

struct instrument_gain_control
{
    static const char* command() { return "IC";  }
    static const char* name() { return "instrument gain control"; }
    struct reply:public comma::packed::packed_struct<reply,reply_header::size + 12>
    {
        reply_header header;
        comma::packed::big_endian_int32 detector;
        comma::packed::big_endian_int32 command_type;
        comma::packed::big_endian_int32 value;
    };
};

struct acquire_data
{
    static const char* command() { return "A,";  }
    static const char* name() { return "acquire data"; }
    struct vnir_header : public comma::packed::packed_struct<vnir_header, 16 * 4>
    {
        comma::packed::big_endian_int32 integration_time;   //IT
        comma::packed::big_endian_int32 scans;
        comma::packed::big_endian_int32 max_channel;
        comma::packed::big_endian_int32 min_channel;
        comma::packed::big_endian_int32 saturation;
        comma::packed::big_endian_int32 shutter;
        boost::array<comma::packed::big_endian_int32,10> reserved;
    };
    struct swir_header : public comma::packed::packed_struct<swir_header, 16 * 4>
    {
        comma::packed::big_endian_int32 tec_status;
        comma::packed::big_endian_int32 tec_current;
        comma::packed::big_endian_int32 max_channel;
        comma::packed::big_endian_int32 min_channel;
        comma::packed::big_endian_int32 saturation;
        comma::packed::big_endian_int32 a_scans;    //A scan
        comma::packed::big_endian_int32 b_scans;    //B scan
        comma::packed::big_endian_int32 dark_current;
        comma::packed::big_endian_int32 gain;
        comma::packed::big_endian_int32 offset;
        comma::packed::big_endian_int32 scan_size_1;
        comma::packed::big_endian_int32 scan_size_2;
        boost::array<comma::packed::big_endian_int32,4> reserved;
    };
    struct spectrum_header : public comma::packed::packed_struct<spectrum_header, reply_header::size + vnir_header::size + ( 2 * swir_header::size ) + 14*4 >
    {
        reply_header header;
        comma::packed::big_endian_int32 sample_count;
        comma::packed::big_endian_int32 trigger;
        comma::packed::big_endian_int32 voltage;
        comma::packed::big_endian_int32 current;
        comma::packed::big_endian_int32 temprature;
        comma::packed::big_endian_int32 motor_current;
        comma::packed::big_endian_int32 instrument_hours;
        comma::packed::big_endian_int32 instrument_minutes;
        comma::packed::big_endian_int32 instrument_type;
        comma::packed::big_endian_int32 ab; //AB
        boost::array<comma::packed::big_endian_int32,4> reserved;
        vnir_header v_header;
        swir_header s1_header;
        swir_header s2_header;
    };
    enum {data_count=2151};
    //FRInterpSpecStruct
    struct spectrum_data : public comma::packed::packed_struct<spectrum_data, spectrum_header::size + data_count * 4>
    {
        spectrum_header header;
        boost::array<comma::packed::big_endian_float32, data_count> values;
    };
};

} } } // namespace snark { namespace asd { namespace commands {

