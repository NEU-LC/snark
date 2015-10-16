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

struct reply_header : public comma::packed::packed_struct< reply_header, 8 >
{
    comma::packed::big_endian_int32 header;
    comma::packed::big_endian_int32 error;
};

struct name_value : public comma::packed::packed_struct< name_value, 38 >
{
    char name[30];
    comma::packed::big_endian_float64 value;
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
        comma::packed::big_endian_int32 gain[2];
        comma::packed::big_endian_int32 offset[2];
    };
};

struct restore
{
    static const char* command() { return "RESTORE";  }
    static const char* name() { return "restore"; }
    static const int entry_count=200;
    struct reply:public comma::packed::packed_struct<reply, reply_header::size + entry_count * name_value::size + 8>
    {
        reply_header header;
        name_value entry[entry_count];
        comma::packed::big_endian_int32 count;
        comma::packed::big_endian_int32 verify;
    };
};

} } } // namespace snark { namespace asd { namespace commands {
