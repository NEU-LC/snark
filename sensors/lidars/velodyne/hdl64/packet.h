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


#ifndef SNARK_SENSORS_VELODYNE_PACKET_H_
#define SNARK_SENSORS_VELODYNE_PACKET_H_

#include <boost/array.hpp>
#include <boost/static_assert.hpp>
#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

namespace snark {  namespace velodyne { namespace hdl64 {

struct packet : public comma::packed::packed_struct< packet, 1206 >
{
    enum { returns_per_block = 32, number_of_blocks = 12 };
    
    struct laser_return : public comma::packed::packed_struct< laser_return, 3 >
    {
        comma::packed::uint16 range;
        comma::packed::byte intensity;
    };
    
    struct laser_block : public comma::packed::packed_struct< laser_block, 2 + 2 + returns_per_block * sizeof( laser_return ) >
    {
        comma::packed::string< 2 > id;
        comma::packed::uint16 rotation;
        boost::array< laser_return, returns_per_block > lasers;
    };
    
    struct status : public comma::packed::packed_struct< status, 6 >
    {
        struct temperature : public comma::packed::packed_struct< temperature, 6 >
        {
            comma::packed::byte fractions;
            comma::packed::byte degrees;
            comma::packed::string< 4 > text;
            bool valid() const { return text() == "DegC"; }
        };
    
        struct version : public comma::packed::packed_struct< version, 6 >
        {
            comma::packed::string< 2 > padding;
            comma::packed::uint16 counter;
            comma::packed::uint16 number;
            bool valid() const { return ::memcmp( data() + 2, "DegC", 4 ) != 0; }
        };
    
        template < class T > const T& as() const { return reinterpret_cast< const T& >( *this ); }
        template < class T > T& As() { return reinterpret_cast< T& >( *this ); }
        boost::array< comma::packed::byte, 6 > value;
    };
    
    static const char* upper_block_id() { return "\xFF\xEE"; }
    
    static const char* lower_block_id() { return "\xFF\xDD"; }

    boost::array< laser_block, number_of_blocks > blocks;
    status status;
};

} } } // namespace snark {  namespace velodyne { namespace hdl64 {

#endif /*SNARK_SENSORS_VELODYNE_PACKET_H_*/
