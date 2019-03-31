// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2019 Vsevolod Vlaskine
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

/// @author vsevolod vlaskine

#pragma once

#include <boost/array.hpp>
#include <boost/static_assert.hpp>
#include <Eigen/Core>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>
#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>

namespace snark { namespace robosense { namespace msop {
    
struct packet : public comma::packed::packed_struct< packet, 1248 >
{
    struct header_t: public comma::packed::packed_struct< header_t, 42 >
    {
        enum models { rs_lidar_16 = 0x01, rs_lidar_32 = 0x02 };
        
        struct sentinel_t: public comma::packed::packed_struct< sentinel_t, 8 >
        {
            comma::packed::const_byte< 0x55 > byte_0;
            comma::packed::const_byte< 0xaa > byte_1;
            comma::packed::const_byte< 0x05 > byte_2;
            comma::packed::const_byte< 0x0a > byte_3;
            comma::packed::const_byte< 0x5a > byte_4;
            comma::packed::const_byte< 0xa5 > byte_5;
            comma::packed::const_byte< 0x50 > byte_6;
            comma::packed::const_byte< 0xa0 > byte_7;
        };
        
        sentinel_t sentinel;
        boost::array< char, 12 > reserved_0;
        comma::packed::byte model;
        boost::array< char, 10 > reserved_1;
    };
    
    struct data_t : public comma::packed::packed_struct< data_t, 1206 >
    {
        enum { number_of_lasers = 16
             , number_of_blocks = 12
             , number_of_subblocks = 2
             , number_of_returns_per_packet = number_of_lasers * number_of_subblocks * number_of_blocks };
        
        struct laser_return: public comma::packed::packed_struct< laser_return, 3 >
        {
            comma::packed::little_endian_uint16 range;
            comma::packed::byte reflectivity;
            
            double range_as_meters() const { return 0.002 * range(); }
        };
        
        struct block: public comma::packed::packed_struct< block, 2 + 2 + number_of_lasers * number_of_subblocks * sizeof( laser_return ) >
        {
            static const char* ffee() { return "\xFF\xEE"; }
            
            comma::packed::string< 2 > flag;
            comma::packed::little_endian_uint16 azimuth;
            boost::array< boost::array< laser_return, number_of_lasers >, 2 > channels;
            
            double azimuth_as_radians() const { return ( double( azimuth() ) / 100 ) * M_PI / 180; }
        };
        
        struct tail_t: public comma::packed::packed_struct< tail_t, 4 >
        {
            comma::packed::uint32 reserved;
            comma::packed::const_byte< 0x00 > byte_00;
            comma::packed::const_byte< 0xFF > byte_ff;
        };

        boost::array< block, number_of_blocks > blocks;
        tail_t tail;
        
        class const_iterator;
    };
    
    header_t header;
    data_t data;
};

class packet::data_t::const_iterator
{
    public:
        struct value_type
        {
            comma::uint32 id;
            double delay;
            double azimuth;
            double range;
            comma::uint32 reflectivity;

            value_type() : id( 0 ), delay( 0 ), azimuth( 0 ), range( 0 ), reflectivity( 0 ) {}
        };

        const_iterator();

        const_iterator( const packet::data_t* p );

        void operator++();

        const value_type& operator->() const { return value_; }

        const value_type& operator*() const { return value_; }

        bool done();

    private:
        const packet::data_t* packet_;
        unsigned int block_;
        unsigned int subblock_;
        double firing_azimuth_step_;
        double recharge_azimuth_step_;
        value_type value_;
        bool is_dual_return_;
        bool done_;
        void update_value_( double step = 0, double delay = 0 );
        void update_azimuth_step_();
};

} } } // namespace snark { namespace robosense { namespace msop {
