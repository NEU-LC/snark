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

#include <boost/array.hpp>
#include <boost/static_assert.hpp>
#include <Eigen/Core>
#include <comma/base/types.h>
#include <comma/packed/byte.h>
#include <comma/packed/little_endian.h>
#include <comma/packed/string.h>
#include <comma/packed/struct.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/optional.hpp>

namespace snark { namespace velodyne { namespace puck {

struct packet : public comma::packed::packed_struct< packet, 1206 >
{
    enum { number_of_lasers = 16, number_of_blocks = 12, number_of_subblocks = 2, number_of_returns_per_packet = number_of_lasers * number_of_subblocks * number_of_blocks };
    
    struct laser_return : public comma::packed::packed_struct< laser_return, 3 >
    {
        comma::packed::little_endian_uint16 range;
        comma::packed::byte reflectivity;
        
        double range_as_meters() const { return 0.002 * range(); }
    };
    
    struct block : public comma::packed::packed_struct< block, 2 + 2 + number_of_lasers * number_of_subblocks * sizeof( laser_return ) >
    {
        static const char* ffee() { return "\xFF\xEE"; }
        
        comma::packed::string< 2 > flag;
        comma::packed::little_endian_uint16 azimuth;
        boost::array< boost::array< laser_return, number_of_lasers >, 2 > channels;
        
        double azimuth_as_radians() const { return ( double( azimuth() ) / 100 ) * M_PI / 180; }
    };
    
    struct factory_t : public comma::packed::packed_struct< factory_t, 2 >
    {
        struct modes { enum values { strongest_return = 0x37, last_return = 0x38, dual_return = 0x39 }; };
        struct models { enum values { hdl32 = 0x21, vlp16 = 0x22 }; };
        
        comma::packed::byte mode;
        comma::packed::byte model;
    };

    boost::array< block, number_of_blocks > blocks;
    comma::packed::uint32 timestamp; // time of first shot of the first block (firing sequence), microseconds since past the hour
    comma::packed::uint32 something;
    factory_t factory;
    
    class const_iterator;
};

class packet::const_iterator
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

        const_iterator( const packet* p );

        void operator++();

        const value_type& operator->() const { return value_; }

        const value_type& operator*() const { return value_; }

        bool done();

    private:
        const packet* packet_;
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

} } } // namespace snark { namespace velodyne { namespace puck {
