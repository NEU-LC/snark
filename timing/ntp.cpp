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


#include <comma/base/exception.h>
#include <snark/timing/ntp.h>

namespace snark{ namespace timing {

static boost::posix_time::ptime ntp_base( boost::posix_time::from_iso_string( "19000101T000000" ) );
static boost::posix_time::ptime epoch_time( timing::epoch );
static comma::int64 ntp_diff = 2208988800ul;
static double ntp_microsec_coeff = ( 1000000.0 / 0x10000 ) / 0x10000; // to avoid overflow error

std::pair< comma::uint32, comma::uint32 > to_ntp_time( boost::posix_time::ptime t )
{
    if( t < ntp_base ) { COMMA_THROW_STREAM( comma::exception, "cannot convert to ntp time: " << t << ", which is less than NTP time base " << ntp_base ); }
    comma::int32 s = ( t - epoch_time ).total_seconds(); // 32 bit signed int in boost and posix
    comma::int32 m = t.time_of_day().total_microseconds() % 1000000;
    if( t >= epoch_time || m == 0 )
    {
        return std::pair< comma::uint32, comma::uint32 >( static_cast< comma::uint32 >( ntp_diff + s ), static_cast< comma::uint32 >( m / ntp_microsec_coeff ) );
    }
    else
    {
        return std::pair< comma::uint32, comma::uint32 >( static_cast< comma::uint32 >( ntp_diff + s - 1 ), static_cast< comma::uint32 >( m / ntp_microsec_coeff ) );
    }
}

boost::posix_time::ptime from_ntp_time( std::pair< comma::uint32, comma::uint32 > ntp )
{
    return from_ntp_time( ntp.first, ntp.second );
}

boost::posix_time::ptime from_ntp_time( comma::uint32 seconds, comma::uint32 fractions )
{
    boost::posix_time::ptime t = ntp_base + boost::posix_time::microseconds( static_cast< comma::uint64 >( fractions * ntp_microsec_coeff ) );
    static const comma::int64 step = 2000000000; // quick and dirty: to avoid overflow in boost::posix_time::seconds(long)
    for( comma::int64 s = seconds; s > 0; t += boost::posix_time::seconds( std::min( s, step ) ), s -= step );
    return t;
}

} } // namespace snark{ namespace timing
