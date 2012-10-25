// This file is part of snark, a generic and flexible library 
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License 
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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
    static const comma::int32 step = 2000000000; // quick and dirty: to avoid overflow in boost::posix_time::seconds(long)
    for( comma::int32 s = seconds; s > 0; t += boost::posix_time::seconds( std::min( s, step ) ), s -= step );
    return t;
}

} } // namespace snark{ namespace timing
