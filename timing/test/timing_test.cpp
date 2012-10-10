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

#include <gtest/gtest.h>
#include <comma/math/compare.h>
#include <snark/timing/clocked_time_stamp.h>
#include <snark/timing/play.h>
#include <snark/timing/ntp.h>
#include <snark/timing/time.h>

namespace snark { namespace test {

static boost::posix_time::ptime make_time( unsigned int seconds )
{
    return boost::posix_time::ptime(timing::epoch, boost::posix_time::seconds(seconds) );
}

TEST(time, clocked_time_stampFrequencyStable)
{
    timing::clocked_time_stamp t( boost::posix_time::seconds( 10 ) );
    boost::posix_time::ptime adjusted;
    adjusted = t.adjusted( make_time( 10 ) );
    EXPECT_EQ( adjusted, make_time( 10 ) );
    adjusted = t.adjusted( make_time( 25 ) );
    EXPECT_EQ( adjusted, make_time( 20 ) );
    adjusted = t.adjusted( make_time( 32 ) );
    EXPECT_EQ( adjusted, make_time( 30 ) );
    adjusted = t.adjusted( make_time( 37 ) );
    EXPECT_EQ( adjusted, make_time( 37 ) );
}

TEST(time, clocked_time_stampFrequencyDriftingUp)
{
    {
        timing::clocked_time_stamp t( boost::posix_time::seconds( 10 ) );
        boost::posix_time::ptime adjusted;
        adjusted = t.adjusted( make_time( 10 ) );
        EXPECT_EQ( adjusted, make_time( 10 ) );
        adjusted = t.adjusted( make_time( 40 ), boost::posix_time::seconds( 20 ), 1 );
        EXPECT_EQ( adjusted, make_time( 30 ) );
    }
}

TEST(time, clocked_time_stampFrequencyDriftingDown)
{
    {
        timing::clocked_time_stamp t( boost::posix_time::seconds( 10 ) );
        boost::posix_time::ptime adjusted;
        adjusted = t.adjusted( make_time( 10 ) );
        EXPECT_EQ( adjusted, make_time( 10 ) );
        adjusted = t.adjusted( make_time( 20 ), boost::posix_time::seconds( 5 ), 1 );
        EXPECT_EQ( adjusted, make_time( 15 ) );
    }
    {
        timing::clocked_time_stamp t( boost::posix_time::seconds( 10 ) );
        boost::posix_time::ptime adjusted;
        adjusted = t.adjusted( make_time( 10 ) );
        EXPECT_EQ( adjusted, make_time( 10 ) );
        adjusted = t.adjusted( make_time( 11 ), boost::posix_time::seconds( 5 ), 1 );
        EXPECT_EQ( adjusted, make_time( 11 ) );
    }
}


static void testTime( const std::string& s )
{
    boost::posix_time::ptime p = boost::posix_time::from_iso_string( s );
    bool secondsOk = ( snark::timing::from_ntp_time( snark::timing::to_ntp_time( p ) ) - p ).total_seconds() == 0;
    bool microsecondsOk = std::abs( static_cast< long >( ( snark::timing::from_ntp_time( snark::timing::to_ntp_time( p ) ) - p ).total_microseconds() ) ) <= 1;
    EXPECT_TRUE( secondsOk ); // TODO fails on windows
    EXPECT_TRUE( microsecondsOk );
    if( !secondsOk || !microsecondsOk )
    {
        std::cerr << "testTime( " << s << " ) failed with" << std::endl;
        std::cerr << "    ntp: seconds: " << snark::timing::to_ntp_time( boost::posix_time::from_iso_string( s ) ).first << std::endl;
        std::cerr << "    ntp: fractions: " << snark::timing::to_ntp_time( boost::posix_time::from_iso_string( s ) ).second << std::endl;
        std::cerr << "    from ntp: " << boost::posix_time::to_iso_string( snark::timing::from_ntp_time( snark::timing::to_ntp_time( boost::posix_time::from_iso_string( s ) ) ) ) << std::endl;
        std::cerr << std::endl;
    }
}

TEST(time, NTP)
{
    static boost::posix_time::ptime epoch( snark::timing::epoch );
    static comma::uint32 epochSec = 2208988800ul;
    static double ntpCoeff = 1000000.0 / ( comma::uint64( 0x100000000 ) );

    EXPECT_TRUE( comma::math::less( 0, ntpCoeff ) ); // sanity check
    EXPECT_EQ( snark::timing::from_ntp_time( epochSec, 0 ), epoch );
    EXPECT_EQ( snark::timing::from_ntp_time( epochSec, 1234 ), epoch + boost::posix_time::microseconds( 1234 * ntpCoeff ) );
    EXPECT_EQ( snark::timing::from_ntp_time( epochSec - 1, 1234 ), epoch - boost::posix_time::seconds( 1 ) + boost::posix_time::microseconds( 1234 * ntpCoeff ) );
    EXPECT_EQ( snark::timing::to_ntp_time( epoch ).first, epochSec );
    EXPECT_EQ( snark::timing::to_ntp_time( epoch ).second, 0u );
    boost::posix_time::ptime t = epoch + boost::posix_time::seconds( 1 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).first, epochSec + 1 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).second, 0u );
    t = epoch - boost::posix_time::seconds( 1 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).first, epochSec - 1 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).second, 0u );
    t = epoch + boost::posix_time::microseconds( 1234 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).first, epochSec );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).second, ( unsigned int )( 1234 / ntpCoeff ) );
    t = epoch - boost::posix_time::microseconds( 1234 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).first, epochSec - 1 );
    EXPECT_EQ( snark::timing::to_ntp_time( t ).second, ( unsigned int )( ( 1000000 - 1234 ) / ntpCoeff ) );

    testTime( "19000101T000000" );
    testTime( "19000101T000001" );
    testTime( "19000101T000010" );
    testTime( "19000102T000010" );
    testTime( "19050101T000000" );
    testTime( "19691231T235959" );
    testTime( "19700101T000000" );
    testTime( "19700101T000001" );
    testTime( "19700101T000010" );
    testTime( "20100101T000000" );
    testTime( "19691231T235959.1" );
    testTime( "19691231T235959.01" );
    testTime( "19691231T235959.001" );
    testTime( "19691231T235959.0001" );
    testTime( "19691231T235959.00001" );
    testTime( "19691231T235959.000001" );
    testTime( "19691231T235959.000009" );
    testTime( "19691231T235959.000099" );
    testTime( "19691231T235959.000999" );
    testTime( "19691231T235959.009999" );
    testTime( "19691231T235959.099999" );
    testTime( "19691231T235959.999999" );
    testTime( "19700101T000000.1" );
    testTime( "19700101T000000.01" );
    testTime( "19700101T000000.001" );
    testTime( "19700101T000000.0001" );
    testTime( "19700101T000000.00001" );
    testTime( "19700101T000000.000001" );
    testTime( "19700101T000000.000009" );
    testTime( "19700101T000000.000099" );
    testTime( "19700101T000000.000999" );
    testTime( "19700101T000000.009999" );
    testTime( "19700101T000000.099999" );
    testTime( "19700101T000000.999999" );
    testTime( "20100101T000000.1" );
    testTime( "20100101T000000.01" );
    testTime( "20100101T000000.001" );
    testTime( "20100101T000000.0001" );
    testTime( "20100101T000000.00001" );
    testTime( "20100101T000000.000001" );
    testTime( "20100101T000000.000009" );
    testTime( "20100101T000000.000099" );
    testTime( "20100101T000000.000999" );
    testTime( "20100101T000000.009999" );
    testTime( "20100101T000000.099999" );
    testTime( "20100101T000000.999999" );
}

} } // namespace snark { namespace Test {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
