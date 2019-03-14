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


#include <gtest/gtest.h>
#include <comma/math/compare.h>
#include "../clocked_time_stamp.h"
#include "../play.h"
#include "../ntp.h"
#include "../time.h"
#include "../timestamped.h"
#include "../traits.h"
#include <comma/csv/stream.h>
#include <comma/csv/options.h>

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
    EXPECT_EQ( snark::timing::from_ntp_time( epochSec, 1234 ), epoch + boost::posix_time::microseconds( long( 1234 * ntpCoeff ) ) );
    EXPECT_EQ( snark::timing::from_ntp_time( epochSec - 1, 1234 ), epoch - boost::posix_time::seconds( 1 ) + boost::posix_time::microseconds( long( 1234 * ntpCoeff ) ) );
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

} } // namespace snark { namespace test {

struct data_t
{
    data_t() {}
    data_t( int i ) : i( i ) {}
    int i;
};

namespace comma { namespace visiting {

template <> struct traits< data_t >
{
    template < typename K, typename V > static void visit( const K&, data_t& p, V& v )
    {
        v.apply( "i", p.i );
    }

    template < typename K, typename V > static void visit( const K&, const data_t& p, V& v )
    {
        v.apply( "i", p.i );
    }
};

} } // namespace comma { namespace visiting {

namespace snark { namespace test {

TEST( time, timestamped_basic )
{
    boost::posix_time::ptime t = boost::posix_time::second_clock::local_time();
    int i = 10;
    snark::timestamped< data_t > a( t, data_t( i ) );
    EXPECT_EQ( t, a.t );
    EXPECT_EQ( i, a.data.i );
}

TEST( time, timestamped_read )
{
    std::string iso_t = "20001020T102030.123456";
    int i = 11;
    std::stringstream s;
    s << iso_t << ',' << i;
    comma::csv::options csv;
    csv.full_xpath = false;
    csv.fields = "t,i";
    comma::csv::input_stream< snark::timestamped< data_t > > istream( s, csv );
    const snark::timestamped< data_t >* p = istream.read();
    EXPECT_EQ( boost::posix_time::from_iso_string( iso_t ), p->t );
    EXPECT_EQ( 11, p->data.i );
}

TEST( time, timestamped_write )
{
    std::string iso_t = "21231121T112131.654321";
    boost::posix_time::ptime t = boost::posix_time::from_iso_string( iso_t );
    int i = 12;
    snark::timestamped< data_t > a( t, data_t( i ) );
    std::stringstream s;
    comma::csv::options csv;
    csv.fields = "t,i";
    csv.full_xpath = false;
    comma::csv::output_stream< snark::timestamped< data_t > > ostream( s, csv );
    ostream.write( a );
    EXPECT_EQ( "21231121T112131.654321,12\n", s.str() );
}

} } // namespace snark { namespace test {

int main( int argc, char* argv[] )
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
