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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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

#ifndef SNARK_SENSORS_HOKUYO_MESSAGE_H
#define SNARK_SENSORS_HOKUYO_MESSAGE_H

#include <boost/static_assert.hpp>
#include <boost/utility/binary.hpp>
#include <comma/base/types.h>
#include <comma/packed/packed.h>
#include <iomanip>

namespace comma { namespace packed {
    
/// The character encoding used by Hokuyo with SCIP 
template < unsigned int N, typename T=comma::uint32 >
class scip_encoding : public comma::packed::field< scip_encoding< N >, T, N >
{
public:
    enum { size = N };
    
    BOOST_STATIC_ASSERT( size > 0 );
    
    typedef T type;
    
    typedef comma::packed::field< scip_encoding< N >, type, size > base_type;
    
    static type default_value() { return 0; }
    
    static const unsigned char mask = BOOST_BINARY( 111111 );
    static const unsigned char offset = 0x30;
    
    static void pack( char* storage, type value )
    {
        for( unsigned int i = 0; i<N; ++i )
        {
            unsigned int bits_shift = 6 * (N - (i+1));
            char c = ( ( value & ( mask << bits_shift ) ) >> bits_shift ) + offset; 
            storage[i] = c;
        }
    }
    
    static type unpack( const char* storage )
    {
        type value = 0;
        for( unsigned int i=0; i<N; ++i )
        {
            comma::uint32 part( storage[i] );
            unsigned int bits_shift = 6 * (N - (i+1));
            // std::cerr << " int: " << part << " shift: " << bits_shift;
            value |= ( ( ( part - offset ) & mask ) << bits_shift );
            // comma::uint32 bits = ( ( part - offset ) & mask );
            // std::cerr << " bits: " << bits << " value: " << value << std::endl;
        }
        return value;
    }
    
    const scip_encoding< N >& operator=( const scip_encoding< N >& rhs ) { return base_type::operator=( rhs ); }
    
    const scip_encoding< N >& operator=( type rhs ) { return base_type::operator=( rhs ); }
};

typedef scip_encoding< 2, comma::uint16 > scip_2chars_t;
typedef scip_encoding< 3 > scip_3chars_t;
typedef scip_encoding< 4 > scip_4chars_t;

template < typename T, std::size_t S, char Padding = ' ' >
class casted_left_fill : public packed::field< casted_left_fill< T, S, Padding >, T, S >
{
public:
    enum { size = S };
    
    BOOST_STATIC_ASSERT( size > 0 );
    
    typedef T Type;
    
    typedef packed::field< casted_left_fill< T, S, Padding >, T, S > base_type;
    
    static T default_value() { return 0; }
    
    static void pack( char* storage, const T& value )
    {
        std::ostringstream ss;
        ss << std::setfill(Padding) << std::setw (S) << value;
        ::memcpy( storage, &(ss.str()[0]), size );
    }
    
    static T unpack( const char* storage )
    {
        // do not strip if Padding is '0' e.g. representing of '00'
        std::string value_str = Padding == '0' ? std::string( storage, size ) : comma::strip( std::string( storage, size ), Padding );
        return boost::lexical_cast< T >( value_str );
    }
    
    const casted_left_fill& operator=( const T& rhs ) { return base_type::operator=( rhs ); }
};

} } // namespace comma { namespace packed {

namespace snark { namespace hokuyo {
    
struct status {
    /// stopped and hardware errors is a range of values, only 0 is success - else error.
    enum { success=0, start_step_format=1, end_step_format=2, cluster_count_format=3, 
           end_step_range=4, end_step_small=5, scan_interval_format=6, num_of_scan_format=7,
           stopped_min=21, stopped_max=49, hardware_min=50, hardware_max=97, resuming_after_stop=98,
           data_success=99
    };
};

typedef comma::packed::const_byte< ';' > semi_colon_t; /// Final terminating line feed
    
/// This is the header for host to sensor message
/// CMD(2) + Start Step(4) + End Step(4) + Cluster Count(2)
struct header : public comma::packed::packed_struct< header, 12  > {
    
    header() { }
    header( const std::string& tag ) { cmd = tag; }
    comma::packed::string< 2 > cmd;
    comma::packed::casted_left_fill< comma::uint16, 4, '0' > start_step;
    comma::packed::casted_left_fill< comma::uint16, 4, '0' > end_step;
    comma::packed::casted_left_fill< comma::uint16, 2, '0' > cluster_count;
};

typedef header host_to_sensor;
typedef header sensor_to_host;
    
typedef comma::packed::const_byte< '\n' > line_feed_t; /// Final terminating line feed

static const unsigned char mask = BOOST_BINARY( 111111 );

/// This is the string ID of request, making it '<cmd><seq num>'
struct sequence_string : comma::packed::packed_struct< sequence_string, 12 >
{
    sequence_string() {}
    sequence_string( const std::string& cmd, comma::uint16 seq )  { tag = cmd; seq_num = seq; }
    semi_colon_t sc;
    comma::packed::string< 2 > tag; // e.g. GD or MD
    comma::packed::casted_left_fill< comma::uint16, 8, '0' > seq_num;
    line_feed_t lf;
    
    const std::string str() const { return tag() + std::string( seq_num.data(), 8 ); }
};

/// This is used to pair the request with the reply, it can roll over
static comma::uint16 sequence = 0;

static const int reply_header_size = sensor_to_host::size + sequence_string::size;

/// For generic state change commands, BM, PP or RB command
struct state_command : public comma::packed::packed_struct< state_command, 15 >
{
    state_command( const std::string& cmd_ ) : message_id( cmd_, ++sequence ) { cmd = cmd_; }
    comma::packed::string< 2 > cmd;
    sequence_string message_id;
    line_feed_t lf_end;
};

/// Represents the generic replies from state_commands.
/// Although commands like PP has extra returned data.
struct state_reply : public comma::packed::packed_struct< state_reply, state_command::size - 1 + 5 >
{
    comma::packed::string< 2 > cmd;
    sequence_string message_id;
    comma::packed::casted_left_fill< comma::uint16, 2, '0' > status;
    char sum;
    line_feed_t lf;
    line_feed_t lf_end;
};



/// Can be used for MD or GD request, just change the 'cmd' member
struct request_gd : public comma::packed::packed_struct< request_gd, reply_header_size  >
{
    
    request_gd( bool is_ge=false ) : header( is_ge ? "GE" : "GD"  ), message_id( is_ge ? "GE" : "GD", ++sequence )  {}
    host_to_sensor header;
    sequence_string message_id;
};

/// Checksum is only one byte
static const char size_of_sum = 1;

/// Verify checksum of the data given that the last byte is the checksum.
/// Value of checksum is the last 6 bits of the sum of the data bytes
bool scip_verify_checksum( const std::string& line );

/// For calculating the size of the data mixed with checksum and line feeds
template < int STEPS >
struct distance_data { 
    /// The data is mixed with check sum every 64 bytes
    static const char encoding_num = 3; // 3 character encoding
    static const comma::uint16 data_only_size = STEPS*encoding_num; // Has distance and intensity data
    static const unsigned char num_of_sums = ( (data_only_size)/64 ) + ( data_only_size % 64 > 0 ? 1: 0 ); 
    static const comma::uint16 value = data_only_size + num_of_sums*(size_of_sum + 1); /// adding one for line feed
    
    /// This is the array of raw data without the checksum and line feeds
    typedef boost::array< comma::packed::scip_3chars_t, STEPS > points_t;
    /// This is the raw or blob data with checksum and linefeeds, see get_values()
    comma::packed::string< value > raw_data;   /// data mixed with checksums and linefeeds
    
    /// This is the array of raw data without the checksum and line feeds
    struct rays : public comma::packed::packed_struct< rays, data_only_size >
    {
        points_t steps;
    };
    
    /// This is the function to get data into 'points', as checksums and linefeeds are stripped.
    void get_values( rays& points ) const; 
    
};

/// This is used to calculate the size of returned data values, each value is 3 bytes/chars.
/// However checksum and linefeed is added after every 64 bytes, as well the last few bytes.
template < int STEPS >
struct di_data {
    /// The data is mixed with check sum every 64 bytes
    static const char encoding_num = 3; // 3 character encoding
    static const comma::uint16 data_only_size = STEPS*encoding_num*2; // Has distance and intensity data
    // 64 bytes per sum check value, plus one checksum for remaining bytes
    static const unsigned char num_of_sums = ( (data_only_size)/64 ) + ( data_only_size % 64 > 0 ? 1 : 0 ); 
    static const comma::uint16 value = data_only_size + num_of_sums*(size_of_sum + 1); /// adding one for line feed

    /// Store an array of contiguous character data - encoded data
    
    struct pair_t 
    {
        comma::packed::scip_3chars_t distance;
        comma::packed::scip_3chars_t intensity;
    };
    /// This is the array of data without the checksum and line feeds
    typedef boost::array< pair_t, STEPS > points_t;
    /// This is the raw or blob data with checksum and linefeeds, see get_values()
    comma::packed::string< value > raw_data;   
    
    /// This is the array of raw data without the checksum and line feeds
    struct rays : public comma::packed::packed_struct< rays, data_only_size >
    {
        points_t steps;
    };
    
    /// This is the function to get data into 'points', as checksums and linefeeds are stripped.
    void get_values( rays& points ) const; 
};

struct status_t : comma::packed::packed_struct< status_t, 3 >
{
    // TODO: this maybe SCIP encoding
    comma::packed::casted_left_fill< comma::uint16, 2, '0' > status;
    //comma::packed::scip_2chars_t status;
    char sum;
    comma::uint32 operator()() const { return this->status(); }
};

struct timestamp_t : comma::packed::packed_struct< timestamp_t, 5 >
{
    static const std::size_t size = 5;
    comma::packed::scip_4chars_t timestamp; // just a count from 0
    char sum;
    
    comma::uint32 operator()() const { return this->timestamp(); }
};

/// Can be used for MD or ME request
struct request_md : comma::packed::packed_struct< request_md, reply_header_size + 1 + 2 >
{
    request_md( bool is_me=false ) : header( is_me ? "ME" : "MD" ), message_id( is_me ? "ME" : "MD", ++sequence ) {}
    host_to_sensor header;
    comma::packed::const_byte< '0' > scan_interval; // skips nothing
    comma::packed::casted_left_fill< comma::uint16, 2, '0' > num_of_scans;
    sequence_string message_id;
};

/// The replies include the exact format for the request, plus status and timestamp.
struct reply
{
    struct gd_header
    {
        request_gd request;
        status_t status;
        line_feed_t lf;
        timestamp_t timestamp;
        line_feed_t lf1;        
    };
    struct md_header
    {
        request_md request;
        status_t status;
        line_feed_t lf;
        timestamp_t timestamp;
        line_feed_t lf1;
    };
};

/// Depends on how many steps, always 3 character encoding
template < int STEPS >
struct reply_gd : comma::packed::packed_struct< reply_gd< STEPS >, sizeof( reply::gd_header ) + distance_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    reply::gd_header header;

    distance_data< STEPS > encoded;
    line_feed_t lf_end; /// Final terminating line feed
};

template < int STEPS >
struct reply_ge : comma::packed::packed_struct< reply_ge< STEPS >, sizeof( reply::gd_header ) + di_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    reply::gd_header header;

    /// each sum value is followed by a line feed char
    di_data< STEPS > encoded;
    line_feed_t lf_end; /// Final terminating line feed
};

/// This is a reply as an acknowledgement of reques     t - no data
struct reply_md : comma::packed::packed_struct< reply_md, request_md::size + status_t::size + 2 > /// 1 for last line feed char
{
    request_md request;
    status_t status;
    line_feed_t lf;
    line_feed_t lf_end;
};


/// Depends on how many steps, always 3 character encoding
template < int STEPS >
struct reply_md_data : comma::packed::packed_struct< reply_md_data< STEPS >, sizeof( reply::md_header ) + distance_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    reply::md_header header;

    distance_data< STEPS > encoded;
    line_feed_t lf_end; /// Final terminating line feed
};

template < int STEPS >
struct reply_me_data : comma::packed::packed_struct< reply_me_data< STEPS >, sizeof( reply::md_header ) + di_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    reply::md_header header;
    
    di_data< STEPS > encoded;
    line_feed_t lf_end; /// Final terminating line feed
};
    
} } // namespace snark { namespace hokuyo {
    
    
#endif // SNARK_SENSORS_HOKUYO_MESSAGE_H
