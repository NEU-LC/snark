#ifndef SNARK_SENSORS_HOKUYO_MESSAGE_H
#define SNARK_SENSORS_HOKUYO_MESSAGE_H
#include <boost/utility/binary.hpp>
#include <comma/base/types.h>
#include <comma/packed/packed.h>

namespace comma { namespace packed {

/// The character encoding used by SCIP 
template < int N >
class scip_encoding : public comma::packed::field< scip_encoding< N >, comma::uint32, N >
{
public:
    enum { size = N };
    
    typedef comma::uint32 type;
    
    typedef comma::packed::field< scip_encoding< N >, comma::uint32, size > base_type;
    
    static type default_value() { return 0; }

    static const unsigned char mask = BOOST_BINARY( 111111 );
    static const unsigned char offset = 0x30;
    
    static void pack( char* storage, type value )
    {
        for( int i=0; i<N; ++i )
        {
            int bits_shift = 6 * (N - (i+1));
            char c = ( ( value & ( mask << bits_shift ) ) >> bits_shift ) + offset; 
            // std::cerr << "byte: " << i << " to " << char(int(c)) << " int: " << int(c) << std::endl;
            storage[i] = c;
        }
    }
    
    static type unpack( const char* storage )
    {
        comma::uint32 value = 0;
        for( int i=0; i<N; ++i )
        {

            // std::cerr << "hokuyo char: " << storage[i]; 
            comma::uint32 part( storage[i] );
            int bits_shift = 6 * (N - (i+1));
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

typedef scip_encoding< 3 > scip_3chars_t;
typedef scip_encoding< 4 > scip_4chars_t;

} } // namespace comma { namespace packed {

namespace snark { namespace hokuyo {
    
struct status {
    /// stopped and hardware errors is a range of values, only 0 is success - else error.
    enum { success=0, start_step_format=1, end_step_format=2, cluster_count_format=3, 
           end_step_range=4, end_step_small=5, scan_interval_format=6, num_of_scan_format=7,
           stopped_min=21, stopped_max=49, hardware_min=50, hardware_max=97, resuming_after_stop=98 };
};

    
/// This is the header for host to sensor message
/// CMD(2) + Start Step(4) + End Step(4) + Cluster Count(2)
struct header : public comma::packed::packed_struct< header, 12  > {
    
    header() {}
    header( const std::string& tag ) { cmd = tag; }
    comma::packed::string< 2 > cmd;
    comma::packed::casted< comma::uint16, 4, '0' > start_step;
    comma::packed::casted< comma::uint16, 4, '0' > end_step;
    comma::packed::casted< char, 2, '0' > cluster_count;
};

typedef header host_to_sensor;
typedef header sensor_to_host;
    
typedef comma::packed::const_byte< '\n' > line_feed_t; /// Final terminating line feed

static const unsigned char mask = BOOST_BINARY( 111111 );

/// This is the string ID of request, making it '<cmd><seq num>'
struct sequence_string : comma::packed::packed_struct< sequence_string, 11 >
{
    sequence_string() {}
    sequence_string( const std::string& cmd, comma::uint16 seq )  { tag = cmd; seq_num = seq; }
    comma::packed::string< 2 > tag; // e.g. GD or MD
    comma::packed::casted< comma::uint16, 8, '0' > seq_num;
    line_feed_t lf;
};

/// This is used to pair the request with the reply, it can roll over
static comma::uint16 sequence = 0;

/// For BM, PP or RB command
struct state_command : public comma::packed::packed_struct< state_command, 14 >
{
    state_command( const std::string& cmd_ ) : message_id( cmd_, ++sequence ) { cmd = cmd_; }
    comma::packed::string< 2 > cmd;
    sequence_string message_id;
    line_feed_t lf_end;
};

struct state_reply : public comma::packed::packed_struct< state_reply, 16 >
{
    comma::packed::string< 2 > cmd;
    sequence_string message_id;
    comma::packed::casted< char, 2, '0' > status;
    char sum;
    line_feed_t lf;
    line_feed_t lf_end; /// TODO: does it work for PP reply?
};


static const int reply_header_size = sensor_to_host::size + sequence_string::size;

/// Can be used for MD or GD request, just change the 'cmd' member
struct request_gd : public comma::packed::packed_struct< request_gd, reply_header_size  >
{
    
    request_gd( bool is_ge=false ) : header( is_ge ? "GD" : "GE"  ), message_id( is_ge ? "GD" : "GE", ++sequence )  {}
    host_to_sensor header;
    sequence_string message_id;
};


/// TODO find out this value
static const char size_of_sum = 1;


/// For calculating the size of the data mixed with checksum and line feeds
template < int STEPS >
struct distance_data {
    /// The data is mixed with check sum every 64 bytes
    static const char encoding_num = 3; // 3 character encoding
    static const unsigned char num_of_sums = (STEPS*encoding_num)/64; // 64 bytes per sum check value
    static const comma::uint16 value = STEPS*encoding_num + num_of_sums*(size_of_sum + 1); /// adding one for line feed

    static const comma::uint16 data_only_size = STEPS*encoding_num; // Has distance and intensity data
    
    typedef boost::array< comma::packed::scip_3chars_t, STEPS > points_t;
    comma::packed::string< value > raw_data;   /// data mixed with checksums and linefeeds
    
    struct rays : public comma::packed::packed_struct< rays, data_only_size >
    {
        points_t steps;
    };
    
    void get_values( rays& points ); 
    
};

template < int STEPS >
struct di_data {
    /// The data is mixed with check sum every 64 bytes
    static const char encoding_num = 3; // 3 character encoding
    static const comma::uint16 data_only_size = STEPS*encoding_num*2; // Has distance and intensity data
    static const unsigned char num_of_sums = (data_only_size)/64; // 64 bytes per sum check value
    static const comma::uint16 value = STEPS*encoding_num + num_of_sums*(size_of_sum + 1); /// adding one for line feed

    /// Store an array of contiguous character data - encoded data
    
    struct pair_t 
    {
        comma::packed::scip_3chars_t distance;
        comma::packed::scip_3chars_t intensity;
    };
    
    typedef boost::array< pair_t, STEPS > points_t;
    comma::packed::string< value > raw_data;   
    
    struct rays : public comma::packed::packed_struct< rays, data_only_size >
    {
        points_t steps;
    };
    
    void get_values( rays& points ); 
};

/// Depends on how many steps, always 3 character encoding
template < int STEPS >
struct reply_gd : comma::packed::packed_struct< reply_gd< STEPS >, reply_header_size + 8 + distance_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_gd request;
    comma::packed::casted< char, 2, '0' > status;
    line_feed_t lf;
    comma::packed::casted< comma::uint16, 4, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = distance_data< STEPS >::value;
    distance_data< STEPS > data;
    line_feed_t lf_end; /// Final terminating line feed
};

template < int STEPS >
struct reply_ge : comma::packed::packed_struct< reply_ge< STEPS >, reply_header_size + 8 + di_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_gd request;
    comma::packed::casted< char, 2, '0' > status;
    line_feed_t lf;
    comma::packed::casted< comma::uint16, 4, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = di_data< STEPS >::value;
    /// each sum value is followed by a line feed char
    di_data< STEPS > data;
    line_feed_t lf_end; /// Final terminating line feed
};

/// Can be used for MD or ME request
struct request_md : comma::packed::packed_struct< request_md, reply_header_size + 1 + 2 >
{
    request_md( bool is_me=false ) : header( is_me ? "MD" : "ME" ), message_id( is_me ? "MD" : "ME", ++sequence ) {}
    host_to_sensor header;
    comma::packed::const_byte< '0' > scan_interval; // skips nothing
    comma::packed::casted< char, 2, '0' > num_of_scans;
    sequence_string message_id;
};

/// This is a reply as an acknowledgement of request - no data
struct reply_md : comma::packed::packed_struct< reply_md, request_md::size + 2 + 2 > /// 1 for last line feed char
{
    request_md request;
    comma::packed::casted< char, 2, '0' > status;   /// Should be '00'
    line_feed_t lf;
    line_feed_t lf_end;
};

/// Depends on how many steps, always 3 character encoding
template < int STEPS >
struct reply_md_data : comma::packed::packed_struct< reply_md_data< STEPS >, request_md::size + 8 + distance_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_md request;
    comma::packed::casted< char, 2, '0' > status;
    line_feed_t lf;
    comma::packed::casted< comma::uint16, 4, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = distance_data< STEPS >::value;
    distance_data< STEPS > data;
    line_feed_t lf_end; /// Final terminating line feed
};

template < int STEPS >
struct reply_me_data : comma::packed::packed_struct< reply_me_data< STEPS >, request_md::size + 8 + di_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_md request;
    comma::packed::casted< char, 2, '0' > status;
    line_feed_t lf;
    comma::packed::casted< comma::uint16, 4, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = di_data< STEPS >::value;
    di_data< STEPS > data;
    line_feed_t lf_end; /// Final terminating line feed
};

    
} } // namespace snark { namespace hokuyo {
    
    
#endif // SNARK_SENSORS_HOKUYO_MESSAGE_H
