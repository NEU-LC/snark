#ifndef SNARK_SENSORS_HOKUYO_MESSAGE_H
#define SNARK_SENSORS_HOKUYO_MESSAGE_H
#include <comma/base/types.h>
#include <comma/packed/packed.h>

namespace snark { namespace hokuyo {
    
/// This is the header for host to sensor message
/// CMD(2) + Start Step(4) + End Step(4) + Cluster Count(2)
struct header : public comma::packed::packed_struct< header, 12  > {
    comma::packed::string< 2 > cmd;
    comma::packed::casted< comma::uint16, 4, '0' > start_step;
    comma::packed::casted< comma::uint16, 4, '0' > end_step;
    comma::packed::casted< char, 2, '0' > cluster_count;
};

struct host_to_sensor : public header {};
struct sensor_to_host : public header {};
    
typedef comma::packed::const_byte< '\n' > line_feed_t; /// Final terminating line feed

/// This is the string ID of request, making it '<cmd><seq num>'
struct sequence_string : comma::packed::packed_struct< sequence_string, 11 >
{
    comma::packed::string< 2 > tag; // e.g. GD or MD
    comma::packed::casted< comma::uint16, 8, '0' > seq_num;
    line_feed_t lf;
};

static const int id_size = sequence_string::size;
static const int reply_header_size = sensor_to_host::size + sequence_string::size;

/// Can be used for MD or GD request, just change the 'cmd' member
struct request_gd : public comma::packed::packed_struct< request_gd, reply_header_size  >
{
    host_to_sensor header;
    sequence_string message_id;
};

/// TODO find out this value
static const char size_of_sum = 2;


/// For calculating the size of the data mixed with checksum and line feeds
template < int STEPS >
struct distance_data {
    /// The data is mixed with check sum every 64 bytes
    static const char encoding_num = 3; // 3 character encoding
    static const unsigned char num_of_sums = (STEPS*encoding_num)/64; // 64 bytes per sum check value
    static const comma::uint16 value = STEPS*encoding_num + num_of_sums*(size_of_sum + 1); /// adding one for line feed
};

template < int STEPS >
struct di_data {
    /// The data is mixed with check sum every 64 bytes
    static const char encoding_num = 3; // 3 character encoding
    static const comma::uint16 data_only_size = STEPS*encoding_num*2; // Has distance and intensity data
    static const unsigned char num_of_sums = (data_only_size)/64; // 64 bytes per sum check value
    static const comma::uint16 value = STEPS*encoding_num + num_of_sums*(size_of_sum + 1); /// adding one for line feed
};

/// Depends on how many steps, always 3 character encoding
template < int STEPS >
struct reply_gd : comma::packed::packed_struct< reply_gd< STEPS >, reply_header_size + 12 + distance_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_gd request;
    comma::packed::string< 2, '0' > status;
    line_feed_t lf;
    comma::packed::string< 8, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = distance_data< STEPS >::value;
    comma::packed::string< data_size > data;   
    line_feed_t lf_end; /// Final terminating line feed
};

template < int STEPS >
struct reply_ge : comma::packed::packed_struct< reply_ge< STEPS >, reply_header_size + 12 + di_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_gd request;
    comma::packed::string< 2, '0' > status;
    line_feed_t lf;
    comma::packed::string< 8, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = di_data< STEPS >::value;
    comma::packed::string< data_size > data;   /// each sum value is followed by a line feed char
    line_feed_t lf_end; /// Final terminating line feed
};

/// Can be used for MD or ME request
struct request_md : comma::packed::packed_struct< request_md, reply_header_size + 1 + 2 >
{
    host_to_sensor header;
    comma::packed::const_byte< '0' > scan_interval; // skips nothing
    comma::packed::casted< char, 2, '0' > num_of_scans;
    sequence_string message_id;
};

/// This is a reply as an acknowledgement of request - no data
struct reply_md : comma::packed::packed_struct< reply_md, request_md::size + 2 + 1 > /// 1 for last line feed char
{
    request_md request;
    comma::packed::string< 2, '0' > status; /// Should be '00'
    line_feed_t lf_end;
};

/// Depends on how many steps, always 3 character encoding
template < int STEPS >
struct reply_md_data : comma::packed::packed_struct< reply_md_data< STEPS >, request_md::size + 12 + distance_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_md request;
    comma::packed::string< 2, '0' > status;
    line_feed_t lf;
    comma::packed::string< 8, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = distance_data< STEPS >::value;
    comma::packed::string< data_size > data;   /// each sum value is followed by a line feed char
    line_feed_t lf_end; /// Final terminating line feed
};

template < int STEPS >
struct reply_me_data : comma::packed::packed_struct< reply_me_data< STEPS >, request_md::size + 12 + di_data< STEPS >::value + 1 > /// 1 for last line feed char
{
    request_md request;
    comma::packed::string< 2, '0' > status;
    line_feed_t lf;
    comma::packed::string< 8, '0' > timestamp; // just a count from 0
    line_feed_t lf1;

    static const int data_size = di_data< STEPS >::value;
    comma::packed::string< data_size > data;   /// each sum value is followed by a line feed char
    line_feed_t lf_end; /// Final terminating line feed
};

    
} } // namespace snark { namespace hokuyo {
    
    
#endif // SNARK_SENSORS_HOKUYO_MESSAGE_H
