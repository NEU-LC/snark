// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
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

#include <glob.h>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <unordered_map>

void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --bags"
        " --fields"
        " --flush"
        " --header --output-header"
        " --header-fields"
        " --header-format"
        " --help -h"
        " --max-datagram-size"
        " --no-discard"
        " --node-name"
        " --output-fields"
        " --output-format"
        " --queue-size"
        " --topic"
        " --verbose -v"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

void usage(bool detail)
{
    std::cerr << std::endl;
    std::cerr << "subscribe to a ROS PointCloud2 topic and output as csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    input: receives messages by subscribing to a ROS topic or reading rosbags" << std::endl;
    std::cerr << "    output: writes data as csv to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "  input" << std::endl;
    std::cerr << "    --bags=[<bags>]: load from rosbags rather than subscribe" << std::endl;
    std::cerr << "    --max-datagram-size: for UDP transport. See ros::TransportHints" << std::endl;
    std::cerr << "    --no-discard: don't discard points with nan or inf in their values"<< std::endl;
    std::cerr << "    --queue-size=[<n>]: ROS Subscriber queue size, default 1" << std::endl;
    std::cerr << "    --topic=<topic>: name of the topic to subscribe to" << std::endl;
    std::cerr << std::endl;
    std::cerr << "  output" << std::endl;
    std::cerr << "    --fields=[<names>]: only output listed fields" << std::endl;
    std::cerr << "    --flush: call flush on stdout after each write" << std::endl;
    std::cerr << "    --header,--output-header: prepend t,block header to output with t,ui format"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "  general" << std::endl;
    std::cerr << "    --header-fields: write csv field names of header to stdout and exit"<< std::endl;
    std::cerr << "    --header-format: write csv format of header to stdout and exit"<< std::endl;
    std::cerr << "    --help,-h: show help; --help --verbose: show more help" << std::endl;
    std::cerr << "    --node-name: node name for this process, when not specified uses" << std::endl;
    std::cerr << "                 ros::init_options::AnonymousName flag" << std::endl;
    std::cerr << "    --output-fields: print field names and exit" << std::endl;
    std::cerr << "    --output-format: print format and exit" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    field names and format are extracted from the first message of the" << std::endl;
    std::cerr << "    subscribed topic" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    view points from a published topic:" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " --topic some_topic --fields x,y,z --binary 3f --header \\" << std::endl;
    std::cerr << "        | view-points --fields t,block,x,y,z --binary t,ui,3f" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    view points from a set of bags:" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " --bags \"*.bag\" --topic some_topic --fields x,y,z --binary 3f --header \\" << std::endl;
    std::cerr << "        | view-points --fields t,block,x,y,z --binary t,ui,3f" << std::endl;
    std::cerr << std::endl;
}

static bool status = 0; // quick and dirty

namespace snark { namespace ros {

static std::vector< std::string > glob_( const std::string& path )
{
    std::vector< std::string > paths;
    // paths.push_back( path );
    glob_t globbuf;
    if( ::glob( &path[0], GLOB_TILDE, NULL, &globbuf ) != 0 ) { std::cerr << "points-from-ros: not found: '" << path << "'" << std::endl; exit( 1 ); }
    for( std::size_t i = 0; i < globbuf.gl_pathc; ++i ) { paths.push_back( std::string( globbuf.gl_pathv[i] ) ); }
    globfree( &globbuf );
    comma::verbose << "processing bags: " << std::endl;
    for( const auto& p: paths ) { comma::verbose << p << std::endl; }
    return paths;
}
    
/// utility functions for ros sensor_msgs::PointCloud2
struct point_cloud
{
private:
    static std::vector<comma::csv::format::types_enum> rmap_data_type;
public:
    static const std::vector<comma::csv::format::types_enum>& get_rmap_data_type()
    {
        if(rmap_data_type.size()==0)
        {
            rmap_data_type.resize(9);
            rmap_data_type.at(sensor_msgs::PointField::INT8)=comma::csv::format::char_t;
            rmap_data_type.at(sensor_msgs::PointField::UINT8)=comma::csv::format::uint8;
            rmap_data_type.at(sensor_msgs::PointField::INT16)=comma::csv::format::int16;
            rmap_data_type.at(sensor_msgs::PointField::UINT16)=comma::csv::format::uint16;
            rmap_data_type.at(sensor_msgs::PointField::INT32)=comma::csv::format::int32;
            rmap_data_type.at(sensor_msgs::PointField::UINT32)=comma::csv::format::uint32;
            rmap_data_type.at(sensor_msgs::PointField::FLOAT32)=comma::csv::format::float_t;
            rmap_data_type.at(sensor_msgs::PointField::FLOAT64)=comma::csv::format::double_t;
        }
        return rmap_data_type;
    }
    static std::size_t size_of_type(comma::csv::format::types_enum t)
    {
        switch(t)
        {
            case comma::csv::format::char_t: return sizeof( char );
            case comma::csv::format::int8: return sizeof( char );
            case comma::csv::format::uint8: return sizeof( unsigned char );
            case comma::csv::format::int16: return sizeof( int16_t );
            case comma::csv::format::uint16: return sizeof( uint16_t );
            case comma::csv::format::int32: return sizeof( int32_t );
            case comma::csv::format::uint32: return sizeof( uint32_t );
            case comma::csv::format::int64: return sizeof( int64_t );
            case comma::csv::format::uint64: return sizeof( uint64_t );
            case comma::csv::format::float_t: return sizeof( float );
            case comma::csv::format::double_t: return sizeof( double );
            case comma::csv::format::time: return sizeof( int64_t );
            case comma::csv::format::long_time: return sizeof( int64_t ) + sizeof( int32_t );
            case comma::csv::format::fixed_string: return 0; // will it blast somewhere?
            default: { COMMA_THROW(comma::exception,"invalid type "<<unsigned(t)); }
        }
    }

    static std::vector< comma::csv::format::types_enum > padding_types( std::size_t num_bytes )
    {
        std::vector< comma::csv::format::types_enum > result;
        std::vector< comma::csv::format::types_enum > candidates = {
            comma::csv::format::uint64,
            comma::csv::format::uint32,
            comma::csv::format::uint16,
            comma::csv::format::uint8
        };
        for( auto candidate: candidates )
        {
            while( num_bytes >= comma::csv::format::size_of( candidate ))
            {
                result.push_back( candidate );
                num_bytes -= comma::csv::format::size_of( candidate );
            }
        }
        return result;
    }

    /// returns list of field names from the message
    static std::string msg_fields_names( const sensor_msgs::PointCloud2::_fields_type& msg_fields
                                       , const std::vector< std::string >& field_filter = std::vector< std::string >() )
    {
        if( !field_filter.empty() ) { return comma::join( field_filter, ',' ); }

        std::string s;
        std::string delimiter;
        std::size_t expected_offset = 0;
        static unsigned int padding_field_count = 0;
        const auto& rmap = get_rmap_data_type();
        for( const auto& f : msg_fields )
        {
            comma::csv::format::types_enum type = rmap.at( f.datatype );
            if( f.offset > expected_offset )
            {
                for( unsigned int i = 0; i < padding_types( f.offset - expected_offset ).size(); ++i )
                {
                    s += delimiter + "padding";
                    if( padding_field_count > 0 ) { s += boost::lexical_cast< std::string >( padding_field_count ); }
                    padding_field_count++;
                }
            }
            s += delimiter + f.name;
            expected_offset = f.offset + comma::csv::format::size_of( type ) * f.count;
            if( delimiter.empty() ) { delimiter = ","; }
        }
        return s;
    }

    /// returns csv format from the message, optionally filtered by field name
    static std::string msg_fields_format( const sensor_msgs::PointCloud2::_fields_type& msg_fields
                                        , const std::vector< std::string >& field_filter = std::vector< std::string >() )
    {
        std::string s;
        std::string delimiter;
        std::size_t expected_offset = 0;
        bool add_field;
        const auto& rmap = get_rmap_data_type();
        for( const auto& f : msg_fields )
        {
            comma::csv::format::types_enum type = rmap.at( f.datatype );
            if( field_filter.empty() )
            {
                if( f.offset > expected_offset )
                {
                    for( auto t: padding_types( f.offset - expected_offset ))
                    {
                        s += delimiter + comma::csv::format::to_format( t );
                    }
                }
                expected_offset = f.offset + comma::csv::format::size_of( type ) * f.count;
                add_field = true;
            }
            else
            {
                add_field = ( std::find( field_filter.begin(), field_filter.end(), f.name ) != field_filter.end());
            }
            if( add_field )
            {
                s += delimiter + ( f.count > 1 ? boost::lexical_cast< std::string >( f.count ) : "" )
                    + comma::csv::format::to_format( type );
                if( delimiter.empty() ) { delimiter = ","; }
            }
        }
        return s;
    }

    struct bin_base
    {
        virtual ~bin_base() { }
        virtual const char* get(const char* data)=0;
        virtual std::size_t size() const=0;
    };
    struct bin_cat : public bin_base
    {
        uint32_t size_;
        bin_cat(uint32_t s=0) : size_(s) { }
        const char* get(const char* data){ return data; }
        std::size_t size() const { return size_; }
    };
    /// copy specified fields from a point record, given msg point field info and a list of field names
    struct bin_shuffle : public bin_base
    {
        /// prepare 
        bin_shuffle(const std::string& field_names,const sensor_msgs::PointCloud2::_fields_type& msg_fields)
        {
            std::vector<std::string> fields=comma::split(field_names,",");
            std::unordered_map<std::string,unsigned> msg_field_name_map;
            std::vector<range_t> elements;
            const auto& rmap=get_rmap_data_type();
            std::size_t size=0;
            for(std::size_t i=0;i<msg_fields.size();i++)
            {
                msg_field_name_map[msg_fields[i].name]=i;
                if(msg_fields[i].datatype<1 || msg_fields[i].datatype>=rmap.size()) { COMMA_THROW(comma::exception,"datatype out of range (1 to 8) "<<unsigned(msg_fields[i].datatype)); }
                comma::csv::format::types_enum type=rmap.at(msg_fields[i].datatype);
                //using comma::csv::format::size_of(type) corrupts stack
//                 elements.push_back(range_t(msg_fields[i].offset,msg_fields[i].count*comma::csv::format::size_of(type)));
                elements.push_back(range_t(msg_fields[i].offset,msg_fields[i].count*point_cloud::size_of_type(type)));
            }
            for(const auto& f : fields)
            {
                unsigned index=msg_field_name_map.at(f);
//                 comma::verbose<<"bin_shuffle "<<f<<"; "<<index<<"; "<<elements[index].first<<","<<elements[index].second<<std::endl;
                ranges.push_back(elements[index]);
                size+=elements[index].second;
            }
            buf.resize(size);
        }
        /// shuffle
        const char* get(const char* data)
        {
            std::size_t offset=0;
            for(const auto& i : ranges)
            {
                std::memcpy(&buf[offset],data+i.first,i.second);
                offset+=i.second;
            }
            return buf.data();
        }
        bool empty() const { return ranges.empty(); }
        std::size_t size() const { return buf.size(); }
    private:
        std::vector<char> buf;
        //first: offset, second: size
        typedef typename std::pair<std::size_t,std::size_t> range_t;
        std::vector<range_t> ranges;
    };
};

std::vector<comma::csv::format::types_enum> point_cloud::rmap_data_type;

} } // namespace snark { namespace ros {


struct header
{
    boost::posix_time::ptime t;
    uint32_t block;
    header() : block(0) { }
    header(const ros::Time& time,uint32_t seq) : t(time.toBoost()), block(seq) { }
};

namespace comma { namespace visiting {

template <> struct traits< header >
{
    template< typename K, typename V > static void visit( const K& k, const header& p, V& v )
    {
        v.apply( "t", p.t );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {


/// process ros sensor_msgs::PointCloud2 message
struct points
{
    std::vector< std::string > fields;
    comma::csv::format format;
    comma::csv::options csv;
    bool output_fields;
    bool output_format;
    bool flush;
    bool write_header;
    bool discard;

    points( const comma::command_line_options& options )
        : csv( options )
        , output_fields( options.exists( "--output-fields" ))
        , output_format( options.exists( "--output-format" ))
        , flush( options.exists( "--flush" ))
        , write_header( options.exists( "--header,--output-header" ))
        , discard( !options.exists( "--no-discard" ))
    {
        fields = comma::split( options.value< std::string >( "--fields", "" ), ',' );
        if( fields.size() == 1 && fields[0].empty() ) { fields.clear(); } // comma::split quirk
    }

    void process(const sensor_msgs::PointCloud2ConstPtr input)
    {
        try
        {
            if( output_fields )
            { 
                if( write_header ) { std::cout << comma::join( comma::csv::names< header >(), ',') << ","; }
                std::cout << snark::ros::point_cloud::msg_fields_names( input->fields, fields ) << std::endl;
                ros::shutdown();
                return;
            }
            if( output_format )
            { 
                if( write_header ) { std::cout << comma::csv::format::value< header >() << ","; }
                std::cout << snark::ros::point_cloud::msg_fields_format( input->fields, fields ) << std::endl;
                ros::shutdown();
                return;
            }
            if( format.count() == 0 )
            {
                format = comma::csv::format( snark::ros::point_cloud::msg_fields_format( input->fields, fields ));
                comma::verbose << "setting format to " << format.string() << std::endl;
            }
            unsigned count=input->width*input->height;
            unsigned record_size=input->point_step;
            ::header header(input->header.stamp,input->header.seq);
            
            std::unique_ptr<snark::ros::point_cloud::bin_base> bin;
            if(csv.fields.empty()) { bin.reset(new snark::ros::point_cloud::bin_cat(record_size)); }
            else { bin.reset(new snark::ros::point_cloud::bin_shuffle(csv.fields,input->fields)); }

            bin_writer writer;
            
            std::unique_ptr<filter_base> filter;
            if(discard) { filter.reset(new float_filter(format)); }
            
            for(unsigned i=0;i<count;i++)
            {
                const char* buf=bin->get(reinterpret_cast<const char*>(&input->data[i*record_size]));
                if(!filter || filter->valid(buf,bin->size()))
                {
                    if(write_header) { writer.write_header(header); }
                    writer.write(buf,bin->size());
                    if(flush) { std::cout.flush(); }
                }
                if( !std::cout.good() ) { ros::shutdown(); break; }
            }
        }
        catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; status = 1; ros::shutdown(); }
        catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; status = 1; ros::shutdown(); }
    }

private:
    struct bin_writer
    {
        comma::csv::binary<header> header_csv_bin;
        std::vector<char> header_buf;
        bin_writer() : header_buf(header_csv_bin.format().size()) { }
        void write_header(const header& h)
        {
            std::cout.write(header_csv_bin.put(h,&header_buf[0]),header_buf.size());
        }
        void write(const char* buf,uint32_t size)
        {
            std::cout.write(buf,size);
        }
    };

    struct filter_base
    {
        virtual ~filter_base() { }
        virtual bool valid(const char* buf,uint32_t size)=0;
    };
    struct float_filter : public filter_base
    {
        const comma::csv::format& format;
        std::vector<std::size_t> offsets;
        float_filter(const comma::csv::format& format) : format(format)
        {
            offsets.reserve(format.elements().size());
            for(const auto& i : format.elements() )
            {
                if(i.type==comma::csv::format::float_t)
                {
                    if(i.count!=1) { COMMA_THROW(comma::exception, "expected format count 1, got "<<i.count); }
                    if(i.size!=sizeof(float)) { COMMA_THROW(comma::exception, "expected format size "<<sizeof(float)<<"; got"<<i.size); }
                    offsets.push_back(i.offset);
                }
            }
        }
        bool valid(const char* buf,uint32_t size)
        {
            for(const std::size_t offset : offsets)
            {
                if(offset+sizeof(float)>size) { COMMA_THROW(comma::exception,"offset out of range. offset: "<<offset<<" size: "<<size); }
                float value=*reinterpret_cast<const float*>(buf+offset);
                if(std::isnan(value)||std::isinf(value))
                    return false;
            }
            return true;
        }
    };
};

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" )) bash_completion( argc, argv );
        if(options.exists("--header-fields")) { std::cout<<comma::join( comma::csv::names<header>(),',')<<std::endl; return 0; }
        if(options.exists("--header-format")) { std::cout<< comma::csv::format::value<header>()<<std::endl; return 0; }
        std::string bags_option = options.value<std::string>( "--bags", "" );
        std::string topic=options.value<std::string>("--topic");
        unsigned queue_size=options.value<unsigned>("--queue-size",1);
        boost::optional<int> max_datagram_size=options.optional<int>("--max-datagram-size");
        boost::optional<std::string> node_name=options.optional<std::string>("--node-name");
        uint32_t node_options=0;
        if(!node_name)
        {
            node_name="points_from_ros";
            node_options=ros::init_options::AnonymousName;
        }
        int arrrgc=1;
        ros::init(arrrgc, argv,*node_name,node_options);
        points points(options);
        if( !bags_option.empty() )
        {
            rosbag::Bag bag;
            std::vector< std::string > bag_names;
            for( auto name: comma::split( bags_option, ',' ))
            {
                std::vector< std::string > expansion = snark::ros::glob_( name );
                bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
            }
            for( auto bag_name: bag_names )
            {
                comma::verbose << "opening " << bag_name << std::endl;
                bag.open( bag_name );
                for( rosbag::MessageInstance const m: rosbag::View( bag, rosbag::TopicQuery( topic )))
                {
                    sensor_msgs::PointCloud2ConstPtr msg = m.instantiate< sensor_msgs::PointCloud2 >();
                    points.process( msg );
                    if( ros::isShuttingDown() ) { break; }
                }
                bag.close();
                if( ros::isShuttingDown() ) { break; }
            }
        }
        else
        {
            if( !ros::master::check() ) { std::cerr << comma::verbose.app_name() << ": roscore appears not to be running, node will wait for it to start" << std::endl; }
            ros::NodeHandle ros_node;
            ros::TransportHints transport_hints;
            if(max_datagram_size) {transport_hints=ros::TransportHints().maxDatagramSize(*max_datagram_size);}
            ros::Subscriber subsriber=ros_node.subscribe(topic,queue_size,&points::process,&points,transport_hints);
            ros::spin();
        }
        return status;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
