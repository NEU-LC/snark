// This file is part of comma, a generic and flexible library
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

#include <vector>
#include <unordered_map>
#include <iostream>
#include <comma/application/verbose.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/csv/stream.h>
// #include <chrono>
// #include <thread>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/csv/traits.h>
#include <cmath>

void usage(bool detail)
{
    std::cerr<<"    subscribe to a ROS PointCloud2 topic and output as csv" << std::endl;
    std::cerr<<"        input: recieves messages by subscribing to a ROS topic, " << std::endl;
    std::cerr<<"        output: writes data as csv to stdout" << std::endl;
    std::cerr<<"        message format and field names are take from ros message, csv options are for output only" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --no-discard; don't discard points with nan or inf in their values"<< std::endl;
    std::cerr << "    --flush; call flush on stdout after each write (otherwise may buffer with small size messages)" << std::endl;
    std::cerr << "    --header,--output-header; prepend t,block header to output with t,ui format"<< std::endl;
    std::cerr << "    --header-fields; write csv field names of header to std out and exit"<< std::endl;
    std::cerr << "    --header-format; write csv format of header to std out and exit"<< std::endl;
    std::cerr << "    --help,-h: show help; --help --verbose: show more help" << std::endl;
    std::cerr << "    --max-datagram-size: If a UDP transport is used, specifies the maximum datagram size (see ros::TransportHints)" << std::endl;
    std::cerr << "    --node-name: node name for this process, when not specified uses ros::init_options::AnonymousName flag" << std::endl;
    std::cerr << "    --output-fields; print field names and exit" << std::endl;
    std::cerr << "    --output-format; print format and exit" << std::endl;
    std::cerr << "    --queue-size=[<n>]: ROS Subscriber queue size, default 1" << std::endl;
    std::cerr << "    --topic=<topic>: name of the topic to subscribe to" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << std::endl;
    std::cerr << "field names and format are extracted from the first message of the subscribed topic; following options wait until they receive a message" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    csv options" << std::endl;
    std::cerr << "        either --format or --binary option must be specified" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail on csv options" << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points from a topic:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --topic some_topic --fields t,block,x,y,z --binary t,ui,3f | view-points --fields t,block,x,y,z --binary t,ui,3f" << std::endl;
    std::cerr << std::endl;
}

namespace snark { namespace ros {

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
    /// returns list of field names from the message
    static std::string msg_fields_names(const sensor_msgs::PointCloud2::_fields_type& msg_fields)
    {
        std::string s;
        std::string delimiter;
        for(const auto& i : msg_fields)
        {
            s+=delimiter+i.name;
            if(delimiter.empty()) { delimiter=","; }
        }
        return s;
    }
    /// returns csv format elements from the message
    static std::vector<comma::csv::format::element> msg_fields_format_elements(const sensor_msgs::PointCloud2::_fields_type& msg_fields)
    {
        std::vector<comma::csv::format::element> elements;
        const auto& rmap=get_rmap_data_type();
        for(const auto& i : msg_fields)
        {
            comma::csv::format::types_enum type=rmap.at(i.datatype);
            elements.push_back(comma::csv::format::element(i.offset,i.count,i.count*comma::csv::format::size_of(type),type));
        }
        return elements;
    }
    static std::string msg_fields_format(const sensor_msgs::PointCloud2::_fields_type& msg_fields)
    {
        std::string s;
        std::string delimiter;
        const auto& rmap=get_rmap_data_type();
        for(const auto& i : msg_fields)
        {
            s+=delimiter+comma::csv::format::to_format(rmap.at(i.datatype));
            if(delimiter.empty()) { delimiter=","; }
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
    comma::csv::format format;
    bool ascii;
    comma::csv::options csv;
    bool output_fields;
    bool output_format;
    bool flush;
    bool write_header;
    bool discard;
    points(const comma::command_line_options& options) : ascii(false), csv(options) , 
        output_fields(options.exists("--output-fields")),
        output_format(options.exists("--output-format")),
        flush(options.exists("--flush")),
        write_header(options.exists("--header,--output-header")),
        discard(!options.exists("--no-discard"))
    {
        if( !output_fields && !output_format)
        {
            ascii= !csv.binary();
            if(ascii && !options.exists("--format")) { COMMA_THROW( comma::exception, "please specify either --format for ascii or --binary"); }
            format=comma::csv::format(comma::csv::format(options.value<std::string>(ascii?"--format":"--binary,-b")).expanded_string());
        }
    }
    void process(const sensor_msgs::PointCloud2ConstPtr input);
private:
    struct writer_base
    {
        virtual ~writer_base() { }
        virtual void write_header(const header& h)=0;
        virtual void write(const char* buf,uint32_t size)=0;
    };
    struct csv_writer : public writer_base
    {
        const comma::csv::options& csv;
        const comma::csv::format& format;
        comma::csv::ascii<header> header_csv_ascii;
        csv_writer(const comma::csv::options& csv,const comma::csv::format& format) : csv(csv), format(format), header_csv_ascii("t,block",csv.delimiter) { }
        void write_header(const header& h)
        {
            std::cout<<header_csv_ascii.put(h)<<",";
        }
        void write(const char* buf,uint32_t size)
        {
            std::string bin_data=format.bin_to_csv(buf,csv.delimiter,csv.precision);
            std::cout.write(bin_data.data(),bin_data.size())<<std::endl;
        }
    };
    struct bin_writer : public writer_base
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

void points::process(const sensor_msgs::PointCloud2ConstPtr input)
{
    try
    {
        if(output_fields)
        { 
            std::cout<<snark::ros::point_cloud::msg_fields_names(input->fields)<<std::endl;
            ros::shutdown();
            return;
        }
        if(output_format)
        { 
            std::cout<<snark::ros::point_cloud::msg_fields_format(input->fields)<<std::endl;
            ros::shutdown();
            return;
        }
        unsigned count=input->width*input->height;
        unsigned record_size=input->point_step;
        ::header header(input->header.stamp,input->header.seq);
        
        std::unique_ptr<snark::ros::point_cloud::bin_base> bin;
        if(csv.fields.empty()) { bin.reset(new snark::ros::point_cloud::bin_cat(record_size)); }
        else { bin.reset(new snark::ros::point_cloud::bin_shuffle(csv.fields,input->fields)); }
        
        std::unique_ptr<writer_base> writer;
        if(ascii) { writer.reset(new csv_writer(csv,format)); }
        else { writer.reset(new bin_writer()); }
        
        std::unique_ptr<filter_base> filter;
        if(discard) { filter.reset(new float_filter(format)); }
        
        for(unsigned i=0;i<count;i++)
        {
            const char* buf=bin->get(reinterpret_cast<const char*>(&input->data[i*record_size]));
            if(!filter || filter->valid(buf,bin->size()))
            {
                if(write_header) { writer->write_header(header); }
                writer->write(buf,bin->size());
                if(flush) { std::cout.flush(); }
            }
            if( !std::cout.good() ) { ros::shutdown(); break; }
        }
        return;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; ros::shutdown(); }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; ros::shutdown(); }
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if(options.exists("--header-fields")) { std::cout<<comma::join( comma::csv::names<header>(),',')<<std::endl; return 0; }
        if(options.exists("--header-format")) { std::cout<< comma::csv::format::value<header>()<<std::endl; return 0; }
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
        ros::NodeHandle ros_node;
        points points(options);
        ros::TransportHints transport_hints;
        if(max_datagram_size) {transport_hints=ros::TransportHints().maxDatagramSize(*max_datagram_size);}
        ros::Subscriber subsriber=ros_node.subscribe(topic,queue_size,&points::process,&points,transport_hints);
        ros::spin();
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
