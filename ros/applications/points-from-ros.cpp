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
    std::cerr << "    --help,-h: show help; --help --verbose: show more help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --topic=<topic>: name of the topic to subscribe to" << std::endl;
    std::cerr << "    --queue-size=[<n>]: ROS Subscriber queue size, default 1" << std::endl;
    std::cerr << "field names and format are extracted from the first message of the subscribed topic; following options wait until they receive a message" << std::endl;
    std::cerr << "    --output-fields; print field names and exit" << std::endl;
    std::cerr << "    --output-format; print format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    csv options --fields must be specified" << std::endl;
    std::cerr << "    either --format or --binary option must be specified" << std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "example" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    to view points from a topic:" << std::endl;
    std::cerr << "        " << comma::verbose.app_name() << "  --topic some_topic --fields t,block,x,y,z --binary t,ui,3f | view-points --fields t,block,x,y,z --binary t,ui,3f" << std::endl;
    std::cerr << std::endl;
}

namespace snark { namespace ros {

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
    /// copy specified fields from a point record, given msg point field info and a list of field names
    struct bin_shuffle
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
                comma::csv::format::types_enum type=rmap.at(msg_fields[i].datatype);
                elements.push_back(range_t(msg_fields[i].offset,msg_fields[i].count*comma::csv::format::size_of(type)));
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
        const char* shuffle(const char* data)
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

struct points
{
    comma::csv::format format;
    bool ascii;
    comma::csv::options csv;
    bool output_fields;
    bool output_format;
    points(const comma::command_line_options& options) : csv(options)
    {
        if(!csv.binary())
        {
            if(!options.exists("--format")) { COMMA_THROW( comma::exception, "please specify either --format for ascii or --binary"); }
            ascii=true;
            format=comma::csv::format(comma::csv::format(options.value<std::string>("--format")).expanded_string());
        }
        output_fields=options.exists("--output-fields");
        output_format=options.exists("--output-format");
    }
    void process(const sensor_msgs::PointCloud2ConstPtr& input)
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
            comma::verbose<<record_size<<"; "<<input->header.seq<<"; "<<input->header.stamp<<std::endl;
            if(csv.fields.empty())
            {
                for(unsigned i=0;i<count;i++)
                {
                    const char* buf=reinterpret_cast<const char*>(&input->data[i*record_size]);
                    if(ascii)
                    {
                        std::string bin_data=format.bin_to_csv(buf,csv.delimiter,csv.precision);
                        std::cout.write(bin_data.data(),bin_data.size())<<std::endl;
                    }
                    else
                    {
                        std::cout.write(buf,record_size);
                    }
                    if(!std::cout.good())
                    {
                        ros::shutdown();
                        break;
                    }
                }
            }
            else    //shuffle to select field names
            {
                //get field names and build shuffle
                snark::ros::point_cloud::bin_shuffle bin_shuffle(csv.fields,input->fields);
                for(unsigned i=0;i<count;i++)
                {
                    const char* buf=bin_shuffle.shuffle(reinterpret_cast<const char*>(&input->data[i*record_size]));
                    if(ascii)
                    {
                        std::string bin_data=format.bin_to_csv(buf,csv.delimiter,csv.precision);
                        std::cout.write(bin_data.data(),bin_data.size())<<std::endl;
                    }
                    else
                    {
                        std::cout.write(buf,bin_shuffle.size());
                    }
                    if(!std::cout.good())
                    {
                        ros::shutdown();
                        break;
                    }
                }
            }
        }
        catch( std::exception& ex )
        {
            std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
            ros::shutdown();
        }
        catch( ... )
        {
            std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
            ros::shutdown();
        }
    }
};

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        std::string topic=options.value<std::string>("--topic");
        unsigned queue_size=options.value<unsigned>("--queue-size",1);
        int arrrgc=1;
        ros::init(arrrgc, argv,"points_from_ros");
        ros::NodeHandle ros_node;
        points points(options);
        ros::Subscriber subsriber=ros_node.subscribe(topic,queue_size,&points::process,&points);
        ros::spin();
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
