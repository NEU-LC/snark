// This file is part of comma, a generic and flexible library
// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd
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

#include <iostream>
#include <comma/application/verbose.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/base/exception.h>
#include <comma/csv/format.h>
#include <comma/csv/stream.h>
#include <chrono>
#include <thread>

#include "ros/ros.h"
#include <rosbag/bag.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"

void bash_completion( unsigned const ac, char const * const * av )
{
    static const char* completion_options =
        " --all"
        " --hang-on --stay"
        " --help -h"
        " --frame"
        " --latch"
        " --node-name"
        " --pass-through --pass"
        " --queue-size"
        " --topic"
        " --verbose -v"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

void usage( bool detail )
{
    std::cerr << "\npublish csv to ROS PointCloud2";
    std::cerr << "\n";
    std::cerr << "\n    input: reads csv from stdin";
    std::cerr << "\n    output: publish as sensor_msg::PointCloud2 on the specified topic in ROS";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " [<options>]";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:             show help; --help --verbose: show more help";
    std::cerr << "\n    --verbose,-v:          show detailed messages";
    std::cerr << "\n    --all:                 send all records as one ros message";
    std::cerr << "\n    --fields,-f=<fields>:  fields names; default=x,y,z";
    std::cerr << "\n    --frame=[<frame>]:     ros message frame as string";
    std::cerr << "\n    --hang-on,--stay:      wait before exiting so that subscribers can receive";
    std::cerr << "\n                           the last message";
    std::cerr << "\n    --latch:               last message will be saved for future subscribers";
    std::cerr << "\n    --node-name:           default=ros::init_options::AnonymousName flag";
    std::cerr << "\n    --output,-o=[<bag>]:   write to bag rather than publish";
    std::cerr << "\n    --pass-through,--pass: pass input data to stdout";
    std::cerr << "\n    --queue-size=[<n>]:    ROS publisher queue size, default=1";
    std::cerr << "\n    --topic=<topic>:       name of topic to publish to";
    std::cerr << "\n";
    std::cerr << "\nfields:";
    std::cerr << "\n    All input fields are placed in the data field of the PointCloud2 message.";
    std::cerr << "\n    If they are present, t and block fields also populate the header.";
    std::cerr << "\n    Either --format or --binary option must be specified";
    std::cerr << "\n";
    std::cerr << "\nROS message:";
    std::cerr << "\n    By default, one ROS message is published per input record.";
    std::cerr << "\n    If a block field is present then one message is pubished per block.";
    std::cerr << "\n    If the --all option is present all records are published as one message.";
    std::cerr << "\n";
    if( detail )
    {
        std::cerr << "\ncsv options:\n";
        std::cerr << comma::csv::options::usage();
    }
    else
    {
        std::cerr << "\nuse -v or --verbose to see more detail on csv options";
        std::cerr << "\n";
    }
    std::cerr << "\nexamples:";
    std::cerr << "\n    csv-random make --type 3d | csv-paste line-number - \\";
    std::cerr << "\n        | csv-blocks group --fields scalar --span 1000 | csv-time-stamp \\";
    std::cerr << "\n        | " << comma::verbose.app_name() << " --topic /points -f t,id,x,y,z,block --format t,ui,3d,ui";
    std::cerr << "\n";
    std::cerr << "\n    cat data.bin | " << comma::verbose.app_name() << " --topic /points -f t,block,x,y,z -b t,ui,3f";
    std::cerr << "\n" << std::endl;
}

namespace snark { namespace ros {

struct point_cloud
{
    std::vector<sensor_msgs::PointField> point_fields;
    static unsigned map_data_type(comma::csv::format::types_enum t)
    {
        switch(t)
        {
            case comma::csv::format::char_t: 
            case comma::csv::format::int8:
                return sensor_msgs::PointField::INT8;
            case comma::csv::format::uint8:
                return sensor_msgs::PointField::UINT8;
            case comma::csv::format::int16:
                return sensor_msgs::PointField::INT16;
            case comma::csv::format::uint16:
                return sensor_msgs::PointField::UINT16;
            case comma::csv::format::int32:
                return sensor_msgs::PointField::INT32;
            case comma::csv::format::uint32:
                return sensor_msgs::PointField::UINT32;
            case comma::csv::format::float_t:
                return sensor_msgs::PointField::FLOAT32;
            case comma::csv::format::double_t:
                return sensor_msgs::PointField::FLOAT64;
            case comma::csv::format::int64:
            case comma::csv::format::uint64:
            case comma::csv::format::time:
                std::cerr<<"warning: ROS PointCloud2 doesn't support data type '"<<comma::csv::format::to_format(t)<<"', using FLOAT64 instead"<<std::endl;
                return sensor_msgs::PointField::FLOAT64;
            default:
//                 comma::csv::format::
//                 , long_time, fixed_string };
            { COMMA_THROW( comma::exception, "data type not supported: "<<comma::csv::format::to_format(t)); }
        }
    }
    std::size_t data_size;
    point_cloud() { }
    point_cloud(const std::string& fields, const std::string& format_str)
    {
        comma::csv::format format(format_str);
        auto vf=comma::split(fields,',');
        data_size=format.size();
        comma::verbose<<"data_size: "<<data_size<<std::endl;
        const auto& elements=format.elements();
        if(vf.size()!=elements.size()) { COMMA_THROW( comma::exception, "size of fields and binary mismatch: "<<vf.size()<< " vs "<<elements.size()); }
        point_fields.reserve(vf.size());
        for(unsigned i=0;i<vf.size();i++)
        {
            sensor_msgs::PointField pf;
            pf.name=vf[i];
            pf.offset=elements[i].offset;
            pf.datatype=map_data_type(elements[i].type);
            pf.count=elements[i].count;
            point_fields.push_back(pf);
            //update offset
//             comma::verbose<<"point field:  "<<pf<<std::endl;
        }
    }
    /// allocate an empty message
    /// @param count number of records in one frame/block
    sensor_msgs::PointCloud2 create_msg(unsigned count)
    {
        sensor_msgs::PointCloud2 msg;
        msg.height=1;
        msg.width=count;
        msg.point_step=data_size;
        msg.row_step=data_size*count;
        msg.fields=point_fields;
        msg.data.resize(count*data_size);
        return msg;
    }
};

} } // namespace snark { namespace ros {

struct record
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    std::vector<char> data;
    record() : block(0) { }
};

namespace comma { namespace visiting {

template <> struct traits< record >
{
    template< typename K, typename V > static void visit( const K&, const record& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
    }
    
    template< typename K, typename V > static void visit( const K&, record& r, V& v )
    {
        v.apply( "t", r.t );
        v.apply( "block", r.block );
    }
};

} } // namespace comma { namespace visiting {
    
struct points
{
    std::vector<record> records;
    comma::csv::format format;
    snark::ros::point_cloud u;
    std::size_t data_size;
    bool ascii;
    //const comma::command_line_options& options

    points( const comma::csv::options& csv, const std::string& format_str )
        : format( format_str )
        , u( csv.fields, format.expanded_string() )
        , data_size( format.size() )
        , ascii( !csv.binary() )
    {}

    void send( ros::Publisher* publisher, rosbag::Bag* bag, const std::string& topic, const std::string& frame_id )
    {
        //create msg
        sensor_msgs::PointCloud2 msg=u.create_msg(records.size());
        //fill in
        msg.header.stamp=::ros::Time::fromBoost(records[0].t);
        msg.header.seq=records[0].block;
        msg.header.frame_id=frame_id;
        std::size_t offset=0;
        for(const auto& i : records)
        {
            std::memcpy(&msg.data[offset],&i.data[0],data_size);
            offset+=data_size;
        }
        //send
        if( publisher )
        {
            publisher->publish(msg);
            ros::spinOnce();
        }
        // save to bag
        if( bag ) { bag->write( topic, msg.header.stamp, msg ); }
        records.clear();
    }

    void push_back(const comma::csv::input_stream<record>& is, const record& p)
    {
        records.push_back(p);
        records.back().data.resize(data_size);
        if(ascii)
        {
            std::string buf=format.csv_to_bin(is.ascii().last());
            if(buf.size()!=data_size) { COMMA_THROW(comma::exception,"csv_to_bin size mismatch "<<buf.size()<<"; "<<data_size);} 
            std::memcpy(&records.back().data[0],buf.data(),data_size);
        }
        else
        {
            std::memcpy(&records.back().data[0],is.binary().last(),data_size);
        }
    }
};
    
int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        comma::csv::options csv(options);
        if( !csv.binary() && !options.exists( "--format" )) { COMMA_THROW( comma::exception, "please specify --binary=<format>, or --format=<format> for ascii"); }
        csv.full_xpath=true;
        std::string topic=options.value<std::string>("--topic");
        unsigned queue_size=options.value<unsigned>("--queue-size",1);
//         comma::csv::format format(csv.binary() ? csv.format() : options.value<std::string>("--format"));
        if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
        bool has_block=csv.has_field("block");
        bool all=options.exists("--all");
        std::string frame_id=options.value<std::string>("--frame","");
        boost::optional<std::string> node_name=options.optional<std::string>("--node-name");
        bool pass_through=options.exists("--pass-through,--pass");
        std::string output_option = options.value< std::string >( "--output,-o", "" );
        bool publishing = output_option.empty();

        std::unique_ptr< ros::NodeHandle > ros_node;
        std::unique_ptr< ros::Publisher > publisher;
        std::unique_ptr< rosbag::Bag > bag;

        if( publishing )
        {
            int arrrgc = 1;
            uint32_t node_options = 0;
            if( !node_name )
            {
                node_name = "points_to_ros";
                node_options = ros::init_options::AnonymousName;
            }
            ros::init( arrrgc, argv, *node_name, node_options );
            ros_node.reset( new ros::NodeHandle );
            publisher.reset( new ros::Publisher( ros_node->advertise< sensor_msgs::PointCloud2 >( topic, queue_size, options.exists( "--latch" ))));
            ros::spinOnce();
        }
        else
        {
            bag.reset( new rosbag::Bag );
            bag->open( output_option, rosbag::bagmode::Write );
        }
        comma::csv::input_stream<record> is(std::cin, csv);
        comma::csv::passed<record> passed(is,std::cout,csv.flush);
        unsigned block=0;
        points points(csv,csv.binary() ? csv.format().string() : options.value<std::string>("--format"));
        while(std::cin.good())
        {
            //read binary from input
            const record* p=is.read();
            if ( ( !p || block != p->block) && !points.records.empty())
            {
                //send the message
                points.send( publisher.get(), bag.get(), topic, frame_id );
            }
            if( !p ) { break; }
            if(pass_through) { passed.write(); }
            block=p->block;
            points.push_back(is,*p);
            if( !has_block && !all ) { points.send( publisher.get(), bag.get(), topic, frame_id ); }
        }
        if( publishing && options.exists( "--hang-on,--stay" ))
        {
            for(int i=0;i<3;i++)
            {
                ros::spinOnce();
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
