// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2019 The University of Sydney
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
#include <fstream>
#include <unordered_map>
#include <boost/bimap.hpp>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include "../../imaging/cv_mat/serialization.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace {

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments[] = {
        " --help -h --node-name --queue-size",
        " --from --flush",
        " --to --max-datagram-size --latch --frame-id"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

void usage( bool const verbose )
{
    static const char* const indent="    ";

    std::cerr << std::endl;
    std::cerr << "Convert image from ros to cv image and vice versa. Binary i/o only." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " <operation> [<options>...]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "    --help,-h; print help and exit." << std::endl;
    std::cerr << "    --node-name=<name>; node name for this process, when not specified uses ros::init_options::AnonymousName flag." << std::endl;
    std::cerr << "    --queue-size=[<n>]; default=1; ROS Subscriber queue size." << std::endl;
    std::cerr << "    --from=<topic>; ros topic, mutually exclusive with --to." << std::endl;
    std::cerr << "        --bags=[<bagfile>]; read from ros bag file." << std::endl;
    std::cerr << "        --flush; flush stream after each stride." << std::endl;
    std::cerr << "    --to=<topic>; ros topic, mutually exclusive with --from." << std::endl;
    std::cerr << "        --frame-id=[<frame_id>]: ros header frame id." << std::endl;
    std::cerr << "        --max-datagram-size=[<size>]: If a UDP transport is used, specifies the maximum datagram size (see ros::TransportHints)." << std::endl;
    std::cerr << "        --latch;  ROS publisher option; If true, the last message published on this topic will be saved and sent to new subscribers when they connect" << std::endl;
    std::cerr << std::endl;
}

void ros_init( char **av, boost::optional< std::string > node_name, std::string const& suffix )
{
    uint32_t node_options = 0;
    int rac = 1;
    if( !node_name )
    {
        node_name = "ros_cv_image" + suffix;
        node_options = ::ros::InitOption::AnonymousName;
    }
    ros::init( rac, av, *node_name, node_options );
}

class cv_io
{
protected:
    static unsigned cv_format( std::string const& ros_encoding );
    static std::string ros_format( unsigned const cv_encoding );

    snark::cv_mat::serialization::options cv_opt;
    snark::cv_mat::serialization cv_strm;

    cv_io() : cv_strm( cv_opt ) {}
};

unsigned cv_io::cv_format( std::string const& ros_encoding )
{
    static const std::unordered_map< std::string, unsigned > map = {
        { "rgb8", CV_8UC3 },
        { "rgba8", CV_8UC4 },
        { "rgb16", CV_16UC3 },
        { "rgba16", CV_16UC4 },
        { "bgr8", CV_8UC3 },
        { "bgra8", CV_8UC4 },
        { "bgr16", CV_16UC3 },
        { "bgra16", CV_16UC4 },
        { "mono8", CV_8UC1 },
        { "mono16", CV_16UC1 },
        { "8UC1", CV_8UC1 },
        { "8UC2", CV_8UC2 },
        { "8UC3", CV_8UC3 },
        { "8UC4", CV_8UC4 },
        { "8SC1", CV_8SC1 },
        { "8SC2", CV_8SC2 },
        { "8SC3", CV_8SC3 },
        { "8SC4", CV_8SC4 },
        { "16UC1", CV_16UC1 },
        { "16UC2", CV_16UC2 },
        { "16UC3", CV_16UC3 },
        { "16UC4", CV_16UC4 },
        { "16SC1", CV_16SC1 },
        { "16SC2", CV_16SC2 },
        { "16SC3", CV_16SC3 },
        { "16SC4", CV_16SC4 },
        { "32SC1", CV_32SC1 },
        { "32SC2", CV_32SC2 },
        { "32SC3", CV_32SC3 },
        { "32SC4", CV_32SC4 },
        { "32FC1", CV_32FC1 },
        { "32FC2", CV_32FC2 },
        { "32FC3", CV_32FC3 },
        { "32FC4", CV_32FC4 },
        { "64FC1", CV_64FC1 },
        { "64FC2", CV_64FC2 },
        { "64FC3", CV_64FC3 },
        { "64FC4", CV_64FC4 },
        { "bayer_rggb8", CV_8UC4 },
        { "bayer_bggr8", CV_8UC4 },
        { "bayer_gbrg8", CV_8UC4 },
        { "bayer_grbg8", CV_8UC4 },
        { "bayer_rggb16", CV_16UC4 },
        { "bayer_bggr16", CV_16UC4 },
        { "bayer_gbrg16", CV_16UC4 },
        { "bayer_grbg16", CV_16UC4 },
    };
    return map.at( ros_encoding );
}

std::string cv_io::ros_format( unsigned const cv_encoding )
{
    static const std::unordered_map< unsigned, std::string > map = {
        { CV_8UC3, "rgb8" },
        { CV_8UC4, "rgba8" },
        { CV_16UC3, "rgb16" },
        { CV_16UC4, "rgba16" },
        { CV_8UC3, "bgr8" },
        { CV_8UC4, "bgra8" },
        { CV_16UC3, "bgr16" },
        { CV_16UC4, "bgra16" },
        { CV_8UC1, "mono8" },
        { CV_16UC1, "mono16" },
        { CV_8UC1, "8UC1" },
        { CV_8UC2, "8UC2" },
        { CV_8UC3, "8UC3" },
        { CV_8UC4, "8UC4" },
        { CV_8SC1, "8SC1" },
        { CV_8SC2, "8SC2" },
        { CV_8SC3, "8SC3" },
        { CV_8SC4, "8SC4" },
        { CV_16UC1, "16UC1" },
        { CV_16UC2, "16UC2" },
        { CV_16UC3, "16UC3" },
        { CV_16UC4, "16UC4" },
        { CV_16SC1, "16SC1" },
        { CV_16SC2, "16SC2" },
        { CV_16SC3, "16SC3" },
        { CV_16SC4, "16SC4" },
        { CV_32SC1, "32SC1" },
        { CV_32SC2, "32SC2" },
        { CV_32SC3, "32SC3" },
        { CV_32SC4, "32SC4" },
        { CV_32FC1, "32FC1" },
        { CV_32FC2, "32FC2" },
        { CV_32FC3, "32FC3" },
        { CV_32FC4, "32FC4" },
        { CV_64FC1, "64FC1" },
        { CV_64FC2, "64FC2" },
        { CV_64FC3, "64FC3" },
        { CV_64FC4, "64FC4" },
        { CV_8UC4, "bayer_rggb8" },
        { CV_8UC4, "bayer_bggr8" },
        { CV_8UC4, "bayer_gbrg8" },
        { CV_8UC4, "bayer_grbg8" },
        { CV_16UC4, "bayer_rggb16" },
        { CV_16UC4, "bayer_bggr16" },
        { CV_16UC4, "bayer_gbrg16" },
        { CV_16UC4, "bayer_grbg16" },
    };
    return map.at( cv_encoding );
}

class ros_subscriber : public cv_io
{
public:
    using message_type = typename sensor_msgs::Image::ConstPtr;

    ros_subscriber( comma::command_line_options const& options )
        : flush( options.exists( "--flush" ) )
        , output_format( options.exists( "--output-format" ) )
        , from_bag( options.exists( "--bags" ) )
    {
        if( from_bag )
        {
            bag_.open( options.value< std::string >( "--bags" ) );
            view_.addQuery( bag_, rosbag::TopicQuery( options.value< std::string >( "--from" ) ) );
        }
        else
        {
            auto datagram = options.optional< int >( "--max-datagram-size" );

            ros::TransportHints hints;
            if( datagram ) { hints = ros::TransportHints().maxDatagramSize( *datagram ); }
            node_.reset( new ros::NodeHandle() );
            subscriber_ = node_->subscribe( options.value< std::string >( "--from" )
                    , options.value< unsigned >( "--queue-size", 1U )
                    , &ros_subscriber::process, this
                    , hints );
        }
    }

    void write( message_type const msg )
    {
        auto time = msg->header.stamp.toBoost();
        cv_strm.write( std::cout, std::make_pair( time, cv::Mat( cv::Size( msg->width, msg->height ), cv_format( msg->encoding ), ( void* )msg->data.data(), cv::Mat::AUTO_STEP ) ) );
        if( flush ) { std::cout.flush(); }
    }

    void process( message_type const msg )
    {
        write( msg );
        if( !std::cout.good() ) { ros::shutdown(); }
    }

    void subscribe( void )
    {
        if( from_bag )
        {
            for( rosbag::MessageInstance const mi : view_ )
            {
                message_type const msg = mi.instantiate< sensor_msgs::Image >();
                write( msg );
            }
        }
        else
        {
            ros::spin();
        }
    }

private:
    bool const flush;
    bool const output_format;
    bool const from_bag;
    std::unique_ptr< ros::NodeHandle > node_;
    ros::Subscriber subscriber_;
    rosbag::Bag bag_;
    rosbag::View view_;
};


class ros_publisher : public cv_io
{
public:
    ros_publisher( comma::command_line_options const& options )
    {
        message_.header.frame_id = options.value< std::string >( "--frame-id", std::string() );

        publisher_ = node_.advertise< sensor_msgs::Image >( options.value< std::string >( "--to" )
                , options.value< unsigned >( "--queue-size", 1U )
                , options.exists( "--latch" ) );
        ros::spinOnce();
    }

    void publish()
    {
        while( std::cin.good() )
        {
            auto record = cv_strm.read< boost::posix_time::ptime >( std::cin );
            message_.header.seq++;
            message_.header.stamp = ros::Time::fromBoost( record.first );
            sensor_msgs::fillImage( message_
                                  , ros_format( record.second.type() )
                                  , record.second.rows
                                  , record.second.cols
                                  , record.second.step
                                  , record.second.data );
            publisher_.publish( message_ );
        }
    }

private:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    typename sensor_msgs::Image message_;
};

void ros_execute( char **av, comma::command_line_options const& options )
{
    if( options.exists( "--from" ) )
    {
        ros_init( av, options.optional< std::string >( "--node-name" ), "_subscriber" );
        ros_subscriber subscriber( options );
        subscriber.subscribe();
    }
    else if( options.exists( "--to" ) )
    {
        ros_init( av, options.optional< std::string >( "--node-name" ), "_publisher" );
        ros_publisher publisher( options );
        publisher.publish();
    }
    else
    {
        COMMA_THROW( comma::exception, "Exactly one topic (either --from or --to) must be given." );
    }
}

}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( ac, av );
        options.assert_mutually_exclusive( "--from,--flush,--output-fields", "--to,--dimensions,--dim,--input-fields" );

        ros_execute( av, options );
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}

