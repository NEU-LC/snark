// Copyright (c) 2019 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

#include "detail/file-util.h"
#include "../../imaging/cv_mat/serialization.h"
#include <comma/io/stream.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <boost/bimap.hpp>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/fill_image.h>
#include <vector>
#include <fstream>
#include <unordered_map>

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments =
        " --help -h --node-name --queue-size"
        " --from --bags --flush"
        " --to --max-datagram-size --latch --frame"
        ;
    std::cout << arguments << std::endl;
    exit( 0 );
}

void usage( bool )
{
    std::cerr << "\nconvert image from ros to cv image and vice-versa";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " --from=<topic> [<options>]";
    std::cerr << "\n       " << comma::verbose.app_name() << " --to=<topic> [<options>]";
    std::cerr << "\n";
    std::cerr << "\ngeneral options:";
    std::cerr << "\n    --help,-h:                 print help and exit";
    std::cerr << "\n    --node,--node-name=<name>: default=ros_image_<publisher|subscriber>";
    std::cerr << "\n    --queue-size=[<n>]:        default=1; ROS queue size";
    std::cerr << "\n";
    std::cerr << "\n    when --node is not specified also sets ros::init_options::AnonymousName flag";
    std::cerr << "\n";
    std::cerr << "\nfrom options:";
    std::cerr << "\n    --from=<topic>:   topic to read";
    std::cerr << "\n    --bags=[<files>]: comma-separated list of bag files, wildcards accepted";
    std::cerr << "\n    --flush:          flush stream after each image";
    std::cerr << "\n";
    std::cerr << "\nto options:";
    std::cerr << "\n    --to=<topic>:                 topic to publish to";
    std::cerr << "\n    --frame=[<frame_id>]:         ros header frame id";
    std::cerr << "\n    --latch:                      latch last message for new subscribers";
    std::cerr << "\n    --max-datagram-size=[<size>]: maximum datagram size for UDP";
    std::cerr << "\n";
    std::cerr << "\nexamples:";
    std::cerr << "\n    read from and write to live topics:";
    std::cerr << "\n    $ ros-image --from camera/image_raw | cv-cat \"view;null\"";
    std::cerr << "\n    $ cat image.bin | ros-image --to camera/image_raw";
    std::cerr << "\n";
    std::cerr << "\n    read from bag files:";
    std::cerr << "\n    $ ros-image --from camera/image_raw --bags a.bag,b.bag | cv-cat \"view;null\"";
    std::cerr << "\n";
    std::cerr << "\n    wildcard expansion need to be protected from expanding by bash";
    std::cerr << "\n    $ ros-image --from camera/image_raw --bags \"*.bag\" | cv-cat \"view;null\"";
    std::cerr << "\n" << std::endl;
}

void ros_init( char **av, boost::optional< std::string > node_name, std::string const& suffix )
{
    uint32_t node_options = 0;
    int rac = 1;
    if( !node_name )
    {
        node_name = "ros_image" + suffix;
        node_options = ::ros::InitOption::AnonymousName;
    }
    ros::init( rac, av, *node_name, node_options );

    #ifndef WIN32
    // Undo ROS's ignoring of SIGPIPE (see roscpp/init.cpp:469)
    sigset_t sigmask;
    sigemptyset( &sigmask );
    sigaddset( &sigmask, SIGPIPE );
    sigprocmask( SIG_UNBLOCK, &sigmask, NULL );
    #endif
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
        : flush( options.exists( "--flush" ))
        , from_bag( options.exists( "--bags" ))
        , topic( options.value< std::string >( "--from" ))
    {
        if( from_bag )
        {
            for( auto name: comma::split( options.value< std::string >( "--bags", "" ), ',' ))
            {
                std::vector< std::string > expansion = snark::ros::glob( name );
                bag_names.insert( bag_names.end(), expansion.begin(), expansion.end() );
            }
        }
        else
        {
            auto datagram = options.optional< int >( "--max-datagram-size" );

            ros::TransportHints hints;
            if( datagram ) { hints = ros::TransportHints().maxDatagramSize( *datagram ); }
            node_.reset( new ros::NodeHandle() );
            subscriber_ = node_->subscribe( topic
                                          , options.value< unsigned >( "--queue-size", 1U )
                                          , &ros_subscriber::process, this
                                          , hints );
        }
    }

    void write( message_type const msg )
    {
        auto time = msg->header.stamp.toBoost();
        cv_strm.write( std::cout
                     , std::make_pair( time
                                     , cv::Mat( cv::Size( msg->width, msg->height )
                                              , cv_format( msg->encoding )
                                              , ( void* )msg->data.data()
                                              , cv::Mat::AUTO_STEP )));
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
            for( auto bag_name: bag_names )
            {
                comma::verbose << "opening " << bag_name << std::endl;
                rosbag::Bag bag( bag_name );
                for( rosbag::MessageInstance const mi : rosbag::View( bag, rosbag::TopicQuery( topic )))
                {
                    if( is_shutdown ) { return; }
                    message_type const msg = mi.instantiate< sensor_msgs::Image >();
                    write( msg );
                }
            }
        }
        else
        {
            ros::spin();
        }
    }

private:
    bool const flush;
    bool const from_bag;
    std::unique_ptr< ros::NodeHandle > node_;
    ros::Subscriber subscriber_;
    std::vector< std::string > bag_names;
    std::string topic;
    comma::signal_flag is_shutdown;
};

class ros_publisher : public cv_io
{
public:
    ros_publisher( comma::command_line_options const& options )
    {
        message_.header.frame_id = options.value< std::string >( "--frame", std::string() );

        publisher_ = node_.advertise< sensor_msgs::Image >( options.value< std::string >( "--to" )
                                                          , options.value< unsigned >( "--queue-size", 1U )
                                                          , options.exists( "--latch" ));
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

void ros_execute( char** av, comma::command_line_options const& options )
{
    auto node_name = options.optional< std::string >( "--node-name,--node" );
    if( options.exists( "--from" ))
    {
        ros_init( av, node_name, "_subscriber" );
        ros_subscriber subscriber( options );
        subscriber.subscribe();
    }
    else
    {
        ros_init( av, node_name, "_publisher" );
        ros_publisher publisher( options );
        publisher.publish();
    }
}

int main( int ac, char* av[] )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--bash-completion" )) { bash_completion( ac, av ); }
        options.assert_mutually_exclusive( "--from,--flush,--output-fields", "--to,--dimensions,--dim,--input-fields" );
        options.assert_exists( "--from,--to" );
        ros_execute( av, options );
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
