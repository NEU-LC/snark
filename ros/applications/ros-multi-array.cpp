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
#include <fstream>
#include <comma/io/stream.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include "./std_msgs/traits.h"

//#include <ros/ros.h>
//#include <std_msgs/Float32MultiArray.h>
//#include <std_msgs/Float64MultiArray.h>
//#include <std_msgs/Int8MultiArray.h>
//#include <std_msgs/Int16MultiArray.h>
//#include <std_msgs/Int32MultiArray.h>
//#include <std_msgs/Int64MultiArray.h>
//#include <std_msgs/UInt8MultiArray.h>
//#include <std_msgs/UInt16MultiArray.h>
//#include <std_msgs/UInt32MultiArray.h>
//#include <std_msgs/UInt64MultiArray.h>



namespace {

void bash_completion( unsigned const ac, char const * const * av )
{
    static char const * const arguments[] = {
        " --binary -b --delimiter -d --fields -f --flush --format --help -h --precision --verbose -v",
        " --dimension --dim --from node-name --to"
    };
    std::cout << arguments << std::endl;
    exit( 0 );
}

void usage( bool const verbose )
{
    static const char* const indent="    ";

    std::cerr << std::endl;
    std::cerr << "Convert mult dimensional array from ros to csv and vice versa. Binary i/o only." << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << indent << comma::verbose.app_name() << " <operation> [<options>...]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "    --help,-h; print help and exit." << std::endl;
    std::cerr << "    --node-name=<name>; node name for this process, when not specified uses ros::init_options::AnonymousName flag." << std::endl;
    std::cerr << "    --queue-size=[<n>]; default=1; ROS Subscriber queue size." << std::endl;
    std::cerr << "    --type=<data_type>; csv format types. One of b,ub,w,uw,i,ui,l,ul,f or d" << std::endl;
    std::cerr << "    --from=<topic>; ros topic, mutually exclusive with --to." << std::endl;
    std::cerr << "        --flush; default=,. flush stream after each stride." << std::endl;
    std::cerr << "        --output-format; print output format and exit." << std::endl;
    std::cerr << "    --to=<topic>; ros topic, mutually exclusive with --from." << std::endl;
    std::cerr << "        --dimension,--dim=<attr>; dimension attributes label,size,stride. For example --dim=rows,3,9." << std::endl;
    std::cerr << "        --input-format; print format according to given dimension attributes and exit." << std::endl;
    std::cerr << "        --max-datagram-size: If a UDP transport is used, specifies the maximum datagram size (see ros::TransportHints)." << std::endl;
    std::cerr << "        --latch;  ROS publisher option; If true, the last message published on this topic will be saved and sent to new subscribers when they connect" << std::endl;
    std::cerr << std::endl;
}

void ros_init( char **av, boost::optional< std::string > node_name, std::string const& suffix )
{
    uint32_t node_options = 0;
    int rac = 1;
    if( !node_name )
    {
        node_name = "ros_multi_array" + suffix;
        node_options = ::ros::InitOption::AnonymousName;
    }
    ros::init( rac, av, *node_name, node_options );
}

template< typename T >
static std::string ros_layout_format( std_msgs::MultiArrayLayout const& layout )
{
    return std::to_string( layout.dim[0].stride ) + snark::ros::type_to_std_msgs< T >::as_string();
}

template< typename T >
class ros_subscriber
{
public:
    ros_subscriber( comma::command_line_options const& options )
        : flush( options.exists( "--flush" ) )
        , output_format( options.exists( "--output-format" ) )
        //, ofs_( "checkthis.csv", std::ofstream::out )
    {
        auto datagram = options.optional< int >( "--max-datagram-size" );

        ros::TransportHints hints;
        if( datagram ) { hints = ros::TransportHints().maxDatagramSize( *datagram ); }
        subscriber_ = node_.subscribe( options.value< std::string >( "--from" )
                , options.value< unsigned >( "--queue-size", 1U )
                , &ros_subscriber::process, this
                , hints );
    }

    void process( typename snark::ros::type_to_std_msgs< T >::array_type::ConstPtr const msg )
    {
        if( output_format ) { std::cout << ros_layout_format< T >( msg->layout ) << std::endl; ros::shutdown(); return; }
        //ofs_ << "Message: " << msg->data[ msg->layout.data_offset ] << ',' << msg->data[ msg->layout.data_offset ] << ',' << msg->data[ msg->layout.data_offset ] << std::endl;
        std::cout.write( reinterpret_cast< char const* >( msg->data.data() + msg->layout.data_offset ), sizeof( T ) * msg->layout.dim[ 0 ].stride );
        if( flush ) { std::cout.flush(); }
        if( !std::cout.good() ) { ros::shutdown(); }
    }

private:
    bool flush;
    bool output_format;
    ros::NodeHandle node_;
    ros::Subscriber subscriber_;
    //std::ofstream ofs_;
};


template< typename T >
class ros_publisher
{
public:
    static std_msgs::MultiArrayLayout get_layout( std::vector< std::string > const& dim_strings )
    {
        std_msgs::MultiArrayLayout layout;
        layout.data_offset = 0;

        comma::csv::ascii< std_msgs::MultiArrayDimension > dim_parse( "label,size,stride" );
        for( auto const& si : dim_strings )
        {
            layout.dim.push_back( dim_parse.get( si ) );
        }
        return layout;
    }

    ros_publisher( comma::command_line_options const& options )
    {
        message_.layout = get_layout(options.values< std::string >( "--dimensions,--dim" ) );
        message_.data.resize( message_.layout.dim[ 0 ].stride );
        publisher_ = node_.advertise< typename snark::ros::type_to_std_msgs< T >::array_type >( options.value< std::string >( "--to" )
                , options.value< unsigned >( "--queue-size", 1U )
                , options.exists( "--latch" ) );
        ros::spinOnce();
    }

    std::string input_format() { return ros_layout_format< T >( message_.layout ); }

    void publish()
    {
        while( std::cin.good() )
        {
            std::cin.read( reinterpret_cast< char* >( message_.data.data() ), sizeof( T ) * message_.layout.dim[ 0 ].stride );
            publisher_.publish( message_ );
        }
    }

private:
    ros::NodeHandle node_;
    ros::Publisher publisher_;
    typename snark::ros::type_to_std_msgs< T >::array_type message_;
};

template< typename T >
void ros_execute( char **av, comma::command_line_options const& options )
{
    if( options.exists( "--from" ) )
    {
        ros_init( av, options.optional< std::string >( "--node-name" ), "_subscriber" );
        ros_subscriber< T > subscriber( options );
        ros::spin();
    }
    else
    {
        if( options.exists( "--input-format" ) )
        {
            std::cout << ros_layout_format< T >( ros_publisher< T >::get_layout( options.values< std::string >( "--dimensions,--dim" ) ) ) << std::endl; return;
        }
        ros_init( av, options.optional< std::string >( "--node-name" ), "_publisher" );
        ros_publisher< T > publisher( options );
        publisher.publish();
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
        comma::csv::options csv( options );

        auto const type = options.value< std::string >( "--type" );
        if( 1 == type.size() )
        {
            switch( type[0] )
            {
                case 'b': ros_execute< char >( av, options ); return 0;
                case 'w': ros_execute< comma::int16 >( av, options ); return 0;
                case 'i': ros_execute< comma::int32 >( av, options ); return 0;
                case 'l': ros_execute< comma::int64 >( av, options ); return 0;
                case 'f': ros_execute< float >( av, options ); return 0;
                case 'd': ros_execute< double >( av, options ); return 0;
            }
        } else if( 2 == type.size() && 'u' == type[0] )
        {
            switch( type[0] )
            {
                case 'b': ros_execute< unsigned char >( av, options ); return 0;
                case 'w': ros_execute< comma::uint16 >( av, options ); return 0;
                case 'i': ros_execute< comma::uint32 >( av, options ); return 0;
                case 'l': ros_execute< comma::uint64 >( av, options ); return 0;
            }
        }
        std::cerr << comma::verbose.app_name() << ": unknown --type: " << type << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 0;
}

