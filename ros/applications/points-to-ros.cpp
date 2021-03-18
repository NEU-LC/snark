// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2021 Mission Systems Pty Ltd

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

void bash_completion( unsigned int const ac, char const * const * av )
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
    std::cerr << "\n    --output-fields=[<fields>]: fields to output; default: all input fields";
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

struct record
{
    boost::posix_time::ptime t;
    comma::uint32 block;
    std::vector< char > data;
    record() : block( 0 ) {}
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

namespace snark { namespace ros {

class point_cloud
{
    struct field_desc
    {
        sensor_msgs::PointField point_field;
        std::size_t input_offset;
        std::size_t size;
        field_desc( sensor_msgs::PointField point_field, std::size_t input_offset, std::size_t size )
            : point_field( point_field )
            , input_offset( input_offset )
            , size( size )
        {}
    };

public:
    point_cloud( const std::string& fields_str
               , const std::string& format_str
               , const std::string& output_fields_str
               , const std::string& frame_id )
        : frame_id( frame_id )
    {
        comma::csv::format format( format_str );
        std::vector< std::string > fields = comma::split( fields_str, ',' );
        std::vector< std::string > output_fields = comma::split( output_fields_str, ',' );
        const auto& elements = format.elements();
        if( fields.size() != elements.size() ) { COMMA_THROW( comma::exception, "size of fields and binary mismatch: " << fields.size() << " vs " << elements.size() ); }
        field_descs.reserve( fields.size() );
        std::size_t output_offset = 0;
        for( auto& output_field : output_fields )
        {
            auto it = std::find( fields.begin(), fields.end(), output_field );
            if( it != fields.end() )
            {
                auto i = std::distance( fields.begin(), it );
                sensor_msgs::PointField point_field;
                point_field.name = fields[i];
                point_field.offset = output_offset;
                point_field.datatype = map_data_type( elements[i].type );
                point_field.count = elements[i].count;
                std::size_t total_size = sizeof_datatype( point_field.datatype ) * point_field.count;
                field_descs.push_back( field_desc( point_field, elements[i].offset, total_size ));
                output_offset += total_size;
                comma::verbose << "added " << point_field.name
                               << "(" << comma::csv::format::to_format( elements[i].type )
                               << ") to point fields" << std::endl;
            }
        }
        output_data_size = output_offset;
        comma::verbose << "output_data_size: " << output_data_size << std::endl;
    }

    sensor_msgs::PointCloud2 create_msg( const std::vector< record >& records )
    {
        unsigned int count = records.size();
        sensor_msgs::PointCloud2 msg;

        std::vector< sensor_msgs::PointField > point_fields;
        for( const auto& field_desc : field_descs ) { point_fields.push_back( field_desc.point_field ); }

        msg.header.stamp = ::ros::Time::fromBoost( records[0].t );
        msg.header.seq = records[0].block;
        msg.header.frame_id = frame_id;
        msg.height = 1;
        msg.width = count;
        msg.point_step = output_data_size;
        msg.row_step = output_data_size * count;
        msg.fields = point_fields;
        msg.data.resize( output_data_size * count );

        std::size_t msg_data_offset = 0;
        for( const auto& record : records )
        {
            size_t field_offset = 0;
            for( const auto& field_desc : field_descs )
            {
                std::memcpy( &msg.data[msg_data_offset] + field_offset
                           , &record.data[0] + field_desc.input_offset
                           , field_desc.size );
                field_offset += field_desc.size;
            }
            msg_data_offset += output_data_size;
        }
        return msg;
    }

private:
    static unsigned int map_data_type( comma::csv::format::types_enum t )
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
                comma::verbose << "warning: ROS PointCloud2 doesn't support data type '" << comma::csv::format::to_format(t) << "', using FLOAT64 instead" << std::endl;
                return sensor_msgs::PointField::FLOAT64;
            default:
                { COMMA_THROW( comma::exception, "data type not supported: " << comma::csv::format::to_format(t) ); }
        }
    }

    static std::size_t sizeof_datatype( std::size_t datatype )
    {
        switch( datatype )
        {
            case sensor_msgs::PointField::INT8:
            case sensor_msgs::PointField::UINT8:
                return 1;
            case sensor_msgs::PointField::INT16:
            case sensor_msgs::PointField::UINT16:
                return 2;
            case sensor_msgs::PointField::INT32:
            case sensor_msgs::PointField::UINT32:
            case sensor_msgs::PointField::FLOAT32:
                return 4;
            case sensor_msgs::PointField::FLOAT64:
                return 8;
            default:
                { COMMA_THROW( comma::exception, "unknown data type: " << datatype ); }
        }
    }

    std::vector< field_desc > field_descs;
    std::size_t output_data_size;
    std::string frame_id;
};

} } // namespace snark { namespace ros {
    
class points
{
public:
    points( const comma::csv::options& csv
          , const comma::csv::format& format
          , const std::string& output_fields
          , const std::string& frame_id )
        : format( format )
        , point_cloud( csv.fields, format.expanded_string(), output_fields, frame_id )
        , data_size( format.size() )
        , ascii( !csv.binary() )
    {}

    void add_record( const record& r, const comma::csv::input_stream<record>& is )
    {
        records.push_back( r );
        records.back().data.resize( data_size );
        if( ascii )
        {
            std::string buf = format.csv_to_bin( is.ascii().last() );
            if( buf.size() != data_size ) { COMMA_THROW( comma::exception, "csv_to_bin size mismatch " << buf.size() << "; " << data_size ); } 
            std::memcpy( &records.back().data[0], buf.data(), data_size );
        }
        else
        {
            std::memcpy( &records.back().data[0], is.binary().last(), data_size );
        }
    }

    void send( const std::function< void( sensor_msgs::PointCloud2 ) >& publisher_fn )
    {
        //create msg
        sensor_msgs::PointCloud2 msg = point_cloud.create_msg( records );
        publisher_fn( msg );
        records.clear();
    }

    bool empty() { return records.empty(); }

private:
    std::vector< record > records;
    comma::csv::format format;
    snark::ros::point_cloud point_cloud;
    std::size_t data_size;
    bool ascii;
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
        unsigned int queue_size = options.value< unsigned int >( "--queue-size", 1 );
        if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
        comma::csv::format format = ( csv.binary()
                                    ? csv.format()
                                    : comma::csv::format( options.value< std::string >( "--format" )));
        std::string output_fields = options.value< std::string >( "--output-fields", csv.fields );
        comma::verbose << "outputting " << output_fields << std::endl;
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
        std::unique_ptr< std::function< void( sensor_msgs::PointCloud2 ) > > publish_fn;

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
            publish_fn.reset( new std::function< void( sensor_msgs::PointCloud2 ) >(
                                  [&]( sensor_msgs::PointCloud2 msg )
                                  {
                                      publisher->publish( msg );
                                      ros::spinOnce();
                                  } ));
        }
        else
        {
            bag.reset( new rosbag::Bag );
            bag->open( output_option, rosbag::bagmode::Write );
            publish_fn.reset( new std::function< void( sensor_msgs::PointCloud2 ) >(
                                  [&]( sensor_msgs::PointCloud2 msg )
                                  {
                                      bag->write( topic, msg.header.stamp, msg );
                                  } ));

        }

        comma::csv::input_stream< record > is( std::cin, csv );
        comma::csv::passed< record > passed( is, std::cout, csv.flush );
        unsigned int block = 0;
        points points( csv, format, output_fields, frame_id );

        while( std::cin.good() )
        {
            //read binary from input
            const record* p = is.read();
            if (( !p || block != p->block ) && !points.empty() )
            {
                points.send( *publish_fn.get() );
            }
            if( !p ) { break; }
            if( pass_through ) { passed.write(); }
            block = p->block;
            points.add_record( *p, is );
            if( !has_block && !all ) { points.send( *publish_fn.get() ); }
        }

        if( publishing && options.exists( "--hang-on,--stay" ))
        {
            for( int i = 0; i < 3; i++ )
            {
                ros::spinOnce();
                std::this_thread::sleep_for( std::chrono::milliseconds(1000) );
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
