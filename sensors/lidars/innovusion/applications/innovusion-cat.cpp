// Copyright (c) 2021 Mission Systems Pty Ltd

#include "../lidar.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <thread>

const std::string default_address( "172.168.1.10" );
const unsigned int default_port( 8001 );
const std::string default_log_dir( "/var/tmp" );
const std::string default_name( "innovusion_lidar" );

static void bash_completion( unsigned int const ac, char const* const* av )
{
    static const char* completion_options =
        " --help -h --verbose -v"
        " --output-fields --output-format"
        " --address --port --name"
        " --log-dir --sample-data"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nStream data from an Innovusion lidar";
    std::cerr << "\n";
    std::cerr << "\nUsage: " << comma::verbose.app_name() << " [<options>]";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:             show this help";
    std::cerr << "\n    --verbose,-v:          more output to stderr";
    std::cerr << "\n    --address=<ip>:        device address; default=" << default_address;
    std::cerr << "\n    --port=<num>:          device port; default=" << default_port;
    std::cerr << "\n    --log-dir=<dir>:       directory for system logs; default=" << default_log_dir;
    std::cerr << "\n    --name=<name>:         device name (max 32 chars); default=" << default_name;
    std::cerr << "\n    --output-fields:       print output fields and exit";
    std::cerr << "\n    --output-format:       print output format and exit";
    std::cerr << "\n    --sample-data=[<dir>]; TODO: read saved data from <dir>";
    std::cerr << "\n";
    std::cerr << "\nExamples:";
    std::cerr << "\n    " << comma::verbose.app_name() << " --address 192.168.10.40 | csv-from-bin $( " << comma::verbose.app_name() << " --output-format )";
    std::cerr << "\n";
    std::cerr << "\nUsing Innovusion LIDAR API version " << inno_api_version();
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

static std::unique_ptr< comma::csv::binary_output_stream< snark::innovusion::output_data_t > > os;
//static comma::uint32 block_id = 0;

void alarm_callback( int lidar_handle, void* context, enum inno_alarm error_level, enum inno_alarm_code alarm_code, const char* error_message )
{
}

int frame_callback( int lidar_handle, void* context, struct inno_frame* frame )
{
    comma::verbose << "frame " << frame->idx << "-" << frame->sub_idx
                   << ", timestamp=" << frame->ts_us_start << ", num points=" << frame->points_number
                   << ", packet size=" << sizeof( inno_point ) << " bytes" << std::endl;
    std::cout.write( (const char*)&frame->points[0], frame->points_number * sizeof( inno_point ));
    return 0;
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );

        std::vector< std::string > unnamed = options.unnamed( "--output-fields,--output-format,--verbose,-v", "-.*" );

        if( options.exists( "--output-fields" )) { std::cout << comma::join( comma::csv::names< snark::innovusion::output_data_t >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << comma::csv::format( comma::csv::format::value< snark::innovusion::output_data_t >()).string() << std::endl; return 0; }

        std::string address = options.value< std::string >( "--address", default_address );
        int port = options.value< unsigned int >( "--port", default_port );
        std::string log_dir = options.value< std::string >( "--log-dir", default_log_dir );
        std::string name = options.value< std::string >( "--name", default_name );

        inno_lidar_set_logs( 2, 2, nullptr );
        inno_lidar_set_log_level( comma::verbose ? INNO_LOG_INFO_LEVEL : INNO_LOG_WARNING_LEVEL );

        comma::signal_flag is_shutdown;
        // signal( SIGPIPE, SIG_IGN );
        inno_lidar_setup_sig_handler();

        snark::innovusion::lidar lidar;
        lidar.init( name, address, port, alarm_callback, frame_callback );
        lidar.start();

        comma::csv::options output_csv;
        output_csv.format( comma::csv::format::value< snark::innovusion::output_data_t >() );
        os.reset( new comma::csv::binary_output_stream< snark::innovusion::output_data_t >( std::cout, output_csv ));

        while( !is_shutdown && std::cout.good() )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ));
        }
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }

        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl;
    }
    return 1;
}
