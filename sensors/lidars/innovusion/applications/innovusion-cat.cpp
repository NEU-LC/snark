// Copyright (c) 2021 Mission Systems Pty Ltd

#include "../lidar.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <thread>

const std::string default_address( "172.168.1.10" );
const unsigned int default_port( 8001 );
const std::string default_name( "innovusion_lidar" );
const std::string default_output_type( "cooked" );

static void bash_completion( unsigned int const ac, char const* const* av )
{
    static const char* completion_options =
        " --help -h --verbose -v --debug"
        " --output-fields --output-format --output-type"
        " --address --port --name"
        " --sample-data"
        ;
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool verbose = false )
{
    std::cerr << "\nstream data from an Innovusion lidar";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " [<options>]";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:             show this help";
    std::cerr << "\n    --verbose,-v:          more output to stderr";
    std::cerr << "\n    --debug;               even more output";
    std::cerr << "\n    --address=<ip>:        device address; default=" << default_address;
    std::cerr << "\n    --port=<num>:          device port; default=" << default_port;
    std::cerr << "\n    --name=<name>:         device name (max 32 chars); default=" << default_name;
    std::cerr << "\n    --output=<type>:       one of none, raw, cooked, full; default=" << default_output_type;
    std::cerr << "\n    --output-fields:       print output fields for cooked or full data and exit";
    std::cerr << "\n    --output-format:       print output format for cooked or full data and exit";
    std::cerr << "\n    --sample-data=[<dir>]; TODO: read saved data from <dir>";
    std::cerr << "\n";
    std::cerr << "\noutput types:";
    std::cerr << "\n    none:   no output, useful for benchmarking the underlying SDK";
    std::cerr << "\n    raw:    raw inno_frame data from SDK";
    std::cerr << "\n    cooked: regular binary data, one packet per point";
    std::cerr << "\n    full:   as for cooked but with additional underlying data";
    std::cerr << "\n";
    std::cerr << "\nexample:";
    std::cerr << "\n    " << comma::verbose.app_name() << " --address 192.168.10.40 \\";
    std::cerr << "\n        | io-publish tcp:4444 \\";
    std::cerr << "\n              --size $( " << comma::verbose.app_name() << " --output-format | csv-format size ) \\";
    std::cerr << "\n              -m 1000 --no-flush";
    std::cerr << "\n";
    std::cerr << "\nusing Innovusion LIDAR API version " << inno_api_version();
    std::cerr << "\n";
    std::cerr << std::endl;
    exit( 0 );
}

enum class output_type_t { none, raw, cooked, full };

output_type_t output_type_from_string( const std::string& output_type_str )
{
    if( output_type_str == "none" )   { return output_type_t::none; }
    if( output_type_str == "raw" )    { return output_type_t::raw; }
    if( output_type_str == "cooked" ) { return output_type_t::cooked; }
    if( output_type_str == "full" ) { return output_type_t::full; }
    { COMMA_THROW( comma::exception, "unknown output type \"" << output_type_str << "\"" ); }
}

static output_type_t output_type = output_type_t::cooked;
static bool fatal_error = false;

namespace snark { namespace innovusion {
struct raw_output {};
struct null_output {};
}; };

template< typename T >
struct app
{
    static std::string output_fields() { return comma::join( comma::csv::names< T >( true ), ',' ); }
    static std::string output_format() { return comma::csv::format::value< T >(); }

    static int run( const comma::command_line_options& options )
    {
        if( options.exists( "--output-fields" )) { std::cout << output_fields() << std::endl; return 0; }
        if( options.exists( "--output-format" )) { std::cout << output_format() << std::endl; return 0; }

        std::string address = options.value< std::string >( "--address", default_address );
        int port = options.value< unsigned int >( "--port", default_port );
        std::string name = options.value< std::string >( "--name", default_name );

        inno_lidar_set_logs( STDERR_FILENO, STDERR_FILENO, nullptr );
        // There are two more log levels beyond INFO: TRACE and EVERYTHING,
        // but they log an insane amount so we'll set the debug level to be INFO
        // Even INFO we'll only activate for --debug, not for --verbose
        inno_lidar_set_log_level( options.exists( "--debug" ) ? INNO_LOG_INFO_LEVEL : INNO_LOG_WARNING_LEVEL );

        comma::signal_flag is_shutdown;
        inno_lidar_setup_sig_handler();

        snark::innovusion::lidar lidar;
        lidar.init( name, address, port, alarm_callback, frame_callback );
        lidar.start();

        while( !is_shutdown && !fatal_error && std::cout.good() )
        {
            std::this_thread::sleep_for( std::chrono::milliseconds( 500 ));
        }
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        if( fatal_error ) { std::cerr << comma::verbose.app_name() << ": fatal error, exiting" << std::endl; return 1; }
        return 0;
    }

    static void alarm_callback( int lidar_handle, void* context
                              , enum inno_alarm error_level, enum inno_alarm_code alarm_code, const char* error_message )
    {
        std::cerr << comma::verbose.app_name() << ": [" << snark::innovusion::alarm_type_to_string( error_level ) << "] "
                  << snark::innovusion::alarm_code_to_string( alarm_code )
                  << " \"" << error_message << "\"" << std::endl;
        if( error_level >= INNO_ALARM_CRITICAL ) { fatal_error = true; }
    }

    static int frame_callback( int lidar_handle, void* context, inno_frame* frame )
    {
        static inno_timestamp_us_t start_time = frame->ts_us_start;
        comma::verbose << "frame " << frame->idx << "-" << frame->sub_idx << "-" << frame->sub_seq
                       << ", timestamp(ms)=" << ( frame->ts_us_start - start_time ) / 1000 << ", num points=" << frame->points_number
                       << std::endl;
        // TODO: to improve performance, don't output in the callback but just flag a separate thread
        output( frame );
        return 0;        // caller frees memory
    }

    static comma::csv::binary_output_stream< T > output_stream()
    {
        comma::verbose << "making output_stream" << std::endl;
        comma::csv::options output_csv;
        output_csv.format( comma::csv::format::value< T >() );
        return comma::csv::binary_output_stream< T >( std::cout, output_csv );
    }

    static void output( inno_frame* frame )
    {
        static comma::csv::binary_output_stream< T > os = output_stream();

        for( unsigned int i = 0; i < frame->points_number; i++ )
        {
            os.write( T( frame, i ));
        }
    }
};

template<> std::string app< snark::innovusion::raw_output >::output_fields()
    { COMMA_THROW( comma::exception, "raw data does not have output fields" ); }
template<> std::string app< snark::innovusion::raw_output >::output_format()
    { COMMA_THROW( comma::exception, "raw data does not have output format" ); }

template<> void app< snark::innovusion::raw_output >::output( inno_frame* frame )
{
    std::cout.write( (const char*)&frame->points[0], frame->points_number * sizeof( inno_point ));
}

template<> std::string app< snark::innovusion::null_output >::output_fields()
    { COMMA_THROW( comma::exception, "null data does not have output fields" ); }
template<> std::string app< snark::innovusion::null_output >::output_format()
    { COMMA_THROW( comma::exception, "null data does not have output format" ); }
template<> void app< snark::innovusion::null_output >::output( inno_frame* frame ) {}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );

        output_type = output_type_from_string(  options.value< std::string >( "--output-type", default_output_type ));

        if( output_type == output_type_t::raw ) { return app< snark::innovusion::raw_output >::run( options ); }
        else if( output_type == output_type_t::cooked ) { return app< snark::innovusion::output_data_t >::run( options ); }
        else if( output_type == output_type_t::full ) { return app< snark::innovusion::output_data_full_t >::run( options ); }
        else if( output_type == output_type_t::none ) { return app< snark::innovusion::null_output >::run( options ); }
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
