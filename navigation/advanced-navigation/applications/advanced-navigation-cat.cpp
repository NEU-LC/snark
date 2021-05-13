// Copyright (c) 2021 Mission Systems Pty Ltd

#include "../device.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>

namespace messages = snark::navigation::advanced_navigation::messages;

const unsigned int default_baud_rate = 115200;
const unsigned int default_sleep = 10000;
bool flush;

void usage( bool verbose )
{
    std::cerr << "\nconnect to Advanced Navigation Certus device and send commands or output data";
    std::cerr << "\n";
    std::cerr << "\nusage: " << comma::verbose.app_name() << " <options>";
    std::cerr << "\n    or " << comma::verbose.app_name() << " --send <command> <options>";
    std::cerr << "\n";
    std::cerr << "\noptions:";
    std::cerr << "\n    --help,-h:              show help";
    std::cerr << "\n    --verbose,-v:           show detailed messages";
    std::cerr << "\n    --baud-rate,--baud=<n>: baud rate for serial connection, default " << default_baud_rate;
    std::cerr << "\n    --device=<device>;      serial or network port";
    std::cerr << "\n    --flush:                flush output stream after each record";
    std::cerr << "\n    --input-fields:         print input command fields for --send and exit";
    std::cerr << "\n    --input-format:         print input command format for --send and exit";
    std::cerr << "\n    --send=<command>        read data from stdin and send as command packet";
    std::cerr << "\n    --sleep=<microseconds>: sleep between reading, default " << default_sleep;
    std::cerr << "\n";
    std::cerr << "\n    --send: read commands from stdin and write command message to device";
    std::cerr << "\n            csv options apply to input stream";
    std::cerr << "\n    output: acknowledgement, use --verbose for human readable message";
    std::cerr << "\n";
    std::cerr << "\n    commands:";
    std::cerr << "\n        magnetic-calibration: send magnetic calibration command";
    std::cerr << "\n            input fields: action";
    std::cerr << "\n            where <action> is:";
    std::cerr << "\n                0 - Cancel magnetic calibration";
    std::cerr << "\n                2 - Start 2D magnetic calibration";
    std::cerr << "\n                3 - Start 3D magnetic calibration";
    std::cerr << "\n                4 - Reset calibration to defaults";
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\nexamples:";
        std::cerr << "\n    " << comma::verbose.app_name() << " --device tcp:certus.local:16718";
        std::cerr << "\n    " << comma::verbose.app_name() << " --device /dev/usb/ttyUSB0 --baud 57600";
        std::cerr << "\n";
        std::cerr << "\n  send 2D magnetic calibration command and see status";
        std::cerr << "\n    " << comma::verbose.app_name() << " --send magnetic-calibration --input-fields";
        std::cerr << "\n    echo 2 | " << comma::verbose.app_name() << " --send magnetic-calibration";
        std::cerr << "\n    " << comma::verbose.app_name() << " | advanced-navigation-to-csv magnetic-calibration";
        std::cerr << "\n    advanced-navigation-to-csv --magnetic-calibration-description";
    }
    else
    {
        std::cerr << "\nuse -v or --verbose for examples";
    }
    std::cerr << "\n" << std::endl;
}

static void bash_completion()
{
    std::cout << " --help -h --verbose -v"
              << " --baud-rate --baud --device --flush --sleep"
              << " --send --input-fields --input-format magnetic-calibration"
              << std::endl;
}

class app_base : protected snark::navigation::advanced_navigation::device
{
public:
    app_base( const comma::command_line_options& options )
        : device( options.value< std::string >( "--device" )
                , options.value< unsigned int >( "--baud-rate,--baud", default_baud_rate ))
        , us( options.value< unsigned int >( "--sleep", default_sleep ))
    {
        select.read().add( fd() );
    }

    void process()
    {
        while( !signal && std::cout.good() )
        {
            select.wait( boost::posix_time::microseconds( us ));
            if( !signal && select.read().ready( fd() )) { device::process(); }
        }
    }

private:
    unsigned int us;
    comma::io::select select;
    comma::signal_flag signal;
};

class app_raw : public app_base
{
public:
    app_raw( const comma::command_line_options& options )
        : app_base( options )
        , obuf( 260 )
    {}

protected:
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length )
    {
        obuf.resize( messages::header::size + msg_data_length );
        std::memcpy( &obuf[0], msg_header->data(), messages::header::size );
        std::memcpy( &obuf[messages::header::size], msg_data, msg_data_length );
        std::cout.write( &obuf[0], obuf.size() );
        if( flush ) { std::cout.flush(); }
    }

private:
    std::vector< char > obuf;
};

struct send_factory_i
{
    virtual ~send_factory_i() {}
    virtual void input_fields() = 0;
    virtual void input_format() = 0;
    virtual unsigned int run( const comma::command_line_options& options ) = 0;
};

template< typename T >
struct send_factory_t : public send_factory_i
{
    typedef T type;
    void input_fields() { T::input_fields(); }
    void input_format() { T::input_format(); }
    unsigned int run( const comma::command_line_options& options )
    {
        T app( options );
        return app.process();
    }
};

template < typename T >
struct send_app : protected snark::navigation::advanced_navigation::device
{
    comma::csv::input_stream< T > is;
    unsigned int result;
    send_app( const comma::command_line_options& options )
        : device( options.value< std::string >( "--device" ), options.value< unsigned int >( "--baud-rate,--baud", default_baud_rate ))
        , is( std::cin, comma::csv::options( options ))
    {}

    virtual void handle( const messages::acknowledgement* msg )
    {
        result = msg->result();
        std::cout << result << std::endl;
        comma::verbose << messages::acknowledgement::result_msg( result ) << std::endl;
    }

    unsigned int process()
    {
        while( std::cin.good() )
        {
            const T* pt = is.read();
            if( !pt )
                break;
            messages::command cmd = pt->get_command();
            send( cmd );
        }
        return result;
    }

    static void input_fields()
    {
        std::cout << comma::join( comma::csv::names< T >( true ), ',' ) << std::endl;
    }

    static void input_format()    //const std::string& fields
    {
        //std::cout << comma::csv::format::value< output_t >( fields, true ) << std::endl;
        std::cout << comma::csv::format::value< T >() << std::endl;
    }
};

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );

        if( options.exists( "--bash-completion" )) { bash_completion(); return 0; }

        flush = options.exists( "--flush" );

        auto opt_send = options.optional< std::string >( "--send" );
        if( opt_send )
        {
            std::unique_ptr< send_factory_i > sf;
            if( *opt_send == "magnetic-calibration" ) { sf.reset( new send_factory_t< send_app< messages::magnetic_calibration_configuration >>() ); }
            else { COMMA_THROW( comma::exception, "invalid send command: " << *opt_send ); }

            if( options.exists( "--input-fields" )) { sf->input_fields(); return 0; }
            if( options.exists( "--input-format" )) { sf->input_format(); return 0; }

            return sf->run( options );
        }

        app_raw( options ).process();

        return 0;
    }
    catch( snark::navigation::advanced_navigation::eois_exception& e )
    {
        // normal exit on end of input stream
        comma::verbose << comma::verbose.app_name() << ": " << e.what() << std::endl;
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl; }
    return 1;
}
