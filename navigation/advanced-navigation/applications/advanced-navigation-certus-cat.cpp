// Copyright (c) 2021 Mission Systems Pty Ltd

#include "../device.h"
#include "../traits.h"
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>

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
    std::cerr << "\n    --sleep=<microseconds>: sleep between reading, default " << default_sleep;
    std::cerr << "\n";
    if( verbose )
    {
        std::cerr << "\nexamples:";
        std::cerr << "\n    " << comma::verbose.app_name() << " --device tcp:certus.local:16718";
        std::cerr << "\n    " << comma::verbose.app_name() << " --device /dev/usb/ttyUSB0 --baud 57600";
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
              << std::endl;
}

class app_base : protected device
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

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );

        if( options.exists( "--bash-completion" )) { bash_completion(); return 0; }

        flush = options.exists( "--flush" );

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
