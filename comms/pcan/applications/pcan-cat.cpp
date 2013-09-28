#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/string/string.h>
#include <snark/comms/pcan/message.h>
#include <snark/comms/pcan/traits.h>
#include <snark/comms/pcan/transport.h>

using namespace snark;

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "read pcan packets as csv or raw on stdin, send them to a given pcan device" << std::endl;
    std::cerr << "read pcan packets from pcan device, output them as csv or raw to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat packets.csv | pcan-cat <address> <options> > responses.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "address: pcan device address, e.g: todo" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --raw: stdin/stdout input/output are raw pcan packets; default: csv (see fields below)" << std::endl;
    std::cerr << "    todo: more options" << std::endl;
    std::cerr << std::endl;
    std::cerr << "fields: todo" << std::endl;
    if( verbose ) { std::cerr << std::endl << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    exit( 1 );
}

static int run_raw( pcan::transport& transport, const comma::command_line_options& options )
{
    std::cerr << "pcan-cat: raw mode: todo" << std::endl; return -1;
}

static int run( pcan::transport& transport, const comma::command_line_options& options )
{
    comma::io::select select;
    select.read().add( 0 );
    select.read().add( transport.fd() );
    comma::csv::options csv( options );
    comma::csv::input_stream< pcan::message > istream( std::cin, csv );
    comma::csv::output_stream< pcan::message > ostream( std::cout, csv );
    // todo: implement packet buffering, just like in segway-base or mars rover?
    while( std::cin.good() && !std::cin.eof() )
    {
        if( !istream.ready() ) { select.wait(); } // does it need to be a timed wait? 
        if( select.read().ready( transport.fd() ) )
        {
            boost::optional< pcan::message > message = transport.read();
            if( !message ) { return 1; }
            ostream.write( *message );
        }
        if( select.read().ready( 0 ) )
        {
            const pcan::message* message = istream.read();
            if( !message ) { return 1; }
            transport.write( *message );
        }
    }
    return 0;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--help,-h" ) ) { usage( verbose ); }
        const std::vector< std::string > unnamed = options.unnamed( "--verbose,-v,--raw", "-.*" );
        if( unnamed.size() > 1 ) { std::cerr << "pcan-cat: expected pcan device name, got" << comma::join( unnamed, ' ' ) << std::endl; return 1; }
        boost::scoped_ptr< pcan::transport > transport( pcan::make_transport( unnamed[0] ) );
        return options.exists( "--raw" ) ? run_raw( *transport, options ) : run( *transport, options );
    }
    catch( std::exception& ex )
    {
        std::cerr << "pcan-cat: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "pcan-cat: unknown exception" << std::endl;
    }
    return 1;
}
