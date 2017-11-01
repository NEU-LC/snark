// This file is part of snark, a generic and flexible library for robotics research
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

#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include "../device.h"
#include "../orientus/traits.h"

using namespace snark::navigation::advanced_navigation;

const unsigned default_baud_rate = 115200;
const unsigned default_sleep = 10000;

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "connect to Advanced Navigation Orientus device and output GPS data" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: " << comma::verbose.app_name() << " <what> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what: " << std::endl;
    std::cerr << "    system-state: full system state packet" << std::endl;
    std::cerr << "    raw-sensors" << std::endl;
    std::cerr << "    header: output packet headers only" << std::endl;
    std::cerr << "    all: combines system-state and raw-sensors" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --baud-rate=<n>       baud rate for connection, default " << default_baud_rate << std::endl;
    std::cerr << "    --binary-output       output in binary" << std::endl;
    std::cerr << "    --description=<field> print out one line description text for input values of <field>; csv options apply to input" << std::endl;
    std::cerr << "                          <field>: system_status | filter_status" << std::endl;
    std::cerr << "    --device=<filename>   filename for serial port e.g. /dev/usb/ttyUSB0" << std::endl;
    std::cerr << "    --help,-h             show help" << std::endl;
    std::cerr << "    --output-fields       print output fields and exit" << std::endl;
    std::cerr << "    --output-format       print output format and exit" << std::endl;
    std::cerr << "    --raw                 output raw packets to stdout, " << std::endl;
    std::cerr << "    --sleep=<n>           microsecond sleep between reading, default " << default_sleep << std::endl;
    std::cerr << "    --stdin               read packets from stdin, can't be used with options that need to write to device" << std::endl;
    std::cerr << "    --verbose,-v          verbose output" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    sudo mknod /dev/usb/ttyUSB0 c 188 0" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " system-state --device /dev/usb/ttyUSB0" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " raw-sensors --device /dev/usb/ttyUSB0" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " all --device /dev/usb/ttyUSB0" << std::endl;
    std::cerr << "    echo 128 | " << comma::verbose.app_name() << " --description system_status" << std::endl;
    std::cerr << "    echo 1029 | " << comma::verbose.app_name() << " --description filter_status" << std::endl;
    std::cerr << std::endl;
}

struct output_all
{
    messages::system_state system_state;
    messages::raw_sensors raw_sensors;
};

struct status_data
{
    uint16_t status;
};

namespace comma { namespace visiting {

template <>
struct traits< output_all >
{
    template < typename Key, class Visitor > static void visit( const Key&, const output_all& p, Visitor& v )
    {
        v.apply( "", p.system_state );
        v.apply( "", p.raw_sensors );
    }
};

template <>
struct traits< status_data >
{
    template < typename Key, class Visitor > static void visit( const Key&, const status_data& p, Visitor& v )
    {
        v.apply( "status", p.status );
    }
    template < typename Key, class Visitor > static void visit( const Key&, status_data& p, Visitor& v )
    {
        v.apply( "status", p.status );
    }
};

} } // namespace comma { namespace visiting {

struct app_i
{
    virtual ~app_i() {}
    virtual void run() = 0;
    virtual void output_fields() = 0;
};

struct app_base : protected device
{
    unsigned us;
    comma::io::select select;
public:
    app_base( const std::string& port, const comma::command_line_options& options )
        : device( port, options.value< unsigned >( "--baud-rate", default_baud_rate ) )
        , us( options.value< unsigned >( "--sleep", default_sleep ) )
    {
        select.read().add( fd() );
    }
    void process()
    {
        while( std::cout.good() )
        {
            select.wait( boost::posix_time::microseconds( us ) );
            if( select.read().ready( fd() ) ) { device::process(); }
        }
    }
};

struct app_raw : public app_base
{
    std::vector<char> obuf;
    app_raw( const std::string& port, const comma::command_line_options& options ) : app_base( port, options ), obuf( 260 ) {}
    static void output_fields() { std::cout << std::endl; }
    static void output_format() { std::cout << std::endl;  }
protected:
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length )
    {
        obuf.resize( messages::header::size + msg_data_length );
        std::memcpy( &obuf[0], msg_header->data(), messages::header::size );
        std::memcpy( &obuf[ messages::header::size ], msg_data, msg_data_length );
        std::cout.write( &obuf[0], obuf.size() );
    }
};

struct app_header : public app_base
{
    comma::csv::output_stream< messages::header > os;
    static void output_fields() { std::cout << comma::join( comma::csv::names< messages::header >( true ), ',' ) << std::endl; }
    static void output_format() { std::cout << comma::csv::format::value< messages::header >() << std::endl; }
    app_header( const std::string& port, const comma::command_line_options& options )
        : app_base( port, options )
        , os( std::cout, options.exists( "--binary-output" ), true, options.exists( "--flush" ) )
    {
    }
    void handle_raw( messages::header* msg_header, const char* msg_data, std::size_t msg_data_length )
    {
        os.write( *msg_header );
    }
};

template< typename T >
struct app_t : public app_base
{
    comma::csv::output_stream< T > os;
    app_t( const std::string& port, const comma::command_line_options& options )
        : app_base( port, options )
        , os( std::cout, options.exists( "--binary-output" ), true, options.exists( "--flush" ) )
    {
    }
    static void output_fields() { std::cout << comma::join( comma::csv::names< T >( true ), ',' ) << std::endl; }
    static void output_format() { std::cout << comma::csv::format::value< T >() << std::endl; }
};

/// accumulate several packets into one big output record
struct app_all : public app_t< output_all >
{
    app_all( const std::string& port, const comma::command_line_options& options ) : app_t( port, options ) {}
    output_all output;
    void handle( const messages::system_state* msg )
    {
        memcpy( output.system_state.data(), msg->data(), messages::system_state::size );
    }
    void handle( const messages::raw_sensors* msg )
    {
        std::memcpy( output.raw_sensors.data(), msg->data(), messages::raw_sensors::size );
        os.write( output );
    }
};

template< typename T >
struct app_packet : public app_t< T >
{
    app_packet( const std::string& port, const comma::command_line_options& options ) : app_t< T >( port, options ) {}
    void handle( const T* msg ) { app_t< T >::os.write( *msg ); }
};

struct factory_i
{
    virtual ~factory_i() {}
    virtual void output_fields() = 0;
    virtual void output_format() = 0;
    virtual void run( const std::string& input, const comma::command_line_options& options ) = 0;
};

template<typename T>
struct factory_t : public factory_i
{
    typedef T type;
    void output_fields() { T::output_fields(); }
    void output_format() { T::output_format(); }
    void run( const std::string& input, const comma::command_line_options& options )
    {
        T app( input, options );
        app.process();
    }
};

template< typename T >
struct description
{
    comma::csv::input_stream< status_data > is;
    description( const comma::command_line_options& options ) : is( std::cin, comma::csv::options( options ) ) {}
    void process()
    {
        while( std::cin.good() )
        {
            const status_data* p = is.read();
            if( !p ) { break; }
            std::cout << T::string( p->status ) << std::endl;
        }
    }
};

static void bash_completion( int argc, char** argv )
{
    std::cout << "--help --verbose"
        << " all raw-sensors system-state header"
        << " --device --baud-rate --sleep --output-fields --output-format --raw --stdin --description"
        << std::endl;
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) { bash_completion( argc, argv ); return 0; }
        std::vector< std::string > unnamed = options.unnamed( comma::csv::options::valueless_options() + ", --verbose, -v, --output-fields, --output-format, --raw, --stdin", "-.*" );
        auto opt_description = options.optional< std::string >( "--description" );
        if( opt_description )
        {
            if( *opt_description == "system_status" ) { description< messages::system_status_description >( options ).process(); }
            else if( *opt_description == "filter_status" ) { description< messages::filter_status_description >( options ).process(); }
            else { COMMA_THROW( comma::exception, "invalid field for description. expected 'system_status' or 'filter_status', got " << *opt_description ); }
            return 0;
        }
        std::unique_ptr<factory_i> factory;
        if( options.exists( "--raw" ) ) { factory.reset( new factory_t< app_raw >() ); }
        else if( unnamed.size() != 1 ) { COMMA_THROW( comma::exception, "expected one unnamed arguement, got: " << unnamed.size() ); }
        else if( unnamed[0] == "system-state" ) { factory.reset( new factory_t< app_packet< messages::system_state > >() ); }
        else if( unnamed[0] == "raw-sensors" ) { factory.reset( new factory_t< app_packet< messages::raw_sensors > >() ); }
        else if( unnamed[0] == "header" ) { factory.reset( new factory_t< app_header >() ); }
        else if( unnamed[0] == "all" ) { factory.reset( new factory_t< app_all >() ); }
        else { COMMA_THROW( comma::exception, "expected <what>: raw-sensors | system-state | all; got " << unnamed[0] );}
        if( options.exists( "--output-fields" ) ) { factory->output_fields(); return 0; }
        if( options.exists( "--output-format" ) ) { factory->output_format(); return 0; }
        options.assert_mutually_exclusive( "--stdin,--device" );
        std::string input = options.exists( "--stdin" ) ? "-" : options.value< std::string >( "--device" );
        factory->run( input, options );
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
