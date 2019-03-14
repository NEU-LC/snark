// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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


#include <iostream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/application/verbose.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/math/compare.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../ptcr/protocol.h"

using namespace snark;

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "control a quickset pan/tilt unit via serial port or tcp" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat commands.csv | " << comma::verbose.app_name() << " <address> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<address>: tcp address or serial port" << std::endl;
    std::cerr << "    tcp address: e.g: tcp:192.168.0.1:12345" << std::endl;
    std::cerr << "    serial port: e.g: /dev/ttyS0" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input: pan,tilt" << std::endl;
    std::cerr << "output: t,pan,tilt,status-bits" << std::endl;
    std::cerr << "    angles are in radians" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this help; --help --verbose for more help" << std::endl;
    std::cerr << "    --diff: if present, take differential input" << std::endl;
    std::cerr << "    --output-fields: print output fields and exit" << std::endl;
    std::cerr << "    --output-format: print binary format of output stream and exit" << std::endl;
    std::cerr << "    --output-if-changed: output position, only when changed" << std::endl;
    std::cerr << "    --pan-limits <limits>: set pan limits and exit; range -2PI:2PI" << std::endl;
    std::cerr << "    --tilt-limits <limits>: set tilt limits and exit; range -PI:PI" << std::endl;
    std::cerr << "        <limits> ::= <lower>:<upper> (radians)" << std::endl;
    std::cerr << "    --camera=<on/off>: turn camera power on/off and exit" << std::endl;
    std::cerr << "    --verbose,-v: more output to stderr" << std::endl;
    std::cerr << "    --debug: even more output to stderr" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output status fields" << std::endl;
    std::cerr << "    con      continuous rotation - limits ignored" << std::endl;
    std::cerr << "    exec     executing command" << std::endl;
    std::cerr << "    des      returning destination coords, not current" << std::endl;
    std::cerr << "    oslr     soft limit override" << std::endl;
    std::cerr << "    cwm      moving cw" << std::endl;
    std::cerr << "    ccwm     moving ccw" << std::endl;
    std::cerr << "    upm      moving up" << std::endl;
    std::cerr << "    dwnm     moving down" << std::endl;
    std::cerr << "    cwsl     cw soft limit reached" << std::endl;
    std::cerr << "    ccwsl    ccw soft limit reached" << std::endl;
    std::cerr << "    cwhl     cw hard limit reached" << std::endl;
    std::cerr << "    ccwhl    ccw hard limit reached" << std::endl;
    std::cerr << "    pan-to   pan timeout" << std::endl;
    std::cerr << "    pan-de   pan direction error" << std::endl;
    std::cerr << "    usl      up soft limit reached" << std::endl;
    std::cerr << "    dsl      down soft limit return" << std::endl;
    std::cerr << "    uhl      up hard limit reached " << std::endl;
    std::cerr << "    dhl      down hard limit return" << std::endl;
    std::cerr << "    tilt-to  tilt timeout" << std::endl;
    std::cerr << "    tilt-de  tilt direction error" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    (all are binary 0/1)" << std::endl;

    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "csv stream options: " << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
        std::cerr << std::endl;
    }
    std::cerr << "examples" << std::endl;
    std::cerr << "    --- absolute positioning ---" << std::endl;
    std::cerr << "    echo 0.1,0.2 | " << comma::verbose.app_name() << " tcp:192.168.1.1:10001" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- relative positioning ---" << std::endl;
    std::cerr << "    echo -0.1,0.2 | " << comma::verbose.app_name() << " --diff tcp:192.168.1.1:10001" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- get current position ---" << std::endl;
    std::cerr << "    echo 0,0 | " << comma::verbose.app_name() << " --diff tcp:192.168.1.1:10001" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- absolute positioning in degrees ---" << std::endl;
    std::cerr << "    echo 45,45 | csv-units --from degrees --fields a,b | " << comma::verbose.app_name() << " tcp:192.168.1.1:10001" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- move with arrow keys ---" << std::endl;
    std::cerr << "    io-console | control-from-console | " << comma::verbose.app_name() << " --diff tcp:192.168.1.1:10001 > /dev/null" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    --- set limits ---" << std::endl;
    std::cerr << "    " << comma::verbose.app_name() << " tcp:192.168.1.1:10001 \\" << std::endl;
    std::cerr << "        --pan-limits $( echo -180:180 | csv-units --from degrees --fields a,b -d=: ) \\" << std::endl;
    std::cerr << "        --tilt-limits $( echo -90:90 | csv-units --from degrees --fields a,b -d=: )" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

struct position // quick and dirty
{
    double pan;
    double tilt;

    position() : pan( 0 ), tilt( 0 ) {}
    position( double pan, double tilt ) : pan( pan ), tilt( tilt ) {}
    static position from_degrees( double p, double t ) { return position( p * M_PI / 180, t * M_PI / 180 ); }
    static const double resolution;
};

const double position::resolution = M_PI / 18000; // quick and dirty

struct status // quick and dirty
{
    boost::posix_time::ptime timestamp;
    quickset::ptcr::commands::get_status::response status_response;
    bool operator==( const status& rhs ) const { return ::memcmp( status_response.data(), rhs.status_response.data(), quickset::ptcr::commands::get_status::response::size ) == 0; }
    bool operator!=( const status& rhs ) const { return !operator==( rhs ); }
    position get_position() const { return position::from_degrees( double( status_response.pan() ) / 100, double( status_response.tilt() ) / 100 ); }
    status( const boost::posix_time::ptime& timestamp, quickset::ptcr::commands::get_status::response s ) : timestamp( timestamp ), status_response( s ) {}
    status() {}
};

namespace comma { namespace visiting {

template <> struct traits< position >
{
    template < typename K, typename V > static void visit( const K& key, position& t, V& v )
    {
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
    }

    template < typename K, typename V > static void visit( const K& key, const position& t, V& v )
    {
        v.apply( "pan", t.pan );
        v.apply( "tilt", t.tilt );
    }
};

template <> struct traits< status >
{
    template < typename K, typename V > static void visit( const K& key, const status& t, V& v )
    {
        v.apply( "t", t.timestamp );
        v.apply( "position", t.get_position() );

        // todo: quick and dirty, output as bitmask (or implement bitmask as csv type)
        v.apply( "con", t.status_response.status.con() );
        v.apply( "exec", t.status_response.status.exec() );
        v.apply( "des", t.status_response.status.des() );
        v.apply( "oslr", t.status_response.status.oslr() );
        v.apply( "cwm", t.status_response.status.cwm() );
        v.apply( "ccwm", t.status_response.status.ccwm() );
        v.apply( "upm", t.status_response.status.upm() );
        v.apply( "dwnm", t.status_response.status.dwnm() );
        
        v.apply( "cwsl", t.status_response.response_pan_status.cwsl() );
        v.apply( "ccwsl", t.status_response.response_pan_status.ccwsl() );
        v.apply( "cwhl", t.status_response.response_pan_status.cwhl() );
        v.apply( "ccwhl", t.status_response.response_pan_status.ccwhl() );
        v.apply( "pan-to", t.status_response.response_pan_status.to() );
        v.apply( "pan-de", t.status_response.response_pan_status.de() );
        
        v.apply( "usl", t.status_response.response_tilt_status.usl() );
        v.apply( "dsl", t.status_response.response_tilt_status.dsl() );
        v.apply( "uhl", t.status_response.response_tilt_status.uhl() );
        v.apply( "dhl", t.status_response.response_tilt_status.dhl() );
        v.apply( "tilt-to", t.status_response.response_tilt_status.to() );
        v.apply( "tilt-de", t.status_response.response_tilt_status.de() );
    }
};
    
} } // namespace comma { namespace visiting {

static bool debug = false;
static boost::scoped_ptr< quickset::ptcr::protocol > protocol;
quickset::ptcr::commands::get_status get_status;
static boost::optional< status > current_status;
static boost::optional< position > target;
static bool output_if_changed;
static boost::scoped_ptr< comma::csv::input_stream< position > > input;
static boost::scoped_ptr< comma::csv::output_stream< status > > output;
static comma::signal_flag is_shutdown;
static bool differential;

static bool handle_status()
{
    const quickset::ptcr::packet< quickset::ptcr::commands::get_status::response >* response = protocol->send( get_status, debug );
    if( !response ) { return false; }
    switch( response->packet_header.type() )
    {
        case quickset::ptcr::constants::ack:
            if( output_if_changed && current_status && response->body == current_status->status_response ) { break; }
            // todo: watch time precision; offset by transmission time?
            current_status = status( boost::posix_time::microsec_clock::universal_time(), response->body );
            output->write( *current_status );
            break;
        case quickset::ptcr::constants::nak:
            comma::verbose << get_status.name << " command failed" << std::endl;
            break;
        default: // never here
            comma::verbose << "expected ack or nak, got 0x" << std::hex << ( 0xff & response->packet_header.type() ) << std::dec << std::endl;
            break;
    }
    return true;
}

template < typename command >
static bool handle_move_to( const position& p )
{
    command move_to_cmd;
    static const double factor = double( 18000 ) / M_PI;
    move_to_cmd.pan = static_cast< int >( p.pan * factor );
    move_to_cmd.tilt = static_cast< int >( p.tilt * factor );
    comma::verbose << "sending " << move_to_cmd.name << " command: pan/tilt: " << p.pan << "," << p.tilt << " ~ " << move_to_cmd.pan() << "," << move_to_cmd.tilt() << "..." << std::endl;
    const quickset::ptcr::packet< typename command::response >* response = protocol->send( move_to_cmd, debug );
    if( !response )
    {
        std::cerr << comma::verbose.app_name() << ": failed to get response to command; resync and resend" << std::endl;
        return false;
    }
    switch( response->packet_header.type() )
    {
        case quickset::ptcr::constants::ack:
            if( response->body.response_pan_status.value() || response->body.response_tilt_status.value() )
            {
                std::cerr << comma::verbose.app_name() << ": " << move_to_cmd.name << " command (" << p.pan << "," << p.tilt << " ~ " << move_to_cmd.pan() << "," << move_to_cmd.tilt() << ") failed:" << std::endl;
                if( response->body.response_pan_status.value() ) { std::cerr << "    pan status not ok: " << response->body.response_pan_status.to_string() << std::endl; }
                if( response->body.response_tilt_status.value() ) { std::cerr << "    tilt status not ok: " << response->body.response_tilt_status.to_string() << std::endl; }
                if( response->body.response_pan_status.fault() || response->body.response_tilt_status.fault() )
                {
                    std::cerr << comma::verbose.app_name() << ": hard fault, clearing..." << std::endl;
                    return false;
                }
                if( differential ) // quick and dirty
                {
                    if(    response->body.response_pan_status.ccwhl()
                        || response->body.response_pan_status.ccwsl()
                        || response->body.response_pan_status.cwhl()
                        || response->body.response_pan_status.cwsl()
                        || response->body.response_tilt_status.uhl()
                        || response->body.response_tilt_status.usl()
                        || response->body.response_tilt_status.dhl()
                        || response->body.response_tilt_status.dsl() )
                    {
                        std::cerr << comma::verbose.app_name() << ": command failed, soft limit reached" << std::endl;
                        return false;
                    }
                }
            }
            comma::verbose << "sent command " << move_to_cmd.name << " command: pan/tilt: " << p.pan << "," << p.tilt << " ~ " << move_to_cmd.pan() << "," << move_to_cmd.tilt() << "..." << std::endl;
            break;
        case quickset::ptcr::constants::nak:
            std::cerr << comma::verbose.app_name() << ": " << move_to_cmd.name << " command failed" << std::endl;
            break;
        default: // never here
            std::cerr << comma::verbose.app_name() << ": expected ack or nak, got 0x" << std::hex << ( 0xff & response->packet_header.type() ) << std::dec << std::endl;
            return false;
    }
    return true;    
}

static bool handle_move_to()
{
    while( input->ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        if( differential )
        {
            const position* position = input->read();
            if( !position ) { return true; }
            return handle_move_to< quickset::ptcr::commands::move_to_delta >( *position );
        }
        else
        {
            if( !target )
            {
                const position* position = input->read();
                if( !position ) { return true; }
                target = *position;
            }
            if( !handle_move_to< quickset::ptcr::commands::move_to >( *target ) ) { return false; }
            target.reset();
            return true;
        }
    }
    return true;
}

typedef quickset::ptcr::commands::set_limits::direction direction;

static void set_limit( double limit, direction::values direction )
{
    bool is_pan = direction == direction::left || direction == direction::right;
    int value = limit * double( 18000 ) / M_PI;
    int maximum = is_pan ? 36000 : 18000;
    if( value < -maximum || value > maximum ) { std::cerr << comma::verbose.app_name() << ": " << ( is_pan ? "pan" : "tilt" ) << " exceeded: " << limit << std::endl; exit( 1 ); }
    target = is_pan ? position( limit, 0 ) : position( 0, limit );
    if( !handle_move_to< quickset::ptcr::commands::move_to >( *target ) ) { std::cerr << comma::verbose.app_name() << ": failed to reach " << limit << std::endl; exit( 1 ); }
    while( !handle_status() || current_status->status_response.status.exec() ) { boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::millisec( 20 ) ); }
    quickset::ptcr::commands::set_limits set_limits( direction, value );
    const quickset::ptcr::packet< quickset::ptcr::commands::set_limits::response >* response = protocol->send( set_limits, debug );
    if( !response || response->packet_header.type() != quickset::ptcr::constants::ack ) { std::cerr << comma::verbose.app_name() << ": failed to set " << ( is_pan ? "pan" : "tilt" ) << " limit " << limit << std::endl; exit( 1 );  }
    std::cerr << comma::verbose.app_name() << ": " << ( is_pan ? "pan" : "tilt" ) << " limit " << limit << " is set" << std::endl;
}

static void set_limits( boost::optional< std::string > limits, direction::values forth, direction::values back )
{
    if( !limits ) { return; }
    std::vector< std::string > v = comma::split( *limits, ':' );
    if( v.size() != 2 ) { COMMA_THROW( comma::exception, "expected limits in radians (e.g. \"-1.5:1.5\"), got \"" << *limits << "\"" ); }
    double lower = boost::lexical_cast< double >( v[0] );
    double upper = boost::lexical_cast< double >( v[1] );
    if( lower > upper ) { COMMA_THROW( comma::exception, "expected lower limits not greater than upper, got \"" << *limits << "\"" ); }
    set_limit( upper, forth );
    set_limit( lower, back );
}

static void synchronize( bool osl = false )
{
    std::cerr << comma::verbose.app_name() << ": synchronizing..." << std::endl;
    const boost::posix_time::time_duration timeout = boost::posix_time::milliseconds( 20 );
    get_status.command = quickset::ptcr::commands::get_status::command::res;
    if( osl ) { get_status.command = get_status.command() | quickset::ptcr::commands::get_status::command::osl; }
    while( !handle_status() ) { boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + timeout ); }
    std::cerr << comma::verbose.app_name() << ": synchronized" << std::endl;    
}

static void set_limits( boost::optional< std::string > pan, boost::optional< std::string > tilt )
{
    if( !pan && !tilt ) { return; }
    differential = false;
    output_if_changed = true;
    synchronize( true );
    set_limits( pan, direction::clockwise, direction::counterclockwise );
    set_limits( tilt, direction::up, direction::down );
    std::cerr << comma::verbose.app_name() << ": going home..." << std::endl;
    target = position( 0, 0 );
    handle_move_to< quickset::ptcr::commands::move_to >( *target );
    while( handle_status() && current_status->status_response.status.exec() ) { boost::thread::sleep( boost::posix_time::microsec_clock::universal_time() + boost::posix_time::milliseconds( 20 ) ); }
    std::cerr << comma::verbose.app_name() << ": done" << std::endl;
    exit( 0 );
}

static void set_camera( bool on )
{
    synchronize();
    quickset::ptcr::commands::set_camera query;
    query.flags[0] = 0x80; // quick and dirty: query
    const quickset::ptcr::packet< quickset::ptcr::commands::set_camera::response >* response = protocol->send( query, debug );
    if( !response ) { std::cerr << comma::verbose.app_name() << ": no response to query camera state" << std::endl; exit( 1 );  }
    if( response->packet_header.type() != quickset::ptcr::constants::ack ) { std::cerr << comma::verbose.app_name() << ": failed to query camera state" << std::endl; exit( 1 );  }
    //std::cerr << comma::verbose.app_name() << ": cameras turned " << ( on ? "on" : "off" ) << std::endl;
    quickset::ptcr::commands::set_camera command;
    command.flags[0] = response->body.flags[0]() & 0x7f;
    command.flags[1] = on ? 0x0c : 0x00; // quick and dirty
    response = protocol->send( command, debug );
    if( !response || response->packet_header.type() != quickset::ptcr::constants::ack ) { std::cerr << comma::verbose.app_name() << ": failed to turn cameras " << ( on ? "on" : "off" ) << std::endl; exit( 1 );  }
    std::cerr << comma::verbose.app_name() << ": cameras turned " << ( on ? "on" : "off" ) << std::endl;
    exit( 0 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< status >( false ), ',' ) << std::endl; return 0; }
        if( options.exists( "--output-format" ) ) { std::cout << comma::csv::format::value< status >() << std::endl; return 0; }
        if( options.exists( "--pan-limits,--tilt-limits,--debug" )) { comma::verbose.init( true, "" ); }
        debug = options.exists( "--debug" );
        std::vector< std::string > v = options.unnamed( "--diff,--help,-h,--verbose,-v,--debug,--output-fields,--output-format,--output-if-changed", "--binary,-b,--fields,-f,--delimiter,-d,--pan-limits,--tilt-limits,--camera" );
        if( v.empty() ) { std::cerr << comma::verbose.app_name() << ": please specify port name" << std::endl; exit( 1 ); }
        if( v.size() > 1 ) { std::cerr << comma::verbose.app_name() << ": expected one serial port name, got \"" << comma::join( v, ' ' ) << std::endl; exit( 1 ); }
        std::string name = v[0];
        output_if_changed = options.exists( "--output-if-changed" );
        differential = options.exists( "--diff" );
        comma::verbose << "connecting to " << name << "..." << std::endl;
        protocol.reset( new quickset::ptcr::protocol( name ) );
        comma::verbose << "connected to " << name << std::endl;
        comma::csv::options input_csv( options );
        input_csv.full_xpath = false;
        comma::csv::options output_csv;
        output_csv.full_xpath = false;
        if( input_csv.fields == "" )
        {
            input_csv.fields = "pan,tilt";
            if( input_csv.binary() ) { output_csv.format( "t,2d,20b" ); }
        }
        output.reset( new comma::csv::output_stream< status >( std::cout, output_csv ) );
        set_limits( options.optional< std::string >( "--pan-limits" ), options.optional< std::string >( "--tilt-limits" ) );
        if( options.exists( "--camera" ) ) { set_camera( options.value< std::string >( "--camera" ) == "on" ); }
        input.reset( new comma::csv::input_stream< position >( std::cin, input_csv ) );
        comma::io::select select;
        select.read().add( 0 );
        const boost::posix_time::time_duration timeout = boost::posix_time::millisec( 20 );
        boost::posix_time::ptime deadline = boost::posix_time::microsec_clock::universal_time();
        bool ok = false;
        comma::verbose << "synchronizing..." << std::endl;
        // Consume any messages already queued for reading
        while( protocol->receive( boost::posix_time::millisec( 500 ))) {}
        while( !is_shutdown && std::cin.good() && !std::cin.eof() && std::cout.good() )
        {
            boost::posix_time::ptime now = boost::posix_time::microsec_clock::universal_time();
            bool was_ok = ok;
            get_status.command = ok ? 0 : quickset::ptcr::commands::get_status::command::res;
            if( now > deadline || get_status.command() ) { ok = handle_status(); deadline = now + timeout; }
            if( was_ok != ok ) { std::cerr << comma::verbose.app_name() << ": " << ( ok ? "synchronized" : "lost sync, synchronizing..." ) << std::endl; }
            if( ok )
            {
                if( target || select.wait( timeout ) != 0 ) { ok = handle_move_to(); }
            }
            else
            {
                boost::thread::sleep( deadline );
            }
        }
        if( is_shutdown ) { std::cerr << comma::verbose.app_name() << ": interrupted by signal" << std::endl; }
        protocol->close();
        comma::verbose << "done" << std::endl;
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
    if( protocol ) { protocol->close(); }
    exit( 0 );
}
