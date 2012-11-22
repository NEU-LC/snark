// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#include <pcap.h>
#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/shared_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/io/publisher.h>
#include <comma/math/compare.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <snark/timing/time.h>
#include <snark/sensors/velodyne/stream.h>
#include <snark/sensors/velodyne/thin/thin.h>
#include <snark/sensors/velodyne/impl/pcap_reader.h>
#include <snark/sensors/velodyne/impl/proprietary_reader.h>
#include <snark/sensors/velodyne/impl/stdin_reader.h>
#include <snark/sensors/velodyne/impl/stream_traits.h>
#include <snark/sensors/velodyne/impl/udp_reader.h>
#include <snark/sensors/velodyne/thin/scan.h>

using namespace snark;

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "Takes velodyne packets on stdin and outputs thinned packets to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Usage: cat velodyne*.bin | velodyne-thin <options>" << std::endl;
    std::cerr << "       netcat shrimp.littleboard 12345 | velodyne-thin <options>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "velodyne options" << std::endl;
    std::cerr << "    --db <velodyne db.xml file>: default /usr/local/etc/db.xml" << std::endl;
    std::cerr << std::endl;
    std::cerr << "data flow options" << std::endl;
    std::cerr << "    --output-raw: if present, output uncompressed thinned packets" << std::endl;
    std::cerr << "    --pcap: if present, velodyne data is read from pcap packets" << std::endl;
    std::cerr << "             e.g: cat velo.pcap | velodyne-thin <options> --pcap" << std::endl;
    std::cerr << "    --proprietary,-q : read velodyne data directly from stdin using the proprietary protocol" << std::endl;
    std::cerr << "        <header, 16 bytes><timestamp, 12 bytes><packet, 1206 bytes><footer, 4 bytes>" << std::endl;
    std::cerr << "    default input format: <timestamp, 8 bytes><packet, 1206 bytes>" << std::endl;
    std::cerr << "    --publish=<address>: if present, publish on given address (see io-publish -h for address syntax)" << std::endl;
    std::cerr << "    --verbose,-v" << std::endl;
    std::cerr << std::endl;
    std::cerr << "filtering options" << std::endl;
    std::cerr << "    --udp-port <port>: if present, read raw velodyne packets from udp and timestamp them" << std::endl;
    std::cerr << "    --rate <rate>: thinning rate between 0 and 1" << std::endl;
    std::cerr << "                    default 1: send all valid datapoints" << std::endl;
    std::cerr << "    --scan-rate <rate>: scan thin rate between 0 and 1" << std::endl;
    std::cerr << "    --focus <options>: focus on particular region" << std::endl;
    std::cerr << "                        e.g. at choosen --rate in the direction of" << std::endl;
    std::cerr << "                        0 degrees 30 degrees wide not farther than 10 metres" << std::endl;
    std::cerr << "                        output 80% points in the focus region and 20% the rest" << std::endl;
    std::cerr << "                        --focus=\"sector;range=10;bearing=0;ken=30;ratio=0.8\"" << std::endl;
    std::cerr << "                        todo: currently only \"sector\" type implemented" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

static bool verbose = false;
static bool outputRaw = false;
static boost::optional< float > rate;
static boost::optional< double > scanRate;
static boost::optional< double > angularSpeed_;
static boost::optional< velodyne::db > db;
static boost::scoped_ptr< velodyne::thin::focus > focus;
static velodyne::thin::scan scan;
static boost::scoped_ptr< comma::io::publisher > publisher;

static double angularSpeed( const snark::velodyne::packet& packet )
{
    if( angularSpeed_ ) { return *angularSpeed_; }
    double da = double( packet.blocks[0].rotation() - packet.blocks[11].rotation() ) / 100;
    double dt = double( ( snark::velodyne::impl::time_offset( 0, 0 ) - snark::velodyne::impl::time_offset( 11, 0 ) ).total_microseconds() ) / 1e6;
    return da / dt;
}

static velodyne::thin::focus* makefocus( const std::string& options, double rate ) // quick and dirty
{
    std::string type = comma::name_value::map( options, "type" ).value< std::string >( "type" );
    double ratio = comma::name_value::map( options ).value( "ratio", 1.0 );
    velodyne::thin::region* region;
    if( type == "sector" ) { region = new velodyne::thin::sector( comma::name_value::parser().get< velodyne::thin::sector >( options ) ); }
    else { COMMA_THROW( comma::exception, "expected type (sector), got " << type ); }
    velodyne::thin::focus* focus = new velodyne::thin::focus( rate, ratio );
    focus->insert( 0, region );
    return focus;
}

template < typename S >
void run( S* stream )
{
    static const unsigned int timeSize = 12;
    boost::mt19937 generator;
    boost::uniform_real< float > distribution( 0, 1 );
    boost::variate_generator< boost::mt19937&, boost::uniform_real< float > > random( generator, distribution );
    comma::uint64 count = 0;
    double compression = 0;
    velodyne::packet packet;
    comma::signal_flag isShutdown;
    while( !isShutdown && std::cin.good() && !std::cin.eof() && std::cout.good() && !std::cout.eof() )
    {
        const char* p = velodyne::impl::stream_traits< S >::read( *stream, sizeof( velodyne::packet ) );
        if( p == NULL ) { break; }
        ::memcpy( &packet, p, velodyne::packet::size );
        boost::posix_time::ptime timestamp = stream->timestamp();
        if( scanRate ) { scan.thin( packet, *scanRate, angularSpeed( packet ) ); }
        if( !scan.empty() )
        {
            if( focus ) { velodyne::thin::thin( packet, *focus, *db, angularSpeed( packet ), random ); }
            else if( rate ) { velodyne::thin::thin( packet, *rate, random ); }
        }
        const boost::posix_time::ptime base( snark::timing::epoch );
        const boost::posix_time::time_duration d = timestamp - base;
        comma::int64 seconds = d.total_seconds();
        comma::int32 nanoseconds = static_cast< comma::int32 >( d.total_microseconds() % 1000000 ) * 1000;
        if( outputRaw ) // real quick and dirty
        {
            static boost::array< char, 16 + timeSize + velodyne::packet::size + 4 > buf;
            static const boost::array< char, 2 > start = {{ -78, 85 }}; // see QLib::Bytestreams::GetDefaultStartDelimiter()
            static const boost::array< char, 2 > end = {{ 117, -97 }}; // see QLib::Bytestreams::GetDefaultStartDelimiter()
            ::memcpy( &buf[0], &start[0], 2 );
            ::memcpy( &buf[0] + buf.size() - 2, &end[0], 2 );
            ::memcpy( &buf[0] + 16, &seconds, 8 );
            ::memcpy( &buf[0] + 16 + 8, &nanoseconds, 4 );
            ::memcpy( &buf[0] + 16 + 8 + 4, &packet, velodyne::packet::size );
            if( publisher )
            {
                publisher->write( &buf[0], buf.size() );
            }
            else
            {
                std::cout.write( &buf[0], buf.size() );
            }
        }
        else
        {
            static char buf[ timeSize + sizeof( comma::uint16 ) + velodyne::thin::maxBufferSize ];
            comma::uint16 size = timeSize + velodyne::thin::serialize( packet, buf + timeSize + sizeof( comma::uint16 ) );
            ::memcpy( buf, &size, sizeof( comma::uint16 ) );
            ::memcpy( buf + sizeof( comma::uint16 ), &seconds, sizeof( comma::int64 ) );
            ::memcpy( buf + sizeof( comma::uint16 ) + sizeof( comma::int64 ), &nanoseconds, sizeof( comma::int32 ) );
            if( publisher )
            {
                publisher->write( buf, size + sizeof( comma::uint16 ) );
            }
            else
            {
                std::cout.write( buf, size + sizeof( comma::uint16 ) );
            }
            if( verbose )
            {
                ++count;
                compression = 0.9 * compression + 0.1 * ( double( size + sizeof( comma::int16 ) ) / ( velodyne::packet::size + timeSize ) );
                if( count % 10000 == 0 ) { std::cerr << "velodyne-thin: processed " << count << " packets; compression rate " << compression << std::endl; }
            }
        }
    }
    if( publisher ) { publisher->close(); }
    std::cerr << "velodyne-thin: " << ( isShutdown ? "signal received" : "no more data" ) << "; shutdown" << std::endl;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        outputRaw = options.exists( "--output-raw" );
        rate = options.optional< float >( "--rate" );
        scanRate = options.optional< double >( "--scan-rate" );
        if( options.exists( "--publish" ) ) { publisher.reset( new comma::io::publisher( options.value< std::string >( "--publish" ), comma::io::mode::binary ) ); }
        options.assert_mutually_exclusive( "--focus,--subtract-by-age,--subtract-max-range,--subtract" );
        if( options.exists( "--focus,--subtract-by-age,--subtract-max-range,--subtract" ) )
        {
            db = velodyne::db( options.value< std::string >( "--db", "/usr/local/etc/db.xml" ) );
        }
        if( options.exists( "--focus" ) )
        {
            focus.reset( makefocus( options.value< std::string >( "--focus" ), rate ? *rate : 1.0 ) );
            std::cerr << "velodyne-thin: rate in focus: " << focus->rate_in_focus() << "; rate out of focus: " << focus->rate_out_of_focus() << "; coverage: " << focus->coverage() << std::endl;
        }
        verbose = options.exists( "--verbose,-v" );
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif
        options.assert_mutually_exclusive( "--pcap,--udp-port,--proprietary,-q" );
        boost::optional< unsigned short > port = options.optional< unsigned short >( "--udp-port" );
        if( port ) { run( new snark::udp_reader( *port ) ); }
        else if( options.exists( "--pcap" ) ) { run( new snark::pcap_reader ); }
        else if( options.exists( "--proprietary,-q" ) )
        {
            run( new snark::proprietary_reader );
            
        }
        else
        {
            run( new snark::stdin_reader );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "velodyne-thin: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "velodyne-thin: unknown exception" << std::endl; }
    usage();
}
