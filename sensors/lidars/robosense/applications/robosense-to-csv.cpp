// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2018 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

// snark is a generic and flexible library for robotics research
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

/// @author vsevolod vlaskine

#include <fstream>
#include <memory>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include "../../../../timing/time.h"
#include "../../../../visiting/traits.h"
#include "../calculator.h"
#include "../packet.h"

static void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "usage: cat robosense*.bin | robosense-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "limitations" << std::endl;
    std::cerr << "    only rs-lidar-16 currently supported" << std::endl;
    std::cerr << "    reflectivity curves not implemented yet" << std::endl;
    std::cerr << std::endl;
    std::cerr << "calibration options" << std::endl;
    std::cerr << "    --calibration,-c=<directory>; directory containing calibration files: angle.csv, ChannelNum.csv, curves.csv etc" << std::endl;
    std::cerr << "    --calibration-angles,--angles,--angle,-a=<filename>; default: as in spec" << std::endl;
    std::cerr << "    --calibration-angles-output,--output-calibration-angles,--output-angles=<how>; output vertical angles to stdout; if --difop present, take vertical angles from difop packet" << std::endl;
    std::cerr << "    --calibration-channels,--channels=<filename>; default: 450 (i.e. 4.50cm) for all channels" << std::endl;
    std::cerr << "    --range-resolution,--resolution=<metres>; default=0.01, but you most likely will want 0.005 (alternatively, --difop will contain range resolution" << std::endl;
    std::cerr << "    --output-range-resolution; if --difop present, output resolution as difop says; otherwise output default resolution" << std::endl;
    std::cerr << std::endl;
    std::cerr << "difop options" << std::endl;
    std::cerr << "    --difop=[<path>]; file or stream containing timestamped difop packets; if present, calibration data will taken from difop packets" << std::endl;
    std::cerr << "                      currently only calibrated vertical angles are supported" << std::endl;
    std::cerr << "                      '-' means difop packets are on stdin (makes sense only with --output-angles or alike" << std::endl;
    std::cerr << "    --difop-max-number-of-packets,--difop-max=<num>; max number of difop packets to read; if not specified, read till the end of file/stream" << std::endl;
    std::cerr << "    --force; use if robosense-to-csv suggests it" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output options:" << std::endl;
    std::cerr << "    --binary,-b[=<format>]: output in binary equivalent of csv" << std::endl;
    std::cerr << "    --fields <fields>: e.g. t,x,y,z,scan" << std::endl;
    std::cerr << "    --output-fields: todo: print output fields and exit" << std::endl;
    std::cerr << "    --output-invalid-points: output invalid points" << std::endl;
    std::cerr << "    --scan-discard-incomplete,--discard-incomplete-scans,--discard-incomplete: don't output scans with missing packets" << std::endl;
    std::cerr << "    --scan-max-missing-packets,--missing-packets=<n>; default 5; number of consecutive missing packets for new/invalid scan (as a rule of thumb: roughly at 20rpm 50 packets per revolution)" << std::endl;
    std::cerr << "    --temperature,-t=<celcius>; default=20; integer from 0 to 39" << std::endl;
    std::cerr << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    data" << std::endl;
    std::cerr << "        default config" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv" << std::endl;
    std::cerr << "        config from difop" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv --difop timestamped-difop.bin" << std::endl;
    std::cerr << "        config from calibration directory" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv --calibration my-calibration-dir" << std::endl;
    std::cerr << "        configure vertical pitch only" << std::endl;
    std::cerr << "            cat timestamped-msop.bin | robosense-to-csv --calibration-angles angles.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    difop" << std::endl;
    std::cerr << "        output angles, if present in difop" << std::endl;
    std::cerr << "            cat timestamped-difop.bin | robosense-to-csv --difop - --output-angles > angles.csv" << std::endl;
    std::cerr << "        output angles, if present in difop, otherwise default angles" << std::endl;
    std::cerr << "            cat timestamped-difop.bin  | robosense-to-csv --difop - --output-angles --force > angles.csv" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

namespace comma { namespace visiting {
    
template <> struct traits< snark::robosense::calculator::point >
{
    template < typename Key, class Visitor > static void visit( const Key&, const snark::robosense::calculator::point& p, Visitor& v )
    {
        v.apply( "t", p.t );
        v.apply( "scan", p.scan );
        v.apply( "id", p.id );
        v.apply( "range", p.range );
        v.apply( "bearing", p.bearing );
        v.apply( "elevation", p.elevation );
        v.apply( "reflectivity", p.reflectivity );
        v.apply( "coordinates", p.coordinates );
    }
};

} } // namespace comma { namespace visiting {

template < typename P >
static std::pair< boost::posix_time::ptime, P* > read( std::istream& is, char* buffer )
{
    comma::uint64 microseconds;
    std::pair< boost::posix_time::ptime, P* > p( boost::posix_time::not_a_date_time, NULL );
    is.read( reinterpret_cast< char* >( &microseconds ), sizeof( comma::uint64 ) );
    if( is.gcount() < int( sizeof( comma::uint64 ) ) || is.bad() || is.eof() ) { return p; }
    is.read( buffer, P::size );
    if( is.gcount() < int( P::size ) || is.bad() || is.eof() ) { return p; }
    comma::uint64 seconds = microseconds / 1000000; //to avoid time overflow on 32bit systems with boost::posix_time::microseconds( m_microseconds ), apparently due to a bug in boost
    microseconds = microseconds % 1000000;
    static boost::posix_time::ptime epoch( snark::timing::epoch );
    p.first = epoch + boost::posix_time::seconds( seconds ) + boost::posix_time::microseconds( microseconds );
    p.second = reinterpret_cast< P* >( buffer );
    return p;
}

// todo? axis directions; frame -> n-e-d
// todo? temperature from difop
// todo? curves from difop
// todo? move msop::packet::const_iterator to calculator
// todo? --distance-resolution: 1cm, 0.5cm, etc.
// todo? 32-beam support?
// todo? move difop stream to calculator?

snark::robosense::calculator make_calculator( const comma::command_line_options& options ) // todo? quick and dirty; move to calculator?
{
    options.assert_mutually_exclusive( "--calibration,-c", "--calibration-angles,--angles,--angle,-a" );
    options.assert_mutually_exclusive( "--calibration,-c", "--calibration-channels,--channels" );
    options.assert_mutually_exclusive( "--difop", "--calibration-angles,--angles,--angle,-a" );
    options.assert_mutually_exclusive( "--difop", "--calibration-channels,--channels" );
    options.assert_mutually_exclusive( "--difop", "--calibration,-c" );
    std::string calibration = options.value< std::string >( "--calibration,-c", "" );
    std::string angles = options.value< std::string >( "--calibration-angles,--angles,--angle,-a", "" );
    std::string channels = options.value< std::string >( "--calibration-channels,--channels", "" );
    std::string difop = options.value< std::string >( "--difop", "" );
    options.assert_mutually_exclusive( "" );
    if( difop.empty() )
    {
        double range_resolution = options.value( "--range-resolution,--resolution", 0.01 );
        if( calibration.empty() )
        {
            if( !angles.empty() ) { std::cerr << "robosense-to-csv: config: angles from --angles" << std::endl; }
            if( !channels.empty() ) { std::cerr << "robosense-to-csv: config: channels from --calibration-channels" << std::endl; }
            return snark::robosense::calculator( angles, channels, range_resolution );
        }
        std::cerr << "robosense-to-csv: config from calibration directory: " << calibration << std::endl;
        return snark::robosense::calculator( calibration + "/angle.csv", calibration + "/ChannelNum.csv", range_resolution );
    }
    std::cerr << "robosense-to-csv: config from difop" << std::endl;
    unsigned int difop_max_number_of_packets = options.value( "--difop-max-number-of-packets,--difop-max", 0 );
    typedef std::pair< boost::posix_time::ptime, snark::robosense::difop::packet* > pair_t;
    comma::io::istream is( difop );
    std::vector< char > buffer( snark::robosense::difop::packet::size );
    unsigned int count = 0;
    unsigned int difop_count = 0;
    pair_t p( boost::posix_time::ptime(), NULL );
    for( count = 0; is->good() && !is->eof() && ( difop_max_number_of_packets == 0 || count < difop_max_number_of_packets ); ++count )
    {
        pair_t q = read< snark::robosense::difop::packet >( *is, &buffer[0] );
        if( !q.second ) { break; }
        if( q.second->header.valid() )
        {
            ++difop_count;
            if( q.second->data.corrected_vertical_angles.empty() ) { continue; }
            p = q;
            break;
        }
    }
    if( !p.second )
    { 
        std::cerr << "robosense-to-csv: got no non-zero corrected vertical angles in " << difop_count << " DIFOP packet(s) (total packet count: " << count << ") in '" << difop << "'" << std::endl;
        if( options.exists( "--force" ) )
        {
            std::cerr << "robosense-to-csv: using defaults for corrected vertical angles" << std::endl;
            return snark::robosense::calculator();
        }
        std::cerr << "robosense-to-csv: use --force to override (defaults will be used)" << std::endl;
        exit( 1 );
    }
    std::cerr << "robosense-to-csv: got DIFOP data in packet " << count << " in '" << difop << "'" << std::endl;
    std::array< double, snark::robosense::msop::packet::data_t::number_of_lasers > elevation;
    for( unsigned int i = 0; i < elevation.size(); ++i ) { elevation[i] = p.second->data.corrected_vertical_angles.as_radians( i ); }
    return snark::robosense::calculator( elevation, p.second->data.top_board_firmware_version.range_resolution() );
}

static comma::csv::options csv;
static snark::robosense::calculator calculator;
static unsigned int temperature;
static bool output_invalid_points;

void write( const boost::posix_time::ptime& t, const snark::robosense::msop::packet* p, comma::uint32 id )
{
    static comma::csv::output_stream< snark::robosense::calculator::point > ostream( std::cout, csv );
    for( snark::robosense::msop::packet::const_iterator it( p ); !it.done() && std::cout.good(); ++it )
    { 
        if( it->valid() || output_invalid_points ) { ostream.write( calculator.make_point( id, t, it, temperature ) ); }
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::robosense::calculator::point >( false ), ',' ) << std::endl; return 0; }
        output_invalid_points = options.exists( "--output-invalid-points" );
        std::vector< char > buffer( snark::robosense::msop::packet::size );
        calculator = make_calculator( options );
        if( options.exists( "--output-range-resolution" ) ) { std::cout << calculator.range_resolution() << std::endl; return 0; }
        if( options.exists( "--calibration-angles-output,--output-calibration-angles,--output-angles" ) )
        {
            const auto& how = options.value< std::string >( "--calibration-angles-output,--output-calibration-angles,--output-angles" );
            double factor = 0;
            if( how == "radians" ) { factor = 1; }
            else if( how == "degrees" ) { factor = 180. / M_PI; }
            else
            { 
                std::cerr << "robosense-to-csv: please specify --calibration-angles-output=degrees or --calibration-angles-output=radians"  << std::endl;
                std::cerr << "robosense-to-csv: ATTENTION: angles in the configuration file (e.g. angle.csv) should be in DEGREES"  << std::endl;
                return 1;
            }
            for( auto a: calculator.elevation() ) { std::cout << ( a * factor ) << std::endl; }
            return 0;
        }
        temperature = options.value( "--temperature,-t", 20 );
        if( temperature > 40 ) { std::cerr << "robosense-to-csv: expected temperature between 0 and 40; got: " << temperature << std::endl; return 1; }
        snark::robosense::calculator::scan scan( options.value( "--scan-max-missing-packets,--missing-packets", 10 ) );
        bool discard_incomplete_scans = options.exists( "--scan-discard-incomplete,--discard-incomplete-scans,--discard-incomplete" );
        csv = comma::csv::options( options );
        csv.full_xpath = false;
        //std::vector< std::pair< boost::posix_time::ptime, std::array< char, snark::robosense::msop::packet::size > > > scan_buffer;
        std::vector< std::pair< boost::posix_time::ptime, snark::robosense::msop::packet > > scan_buffer;
        if( discard_incomplete_scans ) { scan_buffer.reserve( 120 ); } // quick and dirty
        while( std::cin.good() && !std::cin.eof() )
        {
            auto p = read< snark::robosense::msop::packet >( std::cin, &buffer[0] );
            if( !p.second ) { break; }
            if( !p.second->valid() ) { continue; }
            scan.update( p.first, *p.second );
            if( discard_incomplete_scans )
            {
                if( scan.current().is_new() )
                {
                    if( scan.is_complete( scan.last() ) ) { for( const auto& b: scan_buffer ) { write( b.first, &b.second, scan.last().id ); } }
                    scan_buffer.clear();
                }
                scan_buffer.push_back( std::pair< boost::posix_time::ptime, snark::robosense::msop::packet >() );
                scan_buffer.back().first = p.first;
                scan_buffer.back().second = *p.second;
            }
            else
            {
                write( p.first, p.second, scan.current().id );
            }
        }
        if( discard_incomplete_scans )
        {
            if( scan.is_complete( scan.current() ) ) { for( const auto& b: scan_buffer ) { write( b.first, &b.second, scan.last().id ); } }
            scan_buffer.clear();
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "robosense-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "robosense-to-csv: unknown exception" << std::endl; }
    return 1;
}
