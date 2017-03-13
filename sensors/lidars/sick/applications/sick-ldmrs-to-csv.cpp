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


#include <boost/lexical_cast.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../../../../math/range_bearing_elevation.h"
#include "../../../../timing/ntp.h"
#include "../../../../visiting/eigen.h"
#include "../../../../visiting/traits.h"
#include "../ibeo/protocol.h"
#include <fcntl.h>

struct csv_point
{
    boost::posix_time::ptime timestamp;
    ::Eigen::Matrix< double, 3, 1 > cartesian;
    snark::range_bearing_elevation polar;
    comma::uint32 layer;
    comma::uint32 echo;
    comma::uint32 what;
    double width;
    comma::uint32 scan;
};

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "take sick ldmrs data feed on stdin, output csv data to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage" << std::endl;
    std::cerr << "    netcat pantilt 1234 | sick-ldmrs-to-csv [<options>]" << std::endl;
    std::cerr << "    cat ldmrs.log | sick-ldmrs-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show this message" << std::endl;
    std::cerr << "    --binary,-b: output binary equivalent of csv" << std::endl;
    std::cerr << "    --echo <threshold>: output only echoes <= threshold (0-3); default: 3 (all)" << std::endl;
    std::cerr << "    --fields=<fields>: output only given fields" << std::endl;
    std::cerr << "        default: " << comma::join( comma::csv::names< csv_point >( false ), ',' ) << " (" << comma::csv::format::value< csv_point >() << ")" << std::endl;
    std::cerr << "        t: timestamp" << std::endl;
    std::cerr << "        x,y,z: cartesian coordinates in sensor frame" << std::endl;
    std::cerr << "        range,bearing,elevation or r,b,e: polar coordinates in sensor frame" << std::endl;
    std::cerr << "        id: laser id (or \"layer\": 0-3)" << std::endl;
    std::cerr << "        echo: echo number (0-2)" << std::endl;
    std::cerr << "        what: combined value of the bits from the 8-bit sensor flags: transparent (0x01), rain,dust,or similar atmospheric noise (0x02), dirt (0x08)" << std::endl;
    std::cerr << "              register bits in the range 0xF0 are reserved as stated by the LD-MRS manual." << std::endl;
    std::cerr << "        width: echo width in metres" << std::endl;
    std::cerr << "    --format: output binary format for given fields to stdout and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "author:" << std::endl;
    std::cerr << "    vsevolod vlaskine, v.vlaskine@acfr.usyd.edu.au" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

namespace comma { namespace visiting {

template <> struct traits< csv_point >
{
    template < typename Key, class Visitor >
    static void visit( const Key&, const csv_point& p, Visitor& v )
    {
        v.apply( "t", p.timestamp );
        v.apply( "cartesian", p.cartesian );
        v.apply( "polar", p.polar );
        v.apply( "layer", p.layer );
        v.apply( "echo", p.echo );
        v.apply( "what", p.what );
        v.apply( "width", p.width );
        v.apply( "scan", p.scan );
    }
};

} } // namespace comma { namespace visiting {

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        unsigned int threshold = options.value( "--echo", 3 );
        comma::csv::options csv;
        csv.fields = options.value< std::string >( "--fields", "" );
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i ) // convenience shortcuts
        {
            if( v[i] == "r" ) { v[i] = "range"; }
            else if( v[i] == "b" ) { v[i] = "bearing"; }
            else if( v[i] == "e" ) { v[i] = "elevation"; }
            else if( v[i] == "id" ) { v[i] = "layer"; }
            else if( v[i] == "block" ) { v[i] = "scan"; }
        }
        csv.fields = comma::join( v, ',' );
        csv.full_xpath = false;
        if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< csv_point >( csv.fields, false ) << std::endl; return 0; }
        if( options.exists( "--binary,-b" ) ) { csv.format( comma::csv::format::value< csv_point >( csv.fields, false ) ); }
        comma::csv::output_stream< csv_point > ostream( std::cout, csv );
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        #endif
        snark::sick::ibeo::protocol protocol( std::cin );
        while( !std::cin.eof() )
        {
            const snark::sick::ibeo::scan_packet* p;
            try { p = protocol.readscan(); }
            catch( snark::sick::ibeo::protocol::faultException& ex ) { std::cerr << "sick-ldmrs-to-csv: " << ex.what() << std::endl; continue; }
            catch( comma::exception& ex ) { std::cerr << "sick-ldmrs-to-csv: " << ex.what() << std::endl; continue; }
            if( p == NULL ) { std::cerr << "sick-ldmrs-to-csv: done" << std::endl; return 0; }

            std::size_t count = p->packet_scan.scan_header.points_count();
            snark::sick::ibeo::scan::timestamps timestamps( p->packet_scan );
            csv_point point;
            point.scan = p->packet_scan.scan_header.measurement_number();
            for( std::size_t i = 0; i < count; ++i )
            {
                point.echo = p->packet_scan.points()[i].id.echo();
                if( point.echo > threshold ) { continue; }
                point.polar.range( double( p->packet_scan.points()[i].range() ) / 100 );
                point.polar.bearing( p->packet_scan.angle_as_radians( p->packet_scan.points()[i] ) );
                point.polar.elevation( p->packet_scan.points()[i].elevation() );
                point.cartesian = point.polar.to_cartesian();
                point.layer = p->packet_scan.points()[i].id.layer();
                point.what = p->packet_scan.points()[i].flags();
                point.width = double( p->packet_scan.points()[i].echo_pulse_width() ) / 100;
                point.timestamp = timestamps[i];
                ostream.write( point );
            }
        }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "sick-ldmrs-to-csv: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "sick-ldmrs-to-csv: unknown exception" << std::endl;
    }
    usage();
}
