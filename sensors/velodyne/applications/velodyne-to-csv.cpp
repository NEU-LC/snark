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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <boost/optional.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/csv/format.h>
#include <comma/csv/names.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include <snark/sensors/velodyne/impl/pcap_reader.h>
#include <snark/sensors/velodyne/impl/proprietary_reader.h>
#include <snark/sensors/velodyne/impl/thin_reader.h>
#include <snark/sensors/velodyne/impl/udp_reader.h>
#include <snark/sensors/velodyne/impl/stdin_reader.h>
#include <snark/sensors/velodyne/impl/velodyne_stream.h>

//#include <google/profiler.h>

using namespace snark;

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "Takes velodyne packets and outputs coordinate datapoints to stdout" << std::endl;
    std::cerr << "Usage: cat velodyne*.bin | velodyne-to-csv <options>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input options" << std::endl;
    std::cerr << "    default : read velodyne data directly from stdin in the format: <timestamp><packet>" << std::endl;
    std::cerr << "              <timestamp>: 8-byte unsigned int, microseconds from linux epoch" << std::endl;
    std::cerr << "              <packet>: regular velodyne 1206-byte packet" << std::endl;
    std::cerr << "    --db <db.xml file> ; default /usr/local/etc/db.xml" << std::endl;
    std::cerr << "    --pcap : if present, velodyne data is read from pcap packets" << std::endl;
    std::cerr << "    --thin : if present, velodyne data is thinned (e.g. by velodyne-thin)" << std::endl;
    std::cerr << "    --udp-port <port> : read velodyne data directly from udp port" << std::endl;
    std::cerr << "    --proprietary,-q : read velodyne data directly from stdin using the proprietary protocol" << std::endl;
    std::cerr << "        <header, 16 bytes><timestamp, 12 bytes><packet, 1206 bytes><footer, 4 bytes>" << std::endl;
    std::cerr << "    default input format: <timestamp, 8 bytes><packet, 1206 bytes>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "output options:" << std::endl;
    std::cerr << "    --binary,-b[=<format>]: if present, output in binary equivalent of csv" << std::endl;
    std::cerr << "    --fields <fields>: e.g. t,x,y,z,scan" << std::endl;
    std::cerr << "    --format: output full binary format and exit (see examples)" << std::endl;
    std::cerr << "    --min-range=<value>: do not output points closer than <value>; default 0" << std::endl;
    std::cerr << "    --output-invalid-points: output also invalid laser returns" << std::endl;
    std::cerr << "    --scans [<from>]:[<to>] : output only scans in given range" << std::endl;
    std::cerr << "                               e.g. 1:3 for scans 1, 2, 3" << std::endl;
    std::cerr << "                                    5: for scans 5, 6, ..." << std::endl;
    std::cerr << "                                    :3 for scans 0, 1, 2, 3" << std::endl;
    std::cerr << "    default output columns: " << comma::join( comma::csv::names< velodyne_point >(), ',' ) << std::endl;
    std::cerr << "    default binary format: " << comma::csv::format::value< velodyne_point >() << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "    output csv points to file:" << std::endl;
    std::cerr << "    cat raw/*.bin | velodyne-to-csv --db db.xml > velodyne.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    output csv only valid points to file:" << std::endl;
    std::cerr << "    cat raw/*.bin | velodyne-to-csv --db db.xml | grep \",1$\" > velodyne.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    output binary points to file, then view file (colour by laser id):" << std::endl;
    std::cerr << "    (the output could be directed straight to view-points," << std::endl;
    std::cerr << "    just the command line would be longer)" << std::endl;
    std::cerr << "    raw/*.bin | velodyne-to-csv --db db.xml --binary | > velodyne.bin" << std::endl;
    std::cerr << "    cat velodyne.bin | view-points --fields \",id,,,,,x,y,z\" --binary $(velodyne-to-csv --format)" << std::endl;
    std::cerr << std::endl;
    std::cerr << "copyright (c) 2011 Australian Centre for Field Robotics" << std::endl;
    std::cerr << "                   http://www.acfr.usyd.edu.au/" << std::endl;
    std::cerr << std::endl;
    std::cerr << "authors" << std::endl;
    std::cerr << "    Vsevolod Vlaskine, v.vlaskine@acfr.usyd.edu.au" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

template < typename S >
inline static void run( velodyne_stream< S >& v, const comma::csv::options& csv, double min_range )
{
    comma::signal_flag isShutdown;
    comma::csv::output_stream< velodyne_point > ostream( std::cout, csv );
    //Profilerstart( "velodyne-to-csv.prof" );{
    while( !isShutdown && v.read() ) { if( v.point().range > min_range ) { ostream.write( v.point() ); } }
    //Profilerstop(); }
    if( isShutdown ) { std::cerr << "velodyne-to-csv: interrupted by signal" << std::endl; }
    else { std::cerr << "velodyne-to-csv: done, no more data" << std::endl; }
}

static std::string fields_( const std::string& s ) // parsing fields, quick and dirty
{
    if( s == "" ) { return s; }
    std::vector< std::string > v = comma::split( s, ',' );
    for( std::size_t i = 0; i < v.size(); ++i )
    {
        if( v[i] == "x" ) { v[i] = "ray/second/x"; }
        else if( v[i] == "y" ) { v[i] = "ray/second/y"; }
        else if( v[i] == "z" ) { v[i] = "ray/second/z"; }
    }
    return comma::join( v, ',' );
}

static comma::csv::format format_( const std::string& s, const std::string& fields )
{
    if( !s.empty() ) { try { return comma::csv::format( s ); } catch( ... ) {} }
    if( fields.empty() ) { return comma::csv::format::value< velodyne_point >(); }
    std::vector< std::string > v = comma::split( fields, ',' );
    comma::csv::format format;
    for( std::size_t i = 0; i < v.size(); ++i )
    {
        if( v[i] == "t" ) { format += "t"; }
        else if( v[i] == "id" ) { format += "ui"; }
        else if( v[i] == "intensity" ) { format += "ui"; }
        else if( v[i] == "valid" ) { format += "b"; }
        else if( v[i] == "scan" ) { format += "ui"; }
        else if( v[i] == "ray" ) { format += "6d"; }
        else if( v[i] == "ray/first" ) { format += "3d"; }
        else if( v[i] == "ray/second" ) { format += "3d"; }
        else { format += "d"; }
    }
    return format;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help" ) || options.exists( "-h" ) ) { usage(); }
        std::string fields = fields_( options.value< std::string >( "--fields", "" ) );
        comma::csv::format format = format_( options.value< std::string >( "--binary,-b", "" ), fields );
        if( options.exists( "--format" ) ) { std::cout << format.string(); exit( 0 ); }
        velodyne::db db( options.value< std::string >( "--db", "/usr/local/etc/db.xml" ) );
        bool outputInvalidpoints = options.exists( "--output-invalid-points" );
        boost::optional< std::size_t > from;
        boost::optional< std::size_t > to;
        if( options.exists( "--scans" ) )
        {
            std::string range = options.value< std::string >( "--scans" );
            std::vector< std::string > v = comma::split( range, ':' );
            if( v.size() != 2 ) { COMMA_THROW( comma::exception, "expected range in format <from>:<to>, got: \"" << range << "\"" ); }
            from = v[0] == "" ? 0 : boost::lexical_cast< std::size_t >( v[0] );
            if( v[1] != "" ) { to = boost::lexical_cast< std::size_t >( v[1] ); }
            if( from && to && *from > *to ) { COMMA_THROW( comma::exception, "expected <from> not greater than <to> in the range, got: \"" << range << "\"" ); }
        }
        comma::csv::options csv;
        csv.fields = fields;
        csv.full_xpath = true;
        if( options.exists( "--binary,-b" ) ) { csv.format( format ); }
        options.assert_mutually_exclusive( "--pcap,--thin,--udp-port,--proprietary,-q" );
        double min_range = options.value( "--min-range", 0.0 );
        if( options.exists( "--pcap" ) )
        {
            velodyne_stream< snark::pcap_reader > v( db, outputInvalidpoints, from, to );
            run( v, csv, min_range );
        }
        else if( options.exists( "--thin" ) )
        {
            velodyne_stream< snark::thin_reader > v( db, outputInvalidpoints, from, to );
            run( v, csv, min_range );
        }
        else if( options.exists( "--udp-port" ) )
        {
            velodyne_stream< snark::udp_reader > v( options.value< unsigned short >( "--udp-port" ), db, outputInvalidpoints, from, to );
            run( v, csv, min_range );
        }
        else if( options.exists( "--proprietary,-q" ) )
        {
            velodyne_stream< snark::proprietary_reader > v( db, outputInvalidpoints, from, to );
            run( v, csv, min_range );
        }
        else
        {
            velodyne_stream< snark::stdin_reader > v( db, outputInvalidpoints, from, to );
            run( v, csv, min_range );
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "velodyne-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "velodyne-to-csv: unknown exception" << std::endl; }
    usage();
}
