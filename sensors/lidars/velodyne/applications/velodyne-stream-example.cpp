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


#include <fstream>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../../../visiting/eigen.h"
#include "../impl/pcap_reader.h"
#include "../hdl64/stream.h"

using namespace snark;

// output point
struct point
{
    boost::posix_time::ptime timestamp;
    comma::uint32 id;
    bool valid;
    unsigned char intensity;
    std::pair< ::Eigen::Vector3d, ::Eigen::Vector3d > ray;
    comma::uint32 scan;
};

namespace comma { namespace visiting {

template <> struct traits< point >
{
    template < typename Key, class Visitor >
    static void visit( Key k, const point& t, Visitor& v )
    {
        v.apply( "t", t.timestamp );
        v.apply( "id", t.id );
        v.apply( "valid", t.valid );
        v.apply( "intensity", t.intensity );
        v.apply( "ray", t.ray );
        v.apply( "scan", t.scan );
    }
};

} } // namespace comma { namespace visiting {

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "read pcap data from stdin, output velodyne points on stdout" << std::endl;
    std::cerr << "cat velodyne.pcap | velodyne-stream-example [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --binary : output as binary" << std::endl;
    std::cerr << "    --db <db.xml> : default /usr/local/etc/db.calibrated.xml" << std::endl;
    std::cerr << "    --format : return output format to stdout and exit" << std::endl;
    std::cerr << "    --output-invalid : if present, output invalid points" << std::endl;
//     std::cerr << "    --rpm <rotations per minute>: default 600" << std::endl;
    std::cerr << "    --fields : output only given fields (todo)" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

int main( int ac, char** av )
{
    try
    {
        // get command line options
        comma::command_line_options options( ac, av );
        comma::signal_flag isShutdown;
        if( options.exists( "-h,--help" ) ) { usage(); }
        if( options.exists( "--format" ) ) { std::cout << comma::csv::format::value< point >() << std::endl; return 0; }
        bool outputInvalid = options.exists( "--output-invalid" );
//         unsigned int rpm = options.value( "--rpm", 600 );
    
        // create output stream (in this example by default we output a velodyne point followed by scan id)
        comma::csv::options csv;
        csv.full_xpath = true;
        if( options.exists( "--binary" ) ) { csv.format( comma::csv::format::value< point >() ); }
        comma::csv::output_stream< point > ostream( std::cout, csv );
    
        // fill out velodyne db from db.xml file
        velodyne::hdl64::db db( options.value< std::string >( "--db", "/usr/local/etc/db.calibrated.xml" ) );
    
        // create velodyne point stream
        velodyne::hdl64::stream< snark::pcap_reader > stream( new snark::pcap_reader, outputInvalid );
    
        // read velodyne points, get the scan id, and output to stdout
        while( !isShutdown )
        {
            const velodyne::laser_return* r = stream.read();
            if( r == NULL ) { break; }
            point p;
            p.timestamp = r->timestamp;
            p.id = r->id;
            p.intensity = r->intensity;
            p.valid = true;
            p.ray = db.lasers[ p.id ].ray( r->range, r->azimuth );
            p.scan = stream.scan();
            ostream.write( p );
        }
    
        // shutdown
        if( isShutdown ) { std::cerr << "velodyne-stream-example: interrupted by signal" << std::endl; }
        else { std::cerr << "velodyne-stream-example: done, no more data" << std::endl; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "velodyne-stream-example: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "velodyne-stream-example: unknown exception" << std::endl;
    }
    return -1;
}
