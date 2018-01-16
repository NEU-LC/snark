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


/// @author vsevolod vlaskine

#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "../../math/range_bearing_elevation.h"
#include "../../visiting/traits.h"

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "take x,y,z, output range,bearing,elevation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat xyz.csv | points-to-polar [<options>] > rbe.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    --append; append range,bearing,elevation to each input record" << std::endl;
    std::cerr << "              (default behaviour is in place substitution: x with range, y with bearing, z with elevation)" << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << "    fields: x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat xyz.csv | points-to-polar --fields=x,y,z > rbe.csv" << std::endl;
    std::cerr << "    cat xyz.bin | points-to-polar --fields=x,y --binary=3d > rb0.bin" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av );
        if( options.exists( "--help,-h" ) ) { usage(); }
        comma::csv::options input_options( ac, av );
        comma::csv::options output_options( input_options );
        if( input_options.fields == "" ) { input_options.fields = "x,y,z"; }
        bool append = options.exists("--append");
        if ( append )
        {
            output_options.fields=comma::join( comma::csv::names< snark::range_bearing_elevation >(), ',');
            if ( input_options.binary() ) { output_options.format(comma::csv::format::value<snark::range_bearing_elevation>());}
        } 
        else
        {
            std::vector< std::string > fields = comma::split( input_options.fields, input_options.delimiter );
            std::vector< std::string > output_fields = fields;
            bool fields_set = false;
            for( std::size_t i = 0; i < fields.size(); ++i )
            {
                if( fields[i] == "x" ) { output_fields[i] = "range"; fields_set = true; }
                else if( fields[i] == "y" ) { output_fields[i] = "bearing"; fields_set = true; }
                else if( fields[i] == "z" ) { output_fields[i] = "elevation"; fields_set = true; }
            }
            if( !fields_set ) { std::cerr << "points-to-polar: expected some of the fields: " << comma::join( comma::csv::names< Eigen::Vector3d >(), ',' ) << ", got none in: " << input_options.fields << std::endl; return 1; }
            input_options.fields = comma::join( fields, ',' );
            output_options.fields = comma::join( output_fields, ',' );
        }
        comma::csv::input_stream< Eigen::Vector3d > is( std::cin, input_options );
        comma::csv::output_stream< snark::range_bearing_elevation > os( std::cout, output_options );
        comma::csv::tied < Eigen::Vector3d, snark::range_bearing_elevation > tied(is, os);
        comma::signal_flag is_shutdown;
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            const Eigen::Vector3d* p = is.read();
            if( p == NULL ) { return 0; }
            snark::range_bearing_elevation rbe;
            rbe.from_cartesian( *p );
            if( append ) { tied.append(rbe); }
            else { os.write( rbe, is.last() ); }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-polar: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-polar: unknown exception" << std::endl; }
    return 1;
}
