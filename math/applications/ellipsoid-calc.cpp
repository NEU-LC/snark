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

#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include "../spherical_geometry/ellipsoid.h"
#include "../spherical_geometry/traits.h"


static void usage( bool more = false )
{
    std::cerr << std::endl;
    std::cerr << "take coordinates in degrees on ellipsoid from stdin, perform calculations, append result and output to stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage examples" << std::endl;
    std::cerr << "    cat arcs.csv | ellipsoid-calc distance [<options>] > results.csv" << std::endl;
    std::cerr << "    echo 0,0,45,90 | ellipsoid-calc distance --major=6378137 --minor=6356752.314245" << std::endl;
    std::cerr << "    echo 0,0,45,90 | ellipsoid-calc distance --major=6378137 --minor=6356752.314245" << std::endl;
    std::cerr << "    echo 0,0,45,90 | ellipsoid-calc distance --major=1.00336409 --minor=1" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations" << std::endl;
    std::cerr << "    distance   : output length of ellipsoid arc" << std::endl;
    std::cerr << std::endl;
    std::cerr << "operations: details" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    distance: ellipsoid arc distance; if --binary, output as double" << std::endl;
    std::cerr << "        input fields: " << comma::join( comma::csv::names< snark::spherical::ellipsoid::arc >( true ), ',' ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: show help; --help --verbose for more help" << std::endl;
    std::cerr << "    --verbose,-v: more info" << std::endl;
    std::cerr << "    --major: ellipsoid's major semiaxis; default: 1.0" << std::endl;
    std::cerr << "    --minor: ellipsoid's minor semiaxis; default: 1.0" << std::endl;
    if ( more )
        std::cerr << std::endl << "csv options" << std::endl << comma::csv::options::usage() << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

int main( int ac, char **av )
{
    try
    {
        comma::command_line_options options( ac, av );
        bool verbose = options.exists( "--verbose,-v" );
        if ( options.exists( "--help,-h" ) )
            usage( verbose );
        comma::csv::options csv( options );
        if ( !csv.binary() )
            std::cout.precision( options.value( "--precision,-p", 12 ) );
        csv.full_xpath = true;
        const std::vector<std::string> &operations = options.unnamed( "--verbose,-v,--degrees", "-.*" );
        if ( operations.size() != 1 )
        {
            std::cerr << "ellipsoid-calc: expected one operation, got " << operations.size() << std::endl;
            return 1;
        }
        double major_semiaxis = options.value( "--major", 1.0 );
        double minor_semiaxis = options.value( "--minor", 1.0 );
        const std::string &operation = operations[0];
        if ( operation == "distance" )
        {
            snark::spherical::ellipsoid ellipsoid( major_semiaxis, minor_semiaxis );
            comma::csv::input_stream< snark::spherical::ellipsoid::arc > istream( std::cin, csv );
            while ( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
            {
                const snark::spherical::ellipsoid::arc *a = istream.read();
                if ( !a )
                    break;
                double distance = ellipsoid.distance( a->begin, a->end );
                if ( csv.binary() )
                {
                    std::cout.write( istream.binary().last(), istream.binary().binary().format().size() );
                    std::cout.write( reinterpret_cast< const char * >( &distance ), sizeof( double ) );
                }
                else
                    std::cout << comma::join( istream.ascii().last(), csv.delimiter ) << csv.delimiter << distance << std::endl;
            }
            return 0;
        }
        std::cerr << "ellipsoid-calc: unknown operation: \"" << operation << "\"" << std::endl;
        return 1;
    }
    catch ( std::exception &ex )
    {
        std::cerr << "ellipsoid-calc: " << ex.what() << std::endl;
    }
    catch ( ... )
    {
        std::cerr << "ellipsoid-calc: unknown exception" << std::endl;
    }
    return 1;
}
