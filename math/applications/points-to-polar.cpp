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
// License along with snark. If not, see <http://www.gnu.org/licenses/>.auto

/// @author vsevolod vlaskine

#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include <snark/math/range_bearing_elevation.h>
#include <snark/visiting/traits.h>

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "take x,y,z, output range,bearing,elevation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat xyz.csv | points-to-polar [<options>] > rbe.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>ark::csv" << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << "    fields: r or range, b or bearing, e or elevation, default: r,b,e" << std::endl;
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
        std::vector< std::string > fields = comma::split( input_options.fields, input_options.delimiter );
        std::vector< std::string > output_fields = fields;
        for( std::size_t i = 0; i < fields.size(); ++i )
        {
            if( fields[i] == "x" ) { output_fields[i] = "range"; }
            else if( fields[i] == "y" ) { output_fields[i] = "bearing"; }
            else if( fields[i] == "z" ) { output_fields[i] = "elevation"; }
        }
        input_options.fields = comma::join( fields, ',' );
        output_options.fields = comma::join( output_fields, ',' );
        comma::csv::input_stream< Eigen::Vector3d > is( std::cin, input_options );
        comma::csv::output_stream< snark::range_bearing_elevation > os( std::cout, output_options );
        comma::signal_flag is_shutdown;
        while( !is_shutdown && std::cin.good() && !std::cin.eof() )
        {
            const Eigen::Vector3d* p = is.read();
            if( p == NULL ) { return 0; }
            snark::range_bearing_elevation rbe;
            rbe.from_cartesian( *p );
            if( input_options.binary() ) { os.write( rbe, is.binary().last() ); }
            else { os.write( rbe, is.ascii().last() ); }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << "points-to-polar: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-polar: unknown exception" << std::endl; }
    return 1;
}
