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

template< typename IStream, typename OStream >
int run( IStream& is, OStream& os )
{
    comma::signal_flag is_shutdown;
    while( !is_shutdown )
    {
        const snark::range_bearing_elevation* p = is.read();
        if( p == NULL ) { return 0; }
        os.write( p->to_cartesian(), is.last() );
    }
    return 0;
}

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "take range, bearing, elevation as input, output x,y,z" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat rbe.csv | points-to-cartesian [<options>] > xyz.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << comma::csv::options::usage() << std::endl;
    std::cerr << "    fields: r or range, b or bearing, e or elevation, default: r,b,e" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    cat rbe.csv | points-to-cartesian --fields=r,b,e > xyz.csv" << std::endl;
    std::cerr << "    cat rbe.bin | points-to-cartesian --fields=r,b --binary=3d > xy0.bin" << std::endl;
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
        if( input_options.fields == "" ) { input_options.fields = "r,b,e"; }
        std::vector< std::string > fields = comma::split( input_options.fields, input_options.delimiter );
        std::vector< std::string > output_fields = fields;
        for( std::size_t i = 0; i < fields.size(); ++i )
        {
            if( fields[i] == "r" ) { fields[i] = "range"; }
            else if( fields[i] == "b" ) { fields[i] = "bearing"; }
            else if( fields[i] == "e" ) { fields[i] = "elevation"; }
            if( fields[i] == "range" ) { output_fields[i] = "x"; }
            else if( fields[i] == "bearing" ) { output_fields[i] = "y"; }
            else if( fields[i] == "elevation" ) { output_fields[i] = "z"; }
        }
        input_options.fields = comma::join( fields, ',' );
        output_options.fields = comma::join( output_fields, ',' );
        if( input_options.binary() )
        {
            comma::csv::binary_input_stream< snark::range_bearing_elevation > is( std::cin, input_options );
            comma::csv::binary_output_stream< Eigen::Vector3d > os( std::cout, output_options );
            return run( is, os );
        }
        else
        {
            comma::csv::ascii_input_stream< snark::range_bearing_elevation > is( std::cin, input_options );
            comma::csv::ascii_output_stream< Eigen::Vector3d > os( std::cout, output_options );
            return run( is, os );
        }
    }
    catch( std::exception& ex ) { std::cerr << "points-to-cartesian: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-to-cartesian: unknown exception" << std::endl; }
    return 1;
}
