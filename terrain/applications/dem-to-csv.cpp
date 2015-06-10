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

#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include "../../math/spherical_geometry/traits.h"
#include "../../geodesy/geoids.h"
#include "../dem/srtm/data.h"
#include "../dem/srtm/header.h"
#include "../dem/srtm/load.h"
#include "../dem/srtm/traits.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "convert a dem file to csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat W060N40.DEM | dem-to-csv [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --format=[<format>]: default: srtm (we know, it's not the format name)" << std::endl;
    std::cerr << "    --header=<filename>: header file" << std::endl;
    std::cerr << "    --output-header: output header as path-value pairs and exit" << std::endl;
    std::cerr << "    --output-all,--all: output also points no data" << std::endl;
    std::cerr << "    --output-fields: print output fields to stdout and exit" << std::endl;
    std::cerr << "    --scale=<factor>: scale height, mostly for easy visualisation" << std::endl;
    std::cerr << std::endl;
    std::cerr << "formats" << std::endl;
    std::cerr << "    srtm (we know, it's not the format name): only basics currently supported" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    todo" << std::endl;
    std::cerr << std::endl;
}

struct position
{
    snark::spherical::coordinates coordinates;
    double height;
    
    position() : height( 0 ) {}
    position( const snark::spherical::coordinates& coordinates, double height ) : coordinates( coordinates ), height( height ) {}
};

namespace comma { namespace visiting {

template <> struct traits< position >
{
    template< typename K, typename V > static void visit( const K&, position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "height", t.height );
    }
    
    template< typename K, typename V > static void visit( const K&, const position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "height", t.height );
    }
};

} } // namespace comma { namespace visiting {
    
int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( options.exists( "--output-fields" ) ) { std::cout << comma::join( comma::csv::names< position >( false ), ',' ) << std::endl; }
        std::string format = options.value< std::string >( "--format", "srtm" );
        bool output_all = options.exists( "--output-all,--all" );
        bool add_earth_radius = options.exists( "--add-earth-radius,-e" );
        double scale = options.value< double >( "--scale", 1.0 );
        if( format == "srtm" )
        {
            snark::terrain::dem::srtm::header h;
            std::string f = options.value< std::string >( "--header" );
            load( h, f );
            if( options.exists( "--output-header" ) ) { comma::write_path_value( h, std::cout ); std::cout << std::endl; return 0; }
            snark::terrain::dem::tile::properties properties = h.tile_properties();
            #ifdef WIN32
            _setmode( _fileno( stdin ), _O_BINARY );
            #endif
            std::vector< char > buffer( properties.cols * snark::terrain::dem::srtm::data::size );
            comma::csv::output_stream< position > ostream( std::cout, comma::csv::options( options ) );
            for( std::size_t i = 0; i < properties.rows; ++i )
            {
                std::cin.read( &buffer[0], buffer.size() );
                if( std::cin.gcount() != int( buffer.size() ) ) { std::cerr << "dem-to-csv: expected " << buffer.size() << " for a row; got only: " << std::cin.gcount() << std::endl; return 1; }
                for( std::size_t j = 0; j < properties.cols; ++j )
                {
                    int h = reinterpret_cast< snark::terrain::dem::srtm::data* >( &buffer[ j * snark::terrain::dem::srtm::data::size ] )->height();
                    if( h == properties.nodata )
                    { 
                        if( !output_all ) { continue; }
                        //h = 0;
                    }
                    double height = h;
                    if( add_earth_radius ) { height += snark::geodesy::earth_average_radius; }
                    height *= scale;
                    snark::spherical::coordinates offset( -properties.resolution.latitude * i, properties.resolution.longitude * j );
                    ostream.write( position( properties.reference + offset, height ) );
                }
            }
            return 0;
        }
        std::cerr << "dem-to-csv: expected format, got \"" << format << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "dem-to-csv: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "dem-to-csv: unknown exception" << std::endl; }
    return 1;
}
