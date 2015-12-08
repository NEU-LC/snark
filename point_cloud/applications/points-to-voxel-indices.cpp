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

#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <comma/base/exception.h>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/impl/program_options.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include <snark/point_cloud/voxel_map.h>
#include <snark/visiting/eigen.h>

typedef Eigen::Vector3d input_point;
typedef snark::voxel_map< input_point, 3 >::index_type index_type;

static comma::int32 id_( const index_type& i, const index_type& size ) { return ( i[2] * size[1] + i[1] ) * size[0] + i[0]; }

static bool permissive;

static bool is_inside_( const index_type& i, const index_type& size )
{
    for( unsigned int k = 0; k < i.size(); ++k ) { if( i[k] < 0 || i[k] >= size[k] ) { return false; } }
    return true;
}

static void validate_id_( const index_type& i, const index_type& size, const input_point& p )
{
    if( permissive || is_inside_( i, size ) ) { return; }
    COMMA_THROW( comma::exception, "on point: " << p.x() << "," << p.y() << "," << p.z() << ": expected positive index less than " << size[0] << "," << size[1] << "," << size[2] << "; got: " << i[0] << "," << i[1] << "," << i[2] );
}

int main( int argc, char** argv )
{
    try
    {
        std::string binary;
        std::string origin_string;
        std::string extents_string;
        std::string resolution_string;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "begin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "an alias for --origin; voxel map origin" )
            ( "discard", "discard points outside of the given voxel map extents" )
            ( "end", boost::program_options::value< std::string >( &extents_string ), "can be used instead --extents, means origin + extents" )
            ( "enumerate", "append voxel id in a grid with given origin and extents; note that only voxels inside of the box defined by origin and extents are guaranteed to be enumerated correctly" )
            ( "extents", boost::program_options::value< std::string >( &extents_string ), "voxel map extents, i.e. its bounding box counting from origin, e.g. 10,10,10; needed only if --enumerate is present" )
            ( "origin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "voxel map origin" )
            ( "permissive", "if --enumerate given, allows points outside of the given voxel map extents" )
            ( "resolution", boost::program_options::value< std::string >( &resolution_string ), "voxel map resolution, e.g. \"0.2\" or \"0.2,0.2,0.5\"" );
        description.add( comma::csv::program_options::description( "x,y,z" ) );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) )
        {
            std::cerr << "take 3d points, append voxel index to them" << std::endl;
            std::cerr << std::endl;
            std::cerr << "usage: cat points.csv | points-to-voxel-indices [options] > indexed_points.csv" << std::endl;
            std::cerr << std::endl;
            std::cerr << "input fields: default: x,y,z" << std::endl;
            std::cerr << "binary input format: default: 3d" << std::endl;
            std::cerr << std::endl;
            std::cerr << description << std::endl;
            std::cerr << std::endl;
            std::cerr << "examples (try them)" << std::endl;
            std::cerr << "    echo -e 0,0,0\\\\n1,1,1\\\\n1.1,1.1,1.1 | points-to-voxel-indices --enumerate --origin 0,0,0 --extents 10,10,10 --resolution 1" << std::endl;
            std::cerr << "    0,0,0,0,0,0,0" << std::endl;
            std::cerr << "    1,1,1,1,1,1,111" << std::endl;
            std::cerr << "    1.1,1.1,1.1,1,1,1,111" << std::endl;
            std::cerr << std::endl;
            std::cerr << "    todo: more examples" << std::endl;
            std::cerr << std::endl;
            return 1;
        }
        if( vm.count( "resolution" ) == 0 ) { std::cerr << "points-to-voxel-indices: please specify --resolution" << std::endl; return 1; }
        bool output_number = vm.count( "enumerate" );
        permissive = vm.count( "permissive" );
        bool discard = vm.count( "discard" );
        if( output_number && extents_string.empty() ) { std::cerr << "points-to-voxel-indices: if using --enumerate, please specify --extents" << std::endl; return 1; }
        comma::csv::options csv = comma::csv::program_options::get( vm );
        Eigen::Vector3d origin;
        Eigen::Vector3d resolution;
        index_type size;
        comma::csv::ascii< Eigen::Vector3d >().get( origin, origin_string );
        if( resolution_string.find_first_of( ',' ) == std::string::npos ) { resolution_string = resolution_string + ',' + resolution_string + ',' + resolution_string; }
        comma::csv::ascii< Eigen::Vector3d >().get( resolution, resolution_string );
        if( output_number )
        {
            Eigen::Vector3d extents;
            comma::csv::ascii< Eigen::Vector3d >().get( extents, extents_string );
            size = snark::voxel_map< input_point, 3 >::index_of( origin + extents, origin, resolution );
            for( unsigned int k = 0; k < size.size(); size[k] += 1, ++k ); // to position beyond the last voxel
        }
        comma::csv::input_stream< input_point > istream( std::cin, csv );
        if( csv.binary() )
        {
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            while( !std::cin.eof() && std::cin.good() )
            {
                const input_point* point = istream.read();
                if( !point ) { break; }
                index_type index = snark::voxel_map< input_point, 3 >::index_of( *point, origin, resolution );
                if( discard && !is_inside_( index, size ) ) { continue; }
                if( output_number ) { validate_id_( index, size, *point ); }
                std::cout.write( istream.binary().last(), csv.format().size() );
                std::cout.write( reinterpret_cast< const char* >( &index[0] ), 3 * sizeof( comma::int32 ) );
                if( output_number )
                {
                    comma::int32 id = id_( index, size );
                    std::cout.write( reinterpret_cast< const char* >( &id ), sizeof( comma::int32 ) );
                }
                if( csv.flush ) { std::cout.flush(); }
            }
        }
        else
        {
            while( !std::cin.eof() && std::cin.good() )
            {
                const input_point* point = istream.read();
                if( !point ) { break; }
                index_type index = snark::voxel_map< input_point, 3 >::index_of( *point, origin, resolution );
                if( discard && !is_inside_( index, size ) ) { continue; }
                if( output_number ) { validate_id_( index, size, *point ); }
                std::cout << comma::join( istream.ascii().last(), csv.delimiter )
                          << csv.delimiter << index[0]
                          << csv.delimiter << index[1]
                          << csv.delimiter << index[2];
                if( output_number ) { std::cout << csv.delimiter << id_( index, size ); }
                std::cout << std::endl;
            }
        }
        return 0;
    }
    catch( std::exception& ex ) { std::cerr << argv[0] << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << argv[0] << ": unknown exception" << std::endl; }
    return 1;
}
