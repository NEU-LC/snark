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

#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <boost/program_options.hpp>
#include <comma/base/exception.h>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
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

static comma::int32 id_( const index_type& i, const index_type& end ) { return ( i[0] * end[1] + i[1] ) * end[2] + i[2]; }

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
            ( "resolution", boost::program_options::value< std::string >( &resolution_string ), "voxel map resolution, e.g. \"0.2\" or \"0.2,0.2,0.5\"" )
            ( "origin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "voxel map origin" )
            ( "extents", boost::program_options::value< std::string >( &extents_string ), "voxel map extents, needed only if --number is present" )
            ( "enumerate", "append voxel id in a grid with given origin and extents; note that only voxels inside of extents are guaranteed to be enumerated correctly" );
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
            return 1;
        }
        if( vm.count( "resolution" ) == 0 ) { std::cerr << "points-to-voxel-indices: please specify --resolution" << std::endl; return 1; }
        bool output_number = vm.count( "enumerate" );
        if( output_number && !vm.count( "extents" ) ) { std::cerr << "points-to-voxel-indices: if using --enumerate, please specify --extents" << std::endl; return 1; }
        comma::csv::options csv = comma::csv::program_options::get( vm );
        Eigen::Vector3d origin;
        Eigen::Vector3d resolution;
        index_type end;
        comma::csv::ascii< Eigen::Vector3d >().get( origin, origin_string );
        if( resolution_string.find_first_of( ',' ) == std::string::npos ) { resolution_string = resolution_string + ',' + resolution_string + ',' + resolution_string; }
        comma::csv::ascii< Eigen::Vector3d >().get( resolution, resolution_string );
        if( output_number )
        {
            Eigen::Vector3d extents;
            comma::csv::ascii< Eigen::Vector3d >().get( extents, extents_string );
            end = snark::voxel_map< input_point, 3 >::index_of( origin + extents, origin, resolution );
        }
        comma::csv::input_stream< input_point > istream( std::cin, csv );
        comma::signal_flag is_shutdown;
        if( csv.binary() )
        {
            #ifdef WIN32
            _setmode( _fileno( stdout ), _O_BINARY );
            #endif
            while( !is_shutdown && !std::cin.eof() && std::cin.good() )
            {
                const input_point* point = istream.read();
                if( !point ) { break; }
                index_type index = snark::voxel_map< input_point, 3 >::index_of( *point, origin, resolution );
                std::cout.write( istream.binary().last(), csv.format().size() );
                std::cout.write( reinterpret_cast< const char* >( &index[0] ), 3 * sizeof( comma::int32 ) );
                if( output_number )
                {
                    comma::int32 id = id_( index, end );
                    std::cout.write( reinterpret_cast< const char* >( &id ), sizeof( comma::int32 ) );
                }
                std::cout.flush();
            }
        }
        else
        {
            while( !is_shutdown && !std::cin.eof() && std::cin.good() )
            {
                const input_point* point = istream.read();
                if( !point ) { break; }
                index_type index = snark::voxel_map< input_point, 3 >::index_of( *point, origin, resolution );
                std::cout << comma::join( istream.ascii().last(), csv.delimiter )
                          << csv.delimiter << index[0]
                          << csv.delimiter << index[1]
                          << csv.delimiter << index[2];
                if( output_number ) { std::cout << csv.delimiter << id_( index, end ); }
                std::cout << std::endl;
            }
        }
        if( is_shutdown ) { std::cerr << "points-to-voxels: caught signal" << std::endl; return 1; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << argv[0] << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << argv[0] << ": unknown exception" << std::endl;
    }
    return 1;
}
