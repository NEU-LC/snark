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
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

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

int main( int argc, char** argv )
{
    try
    {
        std::string binary;        
        std::string origin_string;
        std::string resolution_string;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "resolution", boost::program_options::value< std::string >( &resolution_string ), "voxel map resolution, e.g. \"0.2\" or \"0.2,0.2,0.5\"" )
            ( "origin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "voxel map origin" );
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
        if( vm.count( "resolution" ) == 0 ) { COMMA_THROW( comma::exception, "please specify --resolution" ); }        
        comma::csv::options csv = comma::csv::program_options::get( vm );
        Eigen::Vector3d origin;
        Eigen::Vector3d resolution;
        comma::csv::ascii< Eigen::Vector3d >().get( origin, origin_string );
        if( resolution_string.find_first_of( ',' ) == std::string::npos ) { resolution_string = resolution_string + ',' + resolution_string + ',' + resolution_string; }
        comma::csv::ascii< Eigen::Vector3d >().get( resolution, resolution_string );
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
                          << csv.delimiter << index[2]
                          << std::endl;
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
