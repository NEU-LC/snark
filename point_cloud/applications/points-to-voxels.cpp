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
#include <comma/application/signal_flag.h>
#include <comma/base/types.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/impl/program_options.h>
#include <comma/visiting/traits.h>
#include "../../visiting/eigen.h"
#include "../voxel_map.h"

struct input_point
{
    Eigen::Vector3d point;
    comma::uint32 block;
    
    input_point() : block( 0 ) {}
};

struct centroid
{
    boost::array< comma::int32, 3 > index;
    Eigen::Vector3d mean;
    comma::uint32 size;
    comma::uint32 block;
    
    centroid() : size( 0 ), block( 0 ) {}
    
    void operator+=( const Eigen::Vector3d& point )
    {
        ++size;
        mean = ( mean * ( size - 1 ) + point ) / size;
    }
};

namespace comma { namespace visiting {

template <> struct traits< input_point >
{
    template < typename K, typename V > static void visit( const K&, input_point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const input_point& p, V& v )
    {
        v.apply( "point", p.point );
        v.apply( "block", p.block );
    }
};

template <> struct traits< centroid >
{
    template < typename K, typename V > static void visit( const K&, centroid& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "mean", p.mean );
        v.apply( "size", p.size );
        v.apply( "block", p.block );
    }

    template < typename K, typename V > static void visit( const K&, const centroid& p, V& v )
    {
        v.apply( "index", p.index );
        v.apply( "mean", p.mean );
        v.apply( "size", p.size );
        v.apply( "block", p.block );
    }
};

} } // namespace comma { namespace visiting {

template < typename T, std::size_t Size >
std::ostream& operator<<( std::ostream& os, const boost::array< T, Size >& a )
{
    for( std::size_t i = 0; i < Size; ++i )
    {
        os << a[i] << " ";
    }
    return os;
}
    
int main( int argc, char** argv )
{
    try
    {
        std::string origin_string;
        std::string resolution_string;
        boost::program_options::options_description description( "options" );
        comma::uint32 neighbourhood_radius;
        description.add_options()
            ( "help,h", "display help message" )
            ( "resolution", boost::program_options::value< std::string >( &resolution_string ), "voxel map resolution, e.g. \"0.2\" or \"0.2,0.2,0.5\"" )
            ( "origin", boost::program_options::value< std::string >( &origin_string )->default_value( "0,0,0" ), "voxel map origin" )
            ( "neighbourhood-radius,r", boost::program_options::value< comma::uint32 >( &neighbourhood_radius )->default_value( 0 ), "calculate count of neighbours at given radius" );
        description.add( comma::csv::program_options::description( "x,y,z,block" ) );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) )
        {
            std::cerr << "downsample a point cloud using a voxel map" << std::endl;
            std::cerr << std::endl;
            std::cerr << "usage: cat points.csv | points-to-voxels [options] > voxels.csv" << std::endl;
            std::cerr << std::endl;
            std::cerr << "input: points: x,y,z[,block]; default: x,y,z,block" << std::endl;
            std::cerr << "output: voxels with indices, centroids, and weights (number of points): i,j,k,x,y,z,weight[,neighbour count][,block]" << std::endl;
            std::cerr << "binary output format: 3ui,3d,ui[,ui][,ui]" << std::endl;
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
        comma::csv::options output_csv = csv;
        output_csv.full_xpath = true;
        if( csv.has_field( "block" ) ) // todo: quick and dirty, make output fields configurable?
        {
            output_csv.fields = "index,mean,size,block";
            if( csv.binary() ) { output_csv.format( "3ui,3d,ui,ui" ); }
        }
        else
        {
            output_csv.fields = "index,mean,size";
            if( csv.binary() ) { output_csv.format( "3ui,3d,ui" ); }
        }
        comma::csv::output_stream< centroid > ostream( std::cout, output_csv );
        comma::signal_flag is_shutdown;
        unsigned int block = 0;
        const input_point* last = NULL;
        while( !is_shutdown && !std::cin.eof() && std::cin.good() )
        {
            snark::voxel_map< centroid, 3 > voxels( origin, resolution );
            if( last ) { voxels.touch_at( last->point )->second += last->point; }
            while( !is_shutdown && !std::cin.eof() && std::cin.good() )
            {
                last = istream.read();
                if( !last || last->block != block ) { break; }
                voxels.touch_at( last->point )->second += last->point;
            }
            if( is_shutdown ) { break; }
//             for( snark::voxel_map< centroid, 3 >::iterator it = voxels.begin(); it != voxels.end(); ++it )
//             {
//                 it->second.block = block;
//                 it->second.index = snark::voxel_map< centroid, 3 >::index_of( it->second.mean, origin, resolution );
//                 if( neighbourhood_radius > 0 ) // quick and dirty
//                 {
//                     snark::voxel_map< centroid, 3 >::index_type index;
//                     snark::voxel_map< centroid, 3 >::index_type begin = {{ it->first[0] - neighbourhood_radius, it->first[1] - neighbourhood_radius, it->first[2] - neighbourhood_radius }};
//                     snark::voxel_map< centroid, 3 >::index_type end = {{ it->first[0] + neighbourhood_radius + 1, it->first[1] + neighbourhood_radius + 1, it->first[2] + neighbourhood_radius + 1 }};
//                     std::size_t size = 0;
//                     Eigen::Vector3d mean( 0, 0, 0 );
//                     for( index[0] = begin[0]; index[0] < end[0]; ++index[0] )                        
//                     {
//                         for( index[1] = begin[1]; index[1] < end[1]; ++index[1] )
//                         {
//                             for( index[2] = begin[2]; index[2] < end[2]; ++index[2] )
//                             {
//                                 snark::voxel_map< centroid, 3 >::const_iterator nit = voxels.find( index );
//                                 if( nit == voxels.end() ) { continue; }
//                                 ostream.write( it->second ); // todo: remove!!
//                                 size += nit->second.size;
//                                 //mean += ( nit->second.mean * nit->second.size );
//                             }
//                         }
//                     }
//                     //it->second.size = size;
//                     
//                     //mean /= size;
//                     //it->second.mean = mean;
//                 }
//                 ostream.write( it->second );
//             }

            for( snark::voxel_map< centroid, 3 >::iterator it = voxels.begin(); it != voxels.end(); ++it )
            {
                it->second.block = block;
                it->second.index = snark::voxel_map< centroid, 3 >::index_of( it->second.mean, origin, resolution );
                if( neighbourhood_radius == 0 )
                {
                    ostream.write( it->second );
                }
                else
                {
                    centroid c = it->second;
                    snark::voxel_map< centroid, 3 >::index_type index;
                    snark::voxel_map< centroid, 3 >::index_type begin = {{ it->first[0] - neighbourhood_radius, it->first[1] - neighbourhood_radius, it->first[2] - neighbourhood_radius }};
                    snark::voxel_map< centroid, 3 >::index_type end = {{ it->first[0] + neighbourhood_radius + 1, it->first[1] + neighbourhood_radius + 1, it->first[2] + neighbourhood_radius + 1 }};
                    for( index[0] = begin[0]; index[0] < end[0]; ++index[0] )                        
                    {
                        for( index[1] = begin[1]; index[1] < end[1]; ++index[1] )
                        {
                            for( index[2] = begin[2]; index[2] < end[2]; ++index[2] )
                            {
                                snark::voxel_map< centroid, 3 >::const_iterator nit = voxels.find( index );
                                if( nit == voxels.end() ) { continue; }
                                c.size += nit->second.size;
                                c.mean += ( nit->second.mean * nit->second.size );
                            }
                        }
                    }
                    c.mean /= c.size;
                    ostream.write( c );
                }
            }
            if( !last ) { break; }
            block = last->block;
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
