// This file is part of comma, a generic and flexible library
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

#include <Eigen/Core>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/name_value/serialize.h>
#include <comma/visiting/traits.h>
#include "../../../sensors/las/packets.h"
#include "../../../sensors/las/traits.h"
#include "../../../visiting/eigen.h"

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "convert las (laser exchange format) to csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "cat points.las | las-to-csv <what> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "what" << std::endl;
    std::cerr << "    points: output points" << std::endl;
    std::cerr << "    header: output header as json and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help; --help --verbose: more help" << std::endl;
    std::cerr << "    --output-fields=<point format>: output fields for a given point format (0-5) and exit" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl; }
    std::cerr << std::endl;
    exit( 0 );
}

template < unsigned int I > struct point;

template <> struct point< 1 >
{
    Eigen::Vector3d coordinates;
    comma::uint16 intensity;
    unsigned char return_number;
    unsigned char number_of_returns;
    bool scan_direction;
    bool edge_of_flight_line;
    unsigned char classification;
    char scan_angle;
    unsigned char user_data;
    comma::uint16 point_source_id;
    double gps_time;
    
    point() {}
    point( const snark::las::point< 1 >& p, const Eigen::Vector3d& factor, const Eigen::Vector3d& offset )
        : coordinates( Eigen::Vector3d( factor.x() * p.coordinates.x()
                                      , factor.y() * p.coordinates.y()
                                      , factor.z() * p.coordinates.z() ) + offset )
        , intensity( p.intensity() )
        , return_number( p.returns().number )
        , number_of_returns( p.returns().size )
        , scan_direction( p.returns().scan_direction )
        , edge_of_flight_line( p.returns().edge_of_flight_line )
        , classification( p.classification() )
        , scan_angle( p.scan_angle() )
        , user_data( p.user_data() )
        , point_source_id( p.point_source_id() )
        , gps_time( p.gps_time() )
    {
    }
};

namespace comma { namespace visiting {

template <> struct traits< point< 1 > >
{
    template< typename K, typename V > static void visit( const K&, const point< 1 >& t, V& v ) // todo
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "intensity", t.intensity );
        v.apply( "return_number", t.return_number );
        v.apply( "number_of_returns", t.number_of_returns );
        v.apply( "scan_direction", t.scan_direction );
        v.apply( "edge_of_flight_line", t.edge_of_flight_line );
        v.apply( "classification", t.classification );
        v.apply( "scan_angle", t.scan_angle );
        v.apply( "user_data", t.user_data );
        v.apply( "point_source_id", t.point_source_id );
        v.apply( "gps_time", t.gps_time );
    }
};

} } // namespace comma { namespace visiting {

template < unsigned int I >
static int read_points( const snark::las::header& header, const comma::csv::options& csv )
{
    Eigen::Vector3d factor( header.scale_factor.x(), header.scale_factor.y(), header.scale_factor.z() );
    Eigen::Vector3d offset( header.offset.x(), header.offset.y(), header.offset.z() );
    comma::csv::output_stream< point< I > > os( std::cout, csv );
    while( std::cin.good() && !std::cin.eof() )
    {
        snark::las::point< I > p;
        std::cin.read( reinterpret_cast< char* >( &p ), snark::las::point< I >::size ); // todo: watch performance
        int count = std::cin.gcount();
        if( count == 0 ) { break; }
        if( count < snark::las::point< I >::size ) { std::cerr << "las-to-csv: expected las point record format " << I << " of " << snark::las::point< I >::size << " bytes, got only: " << count << std::endl; return 1; }
        os.write( point< I >( p, factor, offset ) );
    }
    return 0;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        if( ac < 2 ) { usage(); }
        std::string what = av[1];
        snark::las::header header;
        std::cin.read( reinterpret_cast< char* >( &header ), snark::las::header::size );
        int count = std::cin.gcount();
        if( count < snark::las::header::size ) { std::cerr << "las-to-csv: expected las header of " << snark::las::header::size << " bytes, got only: " << count << std::endl; return 1; }
        if( what == "header" ) { comma::write_json( header, std::cout ); return 0; }
        if( what == "points" )
        {
            std::vector< char > offset( header.offset_to_point_data() - header.header_size() );
            std::cin.read( &offset[0], offset.size() );
            count = std::cin.gcount();
            if( count < int( offset.size() ) ) { std::cerr << "las-to-csv: expected " << offset.size() << " bytes, got only: " << count << std::endl; return 1; }
            switch( header.point_data_format() )
            {
                case 1:
                    return read_points< 1 >( header, comma::csv::options( options ) );
                case 0:
                case 2:
                case 3:
                case 4:
                case 5:
                    std::cerr << "las-to-csv: point data format " << header.point_data_format() << ": todo" << std::endl;
                    return 1;
                default:
                    std::cerr << "las-to-csv: expected point data format between 0 and 5, got: " << header.point_data_format() << std::endl;
                    return 1;
            }
            return 0;
        }
        std::cerr << "las-to-csv: expected operation, got: \"" << what << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex )
    {
        std::cerr << "las-to-csv: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "las-to-csv: unknown exception" << std::endl;
    }
    return 1;
}
