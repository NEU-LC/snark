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

/// @authors abdallah kassir, vsevolod vlaskine

#include <iostream>
#include <comma/application/command_line_options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/io/stream.h>
#include <comma/name_value/parser.h>
#include "../../math/roll_pitch_yaw.h"
#include "../../math/rotation_matrix.h"
#include "../../math/geometry/polygon.h"
#include "../../math/geometry/polytope.h"
#include "../../math/geometry/traits.h"
#include "../../math/applications/frame.h"
#include "../../visiting/traits.h"

struct position
{
    Eigen::Vector3d coordinates;
    snark::roll_pitch_yaw orientation;
    
    position() : coordinates( Eigen::Vector3d::Zero() ) {}
};

struct normal
{
    Eigen::Vector3d coordinates;
    double distance;
    
    normal() : coordinates( Eigen::Vector3d::Zero() ) {}
};

struct input_t : public Eigen::Vector3d
{
    input_t() : Eigen::Vector3d( Eigen::Vector3d::Zero() ) {}
    ::position filter; // quick and dirty
};

struct output_t
{
    output_t( bool included = false ) : included( included ) {}
    bool included;
};

namespace comma { namespace visiting {
    
template <> struct traits< ::position >
{
    template < typename K, typename V > static void visit( const K& k, ::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }
    
    template < typename K, typename V > static void visit( const K& k, const ::position& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }
};

template <> struct traits< ::normal >
{
    template < typename K, typename V > static void visit( const K& k, ::normal& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "distance", t.distance );
    }
    
    template < typename K, typename V > static void visit( const K& k, const ::normal& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "distance", t.distance );
    }
};
    
template <> struct traits< input_t >
{
    template < typename K, typename V > static void visit( const K& k, input_t& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "filter", t.filter );
    }
    
    template < typename K, typename V > static void visit( const K& k, const input_t& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "filter", t.filter );
    }
};

template <> struct traits< output_t >
{ 
    template < typename K, typename V > static void visit( const K& k, const output_t& t, V& v )
    {
        v.apply( "included", t.included );
    }
};

} } // namespace comma { namespace visiting { 

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "filter points from dynamic objects represented by a position and orientation stream" << std::endl;
    std::cerr << "input: point-cloud, bounding stream" << std::endl;
    std::cerr << "       bounding data may either be joined to each point or provided through a separate stream in which points-grep will time-join the two streams" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-grep <what> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<what>" << std::endl;
    std::cerr << "     polytope,planes: arbitrary polytope that can be specified either as a set of planes or a set of plane normals and distances from 0,0,0" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --normals=<filename>[;<csv options>]: normals specifying a polytope" << std::endl;
    std::cerr << "             --normals-fields: output normals fields and exit" << std::endl;
    std::cerr << "             --normals-format: output normals format and exit" << std::endl;
    std::cerr << "             --planes=<filename>[;<csv options>]: planes specifying a polytope" << std::endl;
    std::cerr << "             --planes-fields: output planes fields and exit" << std::endl;
    std::cerr << "             --planes-format: output planes format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     box: rectangular box" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --begin,--origin=<x>,<y>,<z>: lower left back corner of the box" << std::endl;
    std::cerr << "             --end=<x>,<y>,<z>: upper right front corner of the box" << std::endl;
    std::cerr << "             --center,--centre=<x>,<y>,<z>: centre of the box; default: 0,0,0" << std::endl;
    std::cerr << "             --size=<x>,<y>,<z>: size of the box" << std::endl;
    std::cerr << "             any two of the options above are sufficient to specify a box" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit; default input fields: x,y,z" << std::endl;
    std::cerr << "    --input-format: print input format and exit" << std::endl;
    std::cerr << "    --output-all,--all: output all input points, append 1, if point is inside the shape, 0 otherwise" << std::endl;
    std::cerr << "                        for binary data, appended field will have format: " << comma::csv::format::value< output_t >() << std::endl;
    std::cerr << "    --output-fields: if --output-all given, print output fields and exit" << std::endl;
    std::cerr << "    --output-format: if --output-all given, print output format and exit" << std::endl;
    std::cerr << "    --position=<x>,<y>,<z>,<roll>,<pitch>,<yaw>: default filter shape position" << std::endl;
    std::cerr << std::endl;
    if( verbose) { std::cerr << comma::csv::options::usage() << std::endl << std::endl; }
    std::cerr << "examples" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    make a sample dataset" << std::endl;
    std::cerr << "        for i in $( seq -5 0.1 5 ) ; do for j in $( seq -5 0.1 5 ) ; do for k in $( seq -5 0.1 5 ) ; do echo $i,$j,$k ; done ; done ; done > cube.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter by a box; view result" << std::endl;
    std::cerr << "        cat cube.csv | points-grep box --size=1,2,3 --position=1,2,3,0.5,0.6,0.7 > filtered.csv" << std::endl;
    std::cerr << "        view-points \"cube.csv;colour=grey;hide\" \"filtered.csv;colour=red\" <( echo 0,0,0,0:0:0 ; echo 1,2,3,1:2:3 )\";colour=green;weight=10;fields=x,y,z,label\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter by two planes; view result" << std::endl;
    std::cerr << "        cat cube.csv | points-grep planes --normals <( echo 1,1,1,0.5 ; echo -1,1,1,0.5 ) > filtered.csv" << std::endl;
    std::cerr << "        view-points \"cube.csv;colour=grey;hide\" \"filtered.csv;colour=red\" <( echo 0,0,0,0:0:0 )\";colour=green;weight=10;fields=x,y,z,label\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    filter by a polytope; view result" << std::endl;
    std::cerr << "        cat cube.csv | points-grep planes --normals <( echo 0,0,-1,0 ; echo 0,-1,0,0 ; echo -1,0,0,0 ; echo 1,1,1,3 ) > filtered.csv" << std::endl;
    std::cerr << "        view-points \"cube.csv;colour=grey;hide\" \"filtered.csv;colour=red\" <( echo 0,0,0,0:0:0 )\";colour=green;weight=10;fields=x,y,z,label\"" << std::endl;
    std::cerr << std::endl;
    exit( 0 );
}

snark::geometry::convex_polytope transform( const snark::geometry::convex_polytope& polytope, const ::position& position ) { return polytope.transformed( position.coordinates, position.orientation ); } 

template < typename Shape > int run( const Shape& shape, const comma::command_line_options& options )
{
    comma::csv::options csv( options );
    csv.full_xpath = true;
    if( csv.fields.empty() ) { csv.fields = "x,y,z"; }
    std::vector< std::string > fields = comma::split( csv.fields, ',' );
    for( unsigned int i = 0; i < fields.size(); ++i )
    {
        if( fields[i] == "filter/x" ) { fields[i] = "filter/coordinates/x"; }
        else if( fields[i] == "filter/y" ) { fields[i] = "filter/coordinates/y"; }
        else if( fields[i] == "filter/z" ) { fields[i] = "filter/coordinates/z"; }
        else if( fields[i] == "filter/roll" ) { fields[i] = "filter/orientation/roll"; }
        else if( fields[i] == "filter/pitch" ) { fields[i] = "filter/orientation/pitch"; }
        else if( fields[i] == "filter/yaw" ) { fields[i] = "filter/orientation/yaw"; }
    }
    csv.fields = comma::join( fields, ',' );
    bool output_all = options.exists( "--output-all,--all" );
    input_t default_input;
    default_input.filter = comma::csv::ascii< ::position >().get( options.value< std::string >( "--position", "0,0,0,0,0,0" ) );
    boost::optional< Shape > transformed;
    if( !csv.has_some_of_fields( "filter,filter/coordinates,filter/coordinates/x,filter/coordinates/y,filter/coordinates/z,filter/orientation,filter/orientation/roll,filter/orientation/pitch,filter/orientation/yaw" ) ) { transformed = transform( shape, default_input.filter ); }
    comma::csv::input_stream< input_t > istream( std::cin, csv, default_input );
    comma::csv::output_stream< output_t > ostream( std::cout, csv.binary() );
    comma::csv::passed< input_t > passed( istream, std::cout );
    comma::csv::tied< input_t, output_t > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const input_t* p = istream.read();
        if( !p ) { break; }
        bool keep = transformed ? transformed->has( *p ) : transform( shape, p->filter ).has( *p );
        if( output_all ) { tied.append( output_t( keep ) ); }
        else if( keep ) { passed.write(); }
        if( csv.flush ) { std::cout.flush(); }
    }
    return 0;
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        bool output_all = options.exists( "--output-all,--all" );
        if( options.exists( "--input-fields" ) ) { std::cerr << comma::join( comma::csv::names< input_t >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--input-format" ) ) { std::cerr << comma::csv::format::value< input_t >() << std::endl; return 0; }
        if( options.exists( "--normals-fields" ) ) { std::cerr << comma::join( comma::csv::names< ::normal >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--normals-format" ) ) { std::cerr << comma::csv::format::value< ::normal >() << std::endl; return 0; }
        if( options.exists( "--planes-fields" ) ) { std::cerr << comma::join( comma::csv::names< snark::triangle >( true ), ',' ) << std::endl; return 0; }
        if( options.exists( "--planes-format" ) ) { std::cerr << comma::csv::format::value< snark::triangle >() << std::endl; return 0; }
        if( options.exists( "--output-fields" ) ) { if( output_all ) { std::cerr << comma::join( comma::csv::names< output_t >( true ), ',' ) << std::endl; } return 0; }
        if( options.exists( "--output-format" ) ) { if( output_all ) { std::cerr << comma::csv::format::value< output_t >() << std::endl; } return 0; }
        const std::vector< std::string >& unnamed = options.unnamed("--output-all,--all,--verbose,-v,--flush","-.*");
        std::string what = unnamed[0];
        Eigen::Vector3d inflate_by = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--offset,--inflate-by", "0,0,0" ) );
        if( what == "polytope" || what == "planes" )
        {
            Eigen::MatrixXd normals;
            Eigen::VectorXd distances;
            const std::string& normals_input = options.value< std::string >( "--normals", "" );
            const std::string& planes_input = options.value< std::string >( "--planes", "" );
            if( !normals_input.empty() )
            {
                comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( normals_input );
                comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
                comma::csv::input_stream< ::normal > istream( *is, filter_csv );
                std::vector< ::normal > n;
                while( istream.ready() || ( is->good() && !is->eof() ) )
                {
                    const ::normal* p = istream.read();
                    if( !p ) { break; }
                    n.push_back( *p );
                }
                normals.resize( n.size(), 3 );
                distances.resize( n.size() );
                for( unsigned int i = 0; i < n.size(); ++i )
                {
                    normals.row( i ) = n[i].coordinates.normalized();
                    distances[i] = n[i].distance;
                }
            }
            else if( !planes_input.empty() )
            {
                comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( planes_input );
                comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
                comma::csv::input_stream< snark::triangle > istream( *is, filter_csv );
                std::vector< snark::triangle > planes;
                while( istream.ready() || ( is->good() && !is->eof() ) )
                {
                    const snark::triangle* p = istream.read();
                    if( !p ) { break; }
                    planes.push_back( *p );
                }
                normals.resize( planes.size(), 3 );
                distances.resize( planes.size() );
                for( unsigned int i = 0; i < planes.size(); ++i )
                {
                    const Eigen::Vector3d& normal = ( planes[i].corners[1] - planes[i].corners[0] ).cross( planes[i].corners[2] - planes[i].corners[0] ).normalized();
                    normals.row( i ) = normal;
                    distances[i] = normal.dot( planes[i].corners[0] );
                }
            }
            else
            {
                std::cerr << "points-grep: polytope: please specify --planes or --normals" << std::endl;
                return 1;
            }
            return run( snark::geometry::convex_polytope( normals, distances ), options );
        }
        else if( what == "box" )
        {
            boost::optional< Eigen::Vector3d > origin;
            boost::optional< Eigen::Vector3d > end;
            boost::optional< Eigen::Vector3d > size;
            Eigen::Vector3d centre = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--center,--centre", "0,0,0" ) );
            if( options.exists( "--origin,--begin" ) ) { origin = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--origin,--begin" ) ); }
            if( options.exists( "--end" ) ) { end = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--end" ) ); }
            if( options.exists( "--size" ) ) { size = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--size" ) ); }
            if( !origin )
            {
                if( end && size ) { origin = *end - *size; }
                else if( size ) { origin = centre - *size / 2; }
                else { std::cerr << "points-grep: box: please specify --origin or --size" << std::endl; return 1; }
            }
            if( !end )
            {
                if( origin && size ) { end = *origin + *size; }
                else if( size ) { end = centre + *size / 2; }
                else { std::cerr << "points-grep: box: please specify --end or --size" << std::endl; return 1; }
            }
            centre = ( *origin + *end ) / 2 ;
            Eigen::Vector3d radius = ( *end - *origin ) / 2 + inflate_by;
            Eigen::MatrixXd normals( 6, 3 );
            normals <<  0,  0,  1,
                        0,  0, -1,
                        0,  1,  0,
                        0, -1,  0,
                        1,  0,  0,
                       -1,  0,  0;
            Eigen::VectorXd distances( 6 );
            distances << radius.x(), radius.x(), radius.y(), radius.y(), radius.z(), radius.z();
            return run( snark::geometry::convex_polytope( normals, distances ), options );
        }
        std::cerr << "points-grep: expected filter name, got: \"" << what << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "points-grep: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-grep: unknown exception" << std::endl; }
    return 1;
}
