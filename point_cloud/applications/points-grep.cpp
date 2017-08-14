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

#include <boost/geometry/geometries/polygon.hpp> // todo: take a look at multipolygon
#include <boost/geometry/algorithms/intersects.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/strategies/agnostic/point_in_poly_winding.hpp>
#include <boost/geometry/geometries/linestring.hpp>
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
#include "../../visiting/eigen.h"
#include "../../visiting/traits.h"

static void usage( bool verbose = false )
{
    std::cerr << std::endl;
    std::cerr << "filter points from dynamic objects represented by a position and orientation stream" << std::endl;
    std::cerr << "stdin: stream of points to filter" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat points.csv | points-grep <what> [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<what>" << std::endl;
    std::cerr << "     box: rectangular box" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --begin,--origin=<x>,<y>,<z>: lower left back corner of the box" << std::endl;
    std::cerr << "             --end=<x>,<y>,<z>: upper right front corner of the box" << std::endl;
    std::cerr << "             --center,--centre=<x>,<y>,<z>: centre of the box; default: 0,0,0" << std::endl;
    std::cerr << "             --inflate-by=<x>,<y>,<z>: add it to the box size, a convenience option" << std::endl;
    std::cerr << "             --size=<x>,<y>,<z>: size of the box" << std::endl;
    std::cerr << "             any two of the options above are sufficient to specify a box" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     polygons: take input 2d points or line segments, identify whether they are fully contained in a set of 2d polygons" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --fields" << std::endl;
    std::cerr << "                 x,y: input is points" << std::endl;
    std::cerr << "                 first,second,first/x,first/y,second/x,second/y: if any of these fields present, input is line segments" << std::endl;
    std::cerr << "                 default: x,y" << std::endl;
    std::cerr << "             --polygons=<filename>[;<csv options>]: polygon points specified in clockwise order" << std::endl;
    std::cerr << "                 default fields: x,y[,id], where id is the polygon id of this bounding corner" << std::endl;
    std::cerr << "                 polygons: defined by boundary points identified by id field, default id: 0, both clockwise and anti-clockwise direction accepted" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     polytope,planes: arbitrary polytope that can be specified either as a set of planes or a set of plane normals and distances from 0,0,0" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --inflate-by=<radius>: expand prism by the given value, a convenience option" << std::endl;
    std::cerr << "             --normals=<filename>[;<csv options>]: normals specifying a polytope" << std::endl;
    std::cerr << "             --normals-fields: output normals fields and exit" << std::endl;
    std::cerr << "             --normals-format: output normals format and exit" << std::endl;
    std::cerr << "             --normals-size,--number-of-planes,--number-of-normals: if normals fields given on stdin, specify the number of normals" << std::endl;
    std::cerr << "             --planes=<filename>[;<csv options>]: planes specifying a polytope" << std::endl;
    std::cerr << "             --planes-fields: output planes fields and exit" << std::endl;
    std::cerr << "             --planes-format: output planes format and exit" << std::endl;
    std::cerr << std::endl;
    std::cerr << "     prism" << std::endl;
    std::cerr << "         options" << std::endl;
    std::cerr << "             --axis=<x>,<y>,<z>,<length>: prism axis vector; default: 0,0,1,0" << std::endl;
    std::cerr << "             --corners=<filename>[;<csv options>]: corners of prism base" << std::endl;
    std::cerr << "             --corners-fields: output corners fields and exit" << std::endl;
    std::cerr << "             --corners-format: output corners format and exit" << std::endl;
    std::cerr << "             --corners-size,--number-of-corners: if corners fields given on stdin, specify the number of corners" << std::endl;
    std::cerr << "             --inflate-box-by=<radius>: expand prism by the given value, a convenience option" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit; default input fields: x,y,z" << std::endl;
    std::cerr << "    --input-format: print input format and exit" << std::endl;
    std::cerr << "    --output-all,--all: output all input points, append 1, if point is inside the shape, 0 otherwise" << std::endl;
    std::cerr << "    --output-fields: if --output-all given, print appended output fields and exit" << std::endl;
    std::cerr << "    --output-format: if --output-all given, print appended output format and exit" << std::endl;
    std::cerr << "    --point=<x>,<y>,<z>: default point coordinates; default: 0,0,0" << std::endl;
    std::cerr << "                         useful e.g. when we need to filter a fixed point by polytope position" << std::endl;
    std::cerr << "    --position=<x>,<y>,<z>,<roll>,<pitch>,<yaw>: default filter shape position" << std::endl;
    std::cerr << std::endl;
    if( verbose ) { std::cerr << comma::csv::options::usage() << std::endl << std::endl; }
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
    std::cerr << "    polygons" << std::endl;
    std::cerr << "      testing for points inside of polygon" << std::endl;
    std::cerr << "        ( echo 0,5; echo 5,5; echo 5,0; echo 0,5 ) | csv-paste - value=1 >polygons.csv" << std::endl;
    std::cerr << "        ( echo 5,10; echo 10,10; echo 10,5; echo 5,5; echo 5,10 ) | csv-paste - value=2 >>polygons.csv" << std::endl;
    std::cerr << "        ( for i in $( seq 0 0.2 10 ) ; do for j in $( seq 0 0.2 10 ) ; do echo $i,$j ; done ; done ) | points-grep polygons --polygons polygons.csv" << std::endl;
    std::cerr << std::endl;
    std::cerr << "      making a concave polygon" << std::endl;
    std::cerr << "        ( echo 0,5; echo 5,5; echo 0,0; echo 5,-5; echo 0,-5; echo -5,0 ) | csv-paste - value=1 >polygons.csv" << std::endl;
    std::cerr << "      testing of line totally contained in a concave polygon" << std::endl;
    std::cerr << "        ( echo -2,0,-1,2,inside; echo 1,2,1,-2,partial; echo 2,0,4,0,outside) | points-grep polygons --fields first,second --polygons polygons.csv | tee results.csv" << std::endl;
    std::cerr << "        visualisation:" << std::endl;
    std::cerr << "           view-points 'results.csv;fields=first/x,first/y,second/x,second/y,label,id;shape=line' 'polygons.csv;colour=grey;weight=10;fields=x,y;shape=loop'" << std::endl;
    exit( 0 );
}

static bool verbose;

namespace snark { namespace operations { namespace polygons {
    
struct polygon_point : public Eigen::Vector2d {
    comma::uint32 id;   // default is 0
};

} } } // namespace snark { namespace operations { namespace polygons {

struct flags_t {
    flags_t( comma::uint32 size ) : flags( size, false ) {}
    std::vector< bool > flags;
};

struct normal
{
    Eigen::Vector3d coordinates;
    double distance;
    
    normal( const Eigen::Vector3d& coordinates = Eigen::Vector3d::Zero(), double distance = 0 ) : coordinates( coordinates ), distance( distance ) {}
};

struct position
{
    Eigen::Vector3d coordinates;
    snark::roll_pitch_yaw orientation;
    
    position() : coordinates( Eigen::Vector3d::Zero() ) {}
};

namespace species {
    
struct polytope
{
    struct filter : public ::position // quick and dirty
    {
        std::vector< ::normal > normals;
        
        filter( const ::position& p = ::position() ) : ::position( p ) {}
    };
};

struct prism
{
    struct filter : public ::position // quick and dirty
    {
        ::normal axis;
        std::vector< Eigen::Vector3d > corners;
        
        filter( const ::position& p = ::position() ) : ::position( p ) {}
    };
};

struct box
{
    typedef ::position filter;
};

} // namespace species {

template < typename S > 
struct input_t : public Eigen::Vector3d // quick and dirty
{
    typename S::filter filter;
    
    input_t( const Eigen::Vector3d& p = Eigen::Vector3d::Zero(), const typename S::filter& filter = typename S::filter() ) : Eigen::Vector3d( p ), filter( filter ) {}
};

struct output_t
{
    bool included;
    
    output_t( bool included = false ) : included( included ) {}
};

namespace comma { namespace visiting {
    
template <> struct traits< snark::operations::polygons::polygon_point >
{
    template < typename K, typename V > static void visit( const K& k, snark::operations::polygons::polygon_point& t, V& v )
    {
        traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "id", t.id );
    }
    
    template < typename K, typename V > static void visit( const K& k, const snark::operations::polygons::polygon_point& t, V& v )
    {
        traits< Eigen::Vector2d >::visit( k, t, v );
        v.apply( "id", t.id );
    }
};

template <> struct traits< flags_t >
{
    template < typename K, typename V > static void visit( const K& k, flags_t& t, V& v ) { v.apply( "contains", t.flags ); }
    template < typename K, typename V > static void visit( const K& k, const flags_t& t, V& v ) { v.apply( "contains", t.flags ); }
};

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

template <> struct traits< species::polytope::filter >
{
    template < typename K, typename V > static void visit( const K& k, species::polytope::filter& t, V& v )
    {
        traits< ::position >::visit( k, t, v );
        v.apply( "normals", t.normals );
    }
    
    template < typename K, typename V > static void visit( const K& k, const species::polytope::filter& t, V& v )
    {
        traits< ::position >::visit( k, t, v );
        v.apply( "normals", t.normals );
    }
};

template <> struct traits< species::prism::filter >
{
    template < typename K, typename V > static void visit( const K& k, species::prism::filter& t, V& v )
    {
        traits< ::position >::visit( k, t, v );
        v.apply( "axis", t.axis );
        v.apply( "corners", t.corners );
    }
    
    template < typename K, typename V > static void visit( const K& k, const species::prism::filter& t, V& v )
    {
        traits< ::position >::visit( k, t, v );
        v.apply( "axis", t.axis );
        v.apply( "corners", t.corners );
    }
};

template < typename F > struct traits< input_t< F > >
{
    template < typename K, typename V > static void visit( const K& k, input_t< F >& t, V& v )
    {
        traits< Eigen::Vector3d >::visit( k, t, v );
        v.apply( "filter", t.filter );
    }
    
    template < typename K, typename V > static void visit( const K& k, const input_t< F >& t, V& v )
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

static snark::geometry::convex_polytope transform( const snark::geometry::convex_polytope& polytope, const ::position& position ) { return polytope.transformed( position.coordinates, position.orientation ); } 

template < typename Species, typename Genus = snark::geometry::convex_polytope > struct shape_traits;

template < typename Species > struct shape_traits< Species, snark::geometry::convex_polytope >
{
    static snark::geometry::convex_polytope make( const typename Species::filter& f, double inflate_by = 0 ) // todo: quick and dirty, watch performance
    {
        Eigen::MatrixXd normals( f.normals.size(), 3 );
        Eigen::VectorXd distances( f.normals.size() );
        for( unsigned int i = 0; i < f.normals.size(); ++i )
        {
            normals.row( i ) = f.normals[i].coordinates.normalized().transpose();
            distances( i ) = f.normals[i].distance + inflate_by;
        }
        return snark::geometry::convex_polytope( normals, distances );
    }
    
    static void set_default_filter( typename Species::filter& f, const comma::command_line_options& options, const comma::csv::options& csv )
    {
        bool has_normals = csv.has_some_of_fields( "filter,filter/normals" );
        if( options.exists( "--normals,--planes" ) )
        {
            if( !has_normals ) { return; }
            std::cerr << "points-grep: polytope: expected either --planes or --normals, or filter/normals in input fields, got both" << std::endl;
            exit( 1 );
        }
        if( !has_normals ) { std::cerr << "points-grep: polytope: please specify --planes, or --normals, or filter/normals in input fields" << std::endl; exit( 1 ); }
        f.normals.resize( options.value< unsigned int >( "--normals-size,--number-of-planes,--number-of-normals" ) );
    }
};

template <> struct shape_traits< species::box, snark::geometry::convex_polytope >
{
    static snark::geometry::convex_polytope make( const typename species::box::filter&, double ) { std::cerr << "points-grep: box: making box from stream: not implemented" << std::endl; exit( 1 ); }
    
    static void set_default_filter( typename species::box::filter& f, const comma::command_line_options& options, const comma::csv::options& csv ) { return; }
};

template <> struct shape_traits< species::prism, snark::geometry::convex_polytope >
{
    static snark::geometry::convex_polytope make( const typename species::prism::filter& f, double inflate_by = 0 )
    {
        if( f.corners.size() < 3 ) { std::cerr << "points-grep: prism: expected at least 3 corners, got: " << f.corners.size() << std::endl; exit( 1 ); }
        unsigned int size = f.corners.size() + 2;
        Eigen::MatrixXd normals( size, 3 );
        Eigen::VectorXd distances( size );
        for( unsigned int i = 1; i < f.corners.size(); ++i )
        {
            const Eigen::Vector3d& n = f.axis.coordinates.cross( f.corners[i] - f.corners[ i - 1 ] ).normalized(); 
            normals.row( i ) = n.transpose();
            distances( i ) = n.dot( f.corners[i] );
        }
        const Eigen::Vector3d& n = f.axis.coordinates.cross( f.corners[0] - f.corners.back() ).normalized(); 
        normals.row( 0 ) = n.transpose();
        distances( 0 ) = n.dot( f.corners[0] );        
        const Eigen::Vector3d& m = ( ( f.corners[2] - f.corners[1] ).cross( f.corners[0] - f.corners[1] ) ).normalized(); 
        normals.row( f.corners.size() ) = m.transpose();
        distances( f.corners.size() ) = m.dot( f.corners[0] );
        normals.row( f.corners.size() + 1 ) = -m.transpose();
        distances( f.corners.size() + 1 ) = -m.dot( f.corners[0] + f.axis.coordinates );
        for( unsigned int i = 0; i < distances.size(); ++i ) { distances[i] += inflate_by; }
        return snark::geometry::convex_polytope( normals, distances );
    }
    
    static void set_default_filter( typename species::prism::filter& f, const comma::command_line_options& options, const comma::csv::options& csv )
    {
        bool has_corners = csv.has_some_of_fields( "filter,filter/corners" );
        if( options.exists( "--corners" ) )
        {
            if( !has_corners ) { return; }
            std::cerr << "points-grep: prism: expected either --corners or filter/corners in input fields, got both" << std::endl;
            exit( 1 );
        }
        if( !has_corners ) { std::cerr << "points-grep: prism: please specify --corners, or filter/corners in input fields" << std::endl; exit( 1 ); }
        f.corners.resize( options.value< unsigned int >( "--corners-size,--number-of-corners" ) );
        f.axis = comma::csv::ascii< ::normal >().get( options.value< std::string >( "--axis", "0,0,1,0" ) );
    }
};

template < typename Species, typename Genus > static int run( const boost::optional< Genus >& shape, const comma::command_line_options& options )
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
    input_t< Species > default_input( comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--point", "0,0,0" ) )
                                    , typename Species::filter( comma::csv::ascii< ::position >().get( options.value< std::string >( "--position", "0,0,0,0,0,0" ) ) ) );
    shape_traits< Species, Genus >::set_default_filter( default_input.filter, options, csv );
    bool output_all = options.exists( "--output-all,--all" );
    double inflate_by = options.value( "--inflate-by", 0.0 );
    boost::optional< Genus > transformed;
    if( shape && !csv.has_some_of_fields( "filter,filter/coordinates,filter/coordinates/x,filter/coordinates/y,filter/coordinates/z,filter/orientation,filter/orientation/roll,filter/orientation/pitch,filter/orientation/yaw" ) ) { transformed = transform( *shape, default_input.filter ); }
    comma::csv::input_stream< input_t< Species > > istream( std::cin, csv, default_input );
    comma::csv::output_stream< output_t > ostream( std::cout, csv.binary() );
    comma::csv::passed< input_t< Species > > passed( istream, std::cout );
    comma::csv::tied< input_t< Species >, output_t > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const input_t< Species >* p = istream.read();
        if( !p ) { break; }
        bool keep = transformed ? transformed->has( *p ) : transform( shape ? *shape : shape_traits< Species, Genus >::make( p->filter, inflate_by ), p->filter ).has( *p );
        if( output_all ) { tied.append( output_t( keep ) ); }
        else if( keep ) { passed.write(); }
        if( csv.flush ) { std::cout.flush(); }
    }
    return 0;
}

template < typename Species, typename Genus > int run( const Genus& shape, const comma::command_line_options& options ) { return run< Species >( boost::optional< Genus >( shape ), options ); }

namespace snark { namespace operations { namespace polygons {

typedef boost::geometry::model::d2::point_xy< double > point_t;
typedef boost::geometry::model::polygon< point_t, true, false > polygon_t;
typedef boost::geometry::model::linestring< point_t > line_t;
typedef line_t boundary_t;

// returns a list of polygons, boundary_t is its ring boundary
std::vector< std::pair< polygon_t, boundary_t > > read_polygons(comma::command_line_options& options)
{
    const std::string& polygons_file = options.value< std::string >( "--polygons", "" );
    comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( polygons_file );
    filter_csv.full_xpath = true;
    comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
    comma::csv::input_stream< polygon_point > polystream( *is, filter_csv );
    
    std::vector< std::pair< polygon_t, boundary_t > > polygons;
    boundary_t ring;    // clockwise, or counter-clockwise, it does not seem to matter
    comma::uint32 current_id = 0;
    while( polystream.ready() || ( is->good() && !is->eof() ) )
    {
        const polygon_point* p = polystream.read();
        if( !p ) { break; }
        if( ring.empty() || current_id == p->id ) { ring.push_back( { p->x(), p->y() } ); } 
        else 
        { 
            auto data = std::make_pair(polygon_t(), ring);
            boost::geometry::append( data.first, ring );
            data.second.push_back( ring.front() ); // make linestring to be a loop, polygon's boundary
            polygons.push_back( std::move(data)  ); 
            ring.clear(); 
            ring.push_back( { p->x(), p->y() } );
        }
        current_id = p->id; 
    }
    if( !ring.empty() ) { polygons.push_back( std::make_pair(polygon_t(), ring) );  boost::geometry::append( polygons.back().first, ring ); polygons.back().second.push_back( ring.front() ); }
    if( verbose ) { std::cerr << "points-grep: total number of polygons: " << polygons.size() << std::endl; }
    return std::move( polygons );
}

static point_t make_geometry( const Eigen::Vector2d& v ) { return point_t( v.x(), v.y() ); }

static line_t make_geometry( const std::pair< Eigen::Vector2d, Eigen::Vector2d >& v )
{ 
    line_t line;
    line.push_back( make_geometry( v.first ) ); // todo: watch performance
    line.push_back( make_geometry( v.second ) ); // todo: watch performance
    return std::move(line);
}

static bool within_( const point_t& g, const std::pair< polygon_t, boundary_t >& p ) { return boost::geometry::within( g, p.first ); }

static bool within_( const line_t& g, const std::pair< polygon_t, boundary_t >& p ) { return boost::geometry::within( g[0], p.first ) && boost::geometry::within( g[1], p.first ) && !boost::geometry::intersects( g, p.second ); }

// todo
// - flags_t -> snark::operations::polygons::flags_t
// - rename "contains" to "flags" in fields
// - move all polytope-related stuff into snark::operations::polytopes
// - --restrictive: if present, consider polygon restrictive (mutually exclusive with "restrictive" field, if the latter is present)
// - --polygon-fields
// done - fix --*-fields, --*-format (make it operation-dependent)
// - polygons
//   - --polygons: default fields: x,y
// - box
//   --input-fields
//   --output-fields
//   --output-format
// - prism
//   --input-fields
//   --output-fields
//   --output-format
// - ark, leafy: git grep points-grep: fix --input/--output-fields; --output-format (pass operation)

template < typename T > static int run( const comma::csv::options& csv, const std::vector< std::pair< polygon_t, boundary_t > >& polygons )
{
    comma::csv::input_stream< T > istream( std::cin, csv );
    flags_t outputs( polygons.size() );
    comma::csv::output_stream< flags_t > ostream( std::cout, csv.binary(), true, false, outputs );
    comma::csv::tied< T, flags_t > tied( istream, ostream );
    while( istream.ready() || ( std::cin.good() && !std::cin.eof() ) )
    {
        const T* p = istream.read();
        if( !p ) { break; }
        const auto& g = make_geometry( *p );
        for( std::size_t i=0; i<polygons.size(); ++i ) { outputs.flags[i] = within_( g, polygons[i] ); }
        tied.append( outputs );
    }
    return 0;
}

} } } // namespace snark { namespace operations { namespace polygons {

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv, usage );
        verbose = options.exists( "--verbose,-v" );
        bool output_all = options.exists( "--output-all,--all" );
        const std::vector< std::string >& unnamed = options.unnamed("--output-all,--all,--verbose,-v,--flush","-.*");
        if(!unnamed.size()) { std::cerr << "points-grep: expected filter name, got nothing"<< std::endl; return 1; }
        std::string what = unnamed[0];
        if( what == "polytope" || what == "planes" )
        {
            if( options.exists( "--input-fields" ) ) { std::cerr << comma::join( comma::csv::names< input_t< species::polytope > >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--input-format" ) ) { std::cerr << comma::csv::format::value< input_t< species::polytope > >() << std::endl; return 0; }
            if( options.exists( "--output-fields" ) ) { if( output_all ) { std::cerr << comma::join( comma::csv::names< output_t >( true ), ',' ) << std::endl; } return 0; }
            if( options.exists( "--output-format" ) ) { if( output_all ) { std::cerr << comma::csv::format::value< output_t >() << std::endl; } return 0; }
            
            if( options.exists( "--normals-fields" ) ) { std::cerr << comma::join( comma::csv::names< ::normal >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--normals-format" ) ) { std::cerr << comma::csv::format::value< ::normal >() << std::endl; return 0; }
            if( options.exists( "--planes-fields" ) ) { std::cerr << comma::join( comma::csv::names< snark::triangle >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--planes-format" ) ) { std::cerr << comma::csv::format::value< snark::triangle >() << std::endl; return 0; }
            
            options.assert_mutually_exclusive( "--normals,--planes" );
            const std::string& normals_input = options.value< std::string >( "--normals", "" );
            const std::string& planes_input = options.value< std::string >( "--planes", "" );
            double inflate_by = options.value( "--inflate-by", 0.0 );
            if( !normals_input.empty() )
            {
                comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( normals_input );
                filter_csv.full_xpath = true;
                comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
                comma::csv::input_stream< ::normal > istream( *is, filter_csv );
                species::polytope::filter f;
                while( istream.ready() || ( is->good() && !is->eof() ) )
                {
                    const ::normal* p = istream.read();
                    if( !p ) { break; }
                    f.normals.push_back( *p );
                }
                return run< species::polytope >( shape_traits< species::polytope >::make( f, inflate_by ), options );
            }
            else if( !planes_input.empty() )
            {
                comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( planes_input );
                filter_csv.full_xpath = true;
                comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
                comma::csv::input_stream< snark::triangle > istream( *is, filter_csv );
                std::vector< snark::triangle > planes;
                while( istream.ready() || ( is->good() && !is->eof() ) )
                {
                    const snark::triangle* p = istream.read();
                    if( !p ) { break; }
                    planes.push_back( *p );
                }
                Eigen::MatrixXd normals( planes.size(), 3 );
                Eigen::VectorXd distances( planes.size() );
                for( unsigned int i = 0; i < planes.size(); ++i )
                {
                    const Eigen::Vector3d& normal = ( planes[i].corners[1] - planes[i].corners[0] ).cross( planes[i].corners[2] - planes[i].corners[0] ).normalized();
                    normals.row( i ) = normal;
                    distances[i] = normal.dot( planes[i].corners[0] + normal * inflate_by );
                }
                return run< species::polytope >( snark::geometry::convex_polytope( normals, distances ), options );
            }
            return run< species::polytope >( boost::optional< snark::geometry::convex_polytope >(), options );
        }
        else if( what == "polygons" )
        {
            if( options.exists( "--input-fields" ) ) { std::cerr << "points-grep polygons: --input-fields not implemented, since it is ambiguous; see --help instead" << std::endl; return 1; }
            if( options.exists( "--output-fields" ) ) { std::cerr << "points-grep polygons: --output-fields not implemented, since it is hard to define; see --help instead" << std::endl; return 1; }
            if( options.exists( "--output-format" ) ) { std::cerr << "points-grep polygons: --output-format not implemented, since it is hard to define; see --help instead" << std::endl; return 1; }
            // todo: --polygon-fields
            const auto& polygons = snark::operations::polygons::read_polygons(options);
            if( polygons.empty() ) { std::cerr << "points-grep: please specify at least one polygon in --polygons=" << std::endl; return 1; }
            comma::csv::options csv(options);
            csv.full_xpath = true;
            bool is_line_mode = csv.has_some_of_fields( "first,second,first/x,first/y,second/x,second/y" );
            return is_line_mode
                 ? snark::operations::polygons::run< std::pair< Eigen::Vector2d, Eigen::Vector2d > >( csv, polygons )
                 : snark::operations::polygons::run< Eigen::Vector2d >( csv, polygons );
        }
        else if( what == "prism" )
        {
            if( options.exists( "--corners-fields" ) ) { std::cerr << comma::join( comma::csv::names< Eigen::Vector3d >( true ), ',' ) << std::endl; return 0; }
            if( options.exists( "--corners-format" ) ) { std::cerr << comma::csv::format::value< Eigen::Vector3d >() << std::endl; return 0; }
            
            const std::string& corners_input = options.value< std::string >( "--corners", "" );
            if( corners_input.empty() ) { return run< species::prism >( boost::optional< snark::geometry::convex_polytope >(), options ); }
            comma::csv::options filter_csv = comma::name_value::parser( "filename" ).get< comma::csv::options >( corners_input );
            comma::io::istream is( filter_csv.filename, filter_csv.binary() ? comma::io::mode::binary : comma::io::mode::ascii );
            comma::csv::input_stream< Eigen::Vector3d > istream( *is, filter_csv );
            species::prism::filter f;
            while( istream.ready() || ( is->good() && !is->eof() ) )
            {
                const Eigen::Vector3d* p = istream.read();
                if( !p ) { break; }
                f.corners.push_back( *p );
            }
            f.axis = comma::csv::ascii< ::normal >().get( options.value< std::string >( "--axis", "0,0,1,0" ) );
            return run< species::prism >( shape_traits< species::prism >::make( f, options.value( "--inflate-by", 0.0 ) ), options );
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
                else { std::cerr << "points-grep: box: got --origin; please specify --end or --size" << std::endl; return 1; }
            }
            if( !end )
            {
                if( origin && size ) { end = *origin + *size; }
                else if( size ) { end = centre + *size / 2; }
                else { std::cerr << "points-grep: box: got --end; please specify --origin or --size" << std::endl; return 1; }
            }
            centre = ( *origin + *end ) / 2 ;
            Eigen::Vector3d inflate_by = comma::csv::ascii< Eigen::Vector3d >().get( options.value< std::string >( "--inflate-box-by", "0,0,0" ) );
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
            return run< species::box >( snark::geometry::convex_polytope( normals, distances ), options );
        }
        std::cerr << "points-grep: expected filter name, got: \"" << what << "\"" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << "points-grep: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "points-grep: unknown exception" << std::endl; }
    return 1;
}
