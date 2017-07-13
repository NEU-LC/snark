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

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "../../visiting/eigen.h"
#include <QApplication>
#include "view_points/shape_reader.h"
#include "view_points/main_window.h"
#include "../qt3d/camera_options.h"
#if Qt3D_VERSION==1
#include "view_points/qt3d_v1/viewer.h"
#include "view_points/qt3d_v1/model_reader.h"
#include "view_points/qt3d_v1/texture_reader.h"
#else
#include "view_points/controller.h"
#endif

#if Qt3D_VERSION==2
#include "view_points/traits.h"
#endif

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char * completion_options =
        " --help -h"
        " --version"
        " --colour --color -c"
        " --exit-on-end-of-input"
        " --fill"
        " --label"
        " --no-stdin"
        " --pass-through --pass"
        " --point-size --weight"
        " --shape"
        " --size"
        " --camera"
        " --fov"
        " --camera-config"
        " --camera-position"
        " --orthographic"
        " --background-colour"
        " --output-camera-config --output-camera"
        " --scene-center --center"
        " --scene-radius --radius"
        " --title"
        " --z-is-up";
   
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage()
{
#if Qt3D_VERSION==2
    static const char * const usage_qt55_warning =
        "\nWARNING: this version of view-points is compiled against Qt5.5+"
        "\nIt will not be fully functional (yet)."
        "\n"
        "\nUnsupported features are shown in dimmed text in this help"
        "\n"
        "\nFor an example of current functionality try:"
        "\n    snark-graphics-test-pattern cube 100000 0.1 0.01 | view-points --fields=x,y,z,r,g,b,a"
        "\n"
        "\n----------------------------------------------"
        "\n";

    #define qt55_unsupported_marker_start "\x1B[0;90m"
    #define qt55_unsupported_marker_end "\x1B[0m"
#else
    #define qt55_unsupported_marker_start ""
    #define qt55_unsupported_marker_end ""
#endif

    static const char * const usage_synopsis = 
        "\nview 3D point clouds:"
        "\nview points from given files/streams and stdin"
        "\n(see examples below for a quick start)"
        "\n"
        "\nnote: scene radius, centre and point of view will be decided depending on"
        "\n      the extents of the first data source (if you want it to be stdin,"
        "\n      specify \"-\" as the explicitly as the first data source); see also"
        "\n      --scene-radius option and examples"
        "\n"
        "\nusage: view-points [<options>] [<filenames>]"
        "\n";
        
    static const char * const usage_options = 
        "\ninput data options"
        "\n    --colour,--color,-c <how>: how to colour points"
        "\n        <how>:"
        "\n            colour maps"
        "\n                <min>:<max>: if scalar field present, stretch by scalar (see section <fields>)"
        "\n                <min>:<max>,<from-colour>:<to-colour>: if scalar field present, stretch by scalar (see section <fields>)"
        "\n                <min>:<max>,<colourmap>: if scalar field present, stretch by scalar (see section <fields>)"
        "\n                    <colourmap>: jet, hot, red, green"
        "\n"
        "\n            fixed colour"
        "\n                white, black, red, green, blue, yellow, cyan, magenta, grey, pink, sky, salad: fixed colour"
        "\n"
        "\n            colour by id"
        "\n                if there is \"id\" field in --fields, colour by id, with or without scalar (see section <fields>)"
        "\n"
        "\n            colour by elevation"
        "\n                [<min>:<max>][,<from-colour>:<to-colour>][,cyclic][,sharp][,quadratic] (in any order)"
        "\n                    stretch colours by elevation"
        "\n                    <min>:<max>: from-to in metres, e.g. -3:10"
        "\n                    <from-colour>:<to-colour>: from which to which colour"
        "\n                    cyclic: if present, colour cyclically with <min> and <max>"
        "\n                            meaning the height of the corresponding stip"
        "\n                    sharp: if present (for cyclic only), do not make smooth borders"
        "\n                           between the colours"
        "\n                    quadratic: if present (for cyclic only), parabolic stretching, quick and dirty"
        "\n                                default: stretch colours linearly"
        "\n                    e.g.: cat data.csv | view-points --colour=blue:yellow,1:2,cyclic"
        "\n"
        "\n            default: stretched by elevation from cyan to magenta from 0:1"
        "\n"
        qt55_unsupported_marker_start
        "\n      hide: e.g. \"test.csv;hide\": hide the source, when shown first time (useful, when there are very many inputs"
        qt55_unsupported_marker_end
        "\n    --exit-on-end-of-input: exit immediately on end of input stream"
        "\n    --fill: fill the shape; currently implemented only for triangles"
        "\n    --label <label>: text label displayed next to the latest point"
        "\n    --no-stdin: do not read from stdin"
        "\n    --pass-through,--pass; pass input data to stdout"
        "\n    --point-size,--weight <point size>: default: 1"
        "\n    --shape <shape>: \"point\", \"extents\", \"line\", \"label\"; default \"point\""
        "\n                     \"arc\": e.g: --shape=arc --fields=,,begin,end,centre,"
        "\n                               or: --shape=arc --fields=,,begin,middle,end,,,"
        "\n                                   where 'begin' and 'end' are x,y,z points"
        "\n                                   and 'middle' is a point between begin and end on the arc"
        "\n                                   default: 'begin,end', with centre 0,0,0"
        "\n                     \"ellipse\": e.g. --shape=ellipse --fields=,,center,orientation,minor,major,"
        "\n                                  orientation: roll,pitch,yaw; default: in x,y plane"
        "\n                     \"extents\": e.g. --shape=extents --fields=,,min,max,,,"
        qt55_unsupported_marker_start
        "\n                     \"label\": e.g. --shape=label --fields=,x,y,z,,,label"
        qt55_unsupported_marker_end
        "\n                     \"line\": e.g. --shape=line --fields=,,first,second,,,"
        "\n                     \"lines\": connect all points of a block from first to the last; fields same as for 'point'"
        "\n                     \"loop\": connect all points of a block; fields same as for 'point'"
        "\n                     \"triangle\": e.g. --shape=triangle --fields=,,corners,,,"
        "\n                                     or --shape=triangle --fields=,,corners[0],,,corners[1],,,corners[2],,,"
        "\n                                     or --shape=triangle --fields=,,corners[0]/x,,corners[0]/y,,corners[0]/z,,,,corners[1],,,corners[2],,, etc"
        "\n                     \"axis\": draws three axis lines per record, in red/green/blue corresponding to x/y/z axes, using position and orientation e.g. --shape=axis --fields=position,orientation"
        "\n                                     fields: position,orientation"
        "\n                                         default fields: position,orientation"
        "\n                                         position: x,y,z or position/x,position/y,position/z"
        "\n                                         orientation: roll,pitch,yaw or orientation/roll,orientation/pitch,orientation/yaw"
        "\n                                     options: options can be specified as command line option and/or in each stream (without -- prefix)"
        "\n                                         --length=<d>: length of each axis line"
        "\n                                         --labels=\"<x>:<y>:<z>\" colon separated list of axis labels, leave empty for no labels."
#if Qt3D_VERSION==2
        "\n                                         --weight or --point-size can be used for line thickness"
#endif
        "\n                                         note: colour options are currently not effective for axis shape"
        qt55_unsupported_marker_start
        "\n                     \"<model file ( obj, ply... )>[;<options>]\": e.g. --shape=vehicle.obj"
        "\n                     \"    <options>"
        "\n                     \"        flip\": flip the model around the x-axis"
        "\n                     \"        scale=<value>\": resize model (ply only, todo), e.g. show model half-size: scale=0.5"
        "\n                     \"<image file>[,<image options>]:<image file>[,<image options>]\": show image, e.g. --shape=\"vehicle-lights-on.jpg,vehicle-lights-off.jpg\""
        "\n                            <image options>: <width>,<height> or <pixel-size>"
        "\n                                <width>,<height>: image width and height in meters when displaying images in the scene; default: 1,1"
        "\n                                <pixel size>: single pixel size in metres"
        "\n                            note 1: just like for the cad models, the images will be pinned to the latest point in the stream"
        "\n                            note 2: specify id in fields to switch between multiple images, see examples below"
        qt55_unsupported_marker_end
        "\n    --size <size>: render last <size> points (or other shapes)"
        "\n                   default 2000000 for points, for 200000 for other shapes"
        qt55_unsupported_marker_start
        "\n    --title <title>: title for source, defaults to filename"
        "\n                     if set to \"none\" don't show source in selection box"
        "\n                     (but still display data and checkbox)"
        "\n"
        qt55_unsupported_marker_end
        "\ncamera options"
        "\n    --camera=\"<options>\""
        "\n          <options>: [fov=<fov>];[<type>]"
        "\n          <fov>: field of view in degrees, default 45 degrees"
        "\n          <type>: orthographic | perspective"
        "\n              default: perspective"
        "\n    --fov=<fov>: set camera field of view in degrees"
        qt55_unsupported_marker_start
        "\n    --camera-config=<filename>: camera config in json; to see an example, run --output-camera-config"
        "\n    --camera-position=\"<options>\""
        "\n          <options>: <position>|<stream>"
        "\n          <position>: <x>,<y>,<z>,<roll>,<pitch>,<yaw>"
        "\n          <stream>: position csv stream with options; default fields: x,y,z,roll,pitch,yaw"
        qt55_unsupported_marker_end
        "\n    --orthographic: use orthographic projection instead of perspective"
        qt55_unsupported_marker_start
        "\n"
        "\nmore options"
        "\n    --background-colour <colour> : default: black"
        "\n    --output-camera-config,--output-camera: output camera position as t,x,y,z,r,p,y to stdout"
        "\n    --scene-center,--center=<value>: fixed scene center as \"x,y,z\""
        "\n    --scene-radius,--radius=<value>: fixed scene radius in metres, since sometimes it is hard to imply"
        "\n                            scene size from the dataset (e.g. for streams)"
        "\n    --z-is-up : z-axis is pointing up, default: pointing down ( north-east-down system )"
        qt55_unsupported_marker_end
        "\n";
        
    static const char * const usage_csv_options = 
        "\n"
        "\n    fields:"
        "\n        default: x,y,z"
        "\n        x,y,z: coordinates (%d in binary)"
        "\n        id: if present, colour by id (%ui in binary)"
        "\n        block: if present, clear screen once block id changes (%ui in binary)"
        "\n        r,g,b: if present, specify RGB colour (0-255; %uc in binary)"
        "\n        a: if present, specifies colour transparency (0-255, %uc in binary); default 255"
        "\n        scalar: if present, colour by scalar"
        "\n                  use --colour=<from>:<to>[,<from colour>:<to colour>]"
        "\n                  default: 0:1,cyan:magenta"
        "\n                  todo: implement for shapes (currently works only for points)"
        "\n        label: text label (currenly implemented for ascii only)"
        "\n        roll,pitch,yaw: if present, show orientation"
        "\n"
        "\n    most of the options can be set for individual files (see examples)"
        "\n";
        
    static const char * const usage_examples = 
        "\nmouse clicks:"
        "\n    left press and hold: rotate the scene around the centre"
        "\n    right press and hold: translate the scene"
        "\n    double left click: change the centre of the scene"
        qt55_unsupported_marker_start
        "\n    double right click: output to stdout approximate coordinates of the clicked point"
        qt55_unsupported_marker_end
        "\n    scroll wheel: zoom"
        "\n"
        "\nexamples"
        "\n"
        "\nbasics"
        "\n    view points from file:"
        "\n        view-points xyz.csv"
        "\n"
        "\n    hint that the file contains not more than 200000 points"
        "\n        cat $(ls *.csv) | view-points --size=200000"
        "\n"
        "\n    view points from all the binary files in the directory"
        "\n        cat $(ls *.bin) | view-points --size=200000 --binary \"%d%d%d\""
        "\n"
        "\n    colour points"
        "\n        view-points --colour blue $(ls labeled.*.csv)"
        "\n"
        "\n    each point has an individual color:"
        "\n        cat xyzrgb.csv | view-points --fields=\"x,y,z,r,g,b\""
        "\n"
        "\n    view multiple files"
        "\n        view-points \"raw.csv;colour=0:20\" \"partitioned.csv;fields=x,y,z,id;point-size=2\""
        "\n"
        qt55_unsupported_marker_start
        "\n    view multiple files with titles"
        "\n        view-points \"raw.csv;colour=0:20;title=raw\" \"partitioned.csv;fields=x,y,z,id;point-size=2;title=partitioned\""
        "\n"
        "\n    view a cad model"
        "\n        echo \"0,0,0\" | view-points --shape /usr/local/etc/segway.shrimp.obj --z-is-up --orthographic"
        "\n"
        "\n    use stdin as the primary source for scene radius:"
        "\n        cat xyz.csv | view-points \"-\" scan.csv"
        "\n"
        "\n    specify fixed scene radius explicitly:"
        "\n        cat xyz.csv | view-points --scene-radius=100"
        "\n"
        qt55_unsupported_marker_end
        "\n    passing input data through:"
        "\n        cat xyz.csv | view-points \"-;pass-through\" scan.csv"
        "\n"
        qt55_unsupported_marker_start
        "\nusing images"
        "\n    show image with given position"
        "\n        echo 0,0,0 | view-points \"-;shape=image.jpg\""
        "\n"
        "\n    show resized image"
        "\n        echo 0,0,0 | view-points \"-;shape=image.jpg,3,4\""
        "\n"
        "\n    specify pixel size instead of image size"
        "\n        echo 0,0,0 | view-points \"-;shape=image.jpg,0.1\""
        "\n"
        "\n    show image with given position and orientation"
        "\n        echo 0,0,0,0,0,0 | view-points \"-;shape=image.jpg;fields=x,y,z,roll,pitch,yaw\""
        "\n"
        "\n    switch between images by their index"
        "\n        echo 0,0,0,0 > points.csv"
        "\n        echo 0,0,0,1 >> points.csv"
        "\n        echo 0,0,0,2 >> points.csv"
        "\n        echo points.csv | view-points \"-;shape=image1.jpg,image2.jpg,image3.jpg;fields=x,y,z,id\""
        "\n"
        "\n    show points selected with a double right click"
        "\n        rm -rf pipe && mkfifo pipe && cat pipe | view-points \"rose.st.ground.csv;fields=x,y,z,r,g,b\" \"-;colour=sky;weight=10\" > pipe"
        "\n"
        "\n    publish a real time playback of georeferenced velodyne data on port 12345, visualise the data in real time and show points selected with a double right click"
        "\n        cat velodyne-georeferenced.bin | csv-play --binary t,3d,ui | io-publish --size $( csv-size t,3d,ui ) -m 10000 tcp:12345"
        "\n        rm -rf pipe && mkfifo pipe && cat pipe | view-points \"tcp:localhost:12345;binary=t,3d,ui;fields=,x,y,z,block\" \"-;fields=x,y,z;colour=sky;weight=10\" > pipe"
        "\n"
        "\n    similar to above but uses different colours for the shown points and adds labels next to the points indicating the click order "
        "\n        cat velodyne-georeferenced.bin | csv-play --binary t,3d,ui | io-publish --size $( csv-size t,3d,ui ) -m 10000 tcp:12345"
        "\n        rm -rf pipe && mkfifo pipe && cat pipe | view-points \"tcp:localhost:12345;binary=t,3d,ui;fields=,x,y,z,block\" \"-;fields=x,y,z,id,label;weight=10\" | csv-paste \"-\" line-number line-number > pipe"
        "\n"
        qt55_unsupported_marker_end
        "\n    an example of many of the supported shapes"
        "\n        for i in {0..15}; do echo \"a=2*3.1415926532/16*$i;s(a)*3;c(a)*3;s(a)*3\" | bc -l | paste -s -d,; done \\"
        "\n            | view-points \"-;weight=5;color=cyan;label=points\" \\"
        "\n                  <( echo 0,0,0,1,1,1 )\";shape=extents;label=extents;color=blue\" \\"
        "\n                  <( echo 0,0,2,0,0,0,0.5,2 )\";shape=ellipse;label=ellipse;color=salad\" \\"
        "\n                  <( echo -e \"0,0,-2,0\\n0,1,-2,1\\n0.5,1.5,-2,2\\n1,1,-2,3\\n1,0,-2,4\\n0.5,-0.5,-2,5\" )\";shape=loop;fields=x,y,z,id;label=loop\" \\"
        "\n                  <( echo 2,2,-1,-2,-1,-1 )\";shape=arc;label=arc;color=magenta\"\\"
        "\n                  <( echo '-3,-3,-3,0,0,0,6,\"X:Y:Z\"' )\";shape=axis;fields=position,orientation,length,axis_labels\""
        "\n";

    std::cerr
        #if Qt3D_VERSION==2
        << usage_qt55_warning
        #endif
        << usage_synopsis
        << usage_options
        << "\ncsv options\n"
        << comma::csv::options::usage()
        << usage_csv_options
        << usage_examples 
        #if Qt3D_VERSION==2
        << usage_qt55_warning
        #endif
        << std::endl;
    exit( 1 );
}

struct model_options
{
    std::string filename;
    bool flip;
    double scale;
    model_options() : flip( false ), scale( 1.0 ) {}
};

namespace comma { namespace visiting {

template <> struct traits< model_options >
{
    template < typename Key, class Visitor > static void visit( Key, model_options& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "flip", p.flip );
        v.apply( "scale", p.scale );
    }

    template < typename Key, class Visitor > static void visit( Key, const model_options& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "flip", p.flip );
        v.apply( "scale", p.scale );
    }
};

} } // namespace comma { namespace visiting {

static bool data_passed_through = false;

// quick and dirty, todo: a proper structure, as well as a visitor for command line options
std::unique_ptr< snark::graphics::view::Reader > make_reader( const comma::command_line_options& options
                                                             , const comma::csv::options& csv_options
                                                             , const std::string& properties = "" )
{
    //snark::graphics::view::color_t 
    QColor background_color( QColor( QString( options.value< std::string >( "--background-colour", "#000000" ).c_str() ) ) );
    std::string shape = options.value< std::string >( "--shape", "point" );
    snark::graphics::view::Reader::reader_parameters param( csv_options
                                                          , options.value( "--title", csv_options.filename )
                                                          , options.value< std::size_t >( "--size", shape == "point" ? 2000000 : 200000 )
                                                          , options.value( "--point-size,--weight", 1u )
                                                          , options.exists( "--pass-through,--pass" )
                                                          , options.exists( "--fill" )
                                                          ,options.value<std::string>("--labels","")
                                                          ,options.value<double>("--length",1));
    std::string color = options.exists( "--colour" ) ? options.value< std::string >( "--colour" )
                      : options.exists( "--color" ) ? options.value< std::string >( "--color" )
                      : options.value< std::string >( "-c", "" );
    std::string label = options.value( "--label", std::string() );
    bool show = true;
    if( properties != "" )
    {
        comma::name_value::parser nameValue( "filename", ';', '=', false );
        param.options = nameValue.get( properties, csv_options );
        comma::name_value::map m( properties, "filename", ';', '=' );
        param.size = m.value( "size", param.size );
        param.point_size = m.value( "point-size", param.point_size );
        param.point_size = m.value( "weight", param.point_size );
        param.title = m.value( "title", param.title.empty() ? param.options.filename : param.title );
        shape = m.value( "shape", shape );
        if( m.exists( "colour" ) ) { color = m.value( "colour", color ); }
        else if( m.exists( "color" ) ) { color = m.value( "color", color ); }
        label = m.value( "label", label );
        show = !m.exists( "hide" );
        param.pass_through = param.pass_through || ( m.exists( "pass-through" ) || m.exists( "pass" ));
        param.fill = param.fill || m.exists( "fill" );
        param.labels=m.value("labels",param.labels);
        param.length=m.value("length",param.length);
    }
    if( param.pass_through )
    {
        if( data_passed_through ) { COMMA_THROW( comma::exception, "only one input stream can be given \"pass-through\" option" ); }
        data_passed_through = true;
    }
    if( param.title == "none" ) { param.title = ""; }
    if( !show ) { std::cerr << "view-points: " << ( param.title.empty() ? param.options.filename : param.title )<< " will be hidden on startup; tick the box next to the name to make it visible" << std::endl; }
    snark::graphics::view::colored* colored = snark::graphics::view::color_from_string( color, param.options.fields, background_color );
    if( shape == "point" )
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z"; }
        std::vector< std::string > v = comma::split( param.options.fields, ',' );
        bool has_orientation = false;
        for( unsigned int i = 0; !has_orientation && i < v.size(); ++i ) { has_orientation = v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw"; }
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< Eigen::Vector3d >( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    if( shape == "loop" )
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z"; }
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< Eigen::Vector3d, snark::graphics::view::how_t::loop >( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    if( shape == "lines" ) // todo: get a better name
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z"; }

        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< Eigen::Vector3d, snark::graphics::view::how_t::connected >( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    if( shape == "label" )
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z,label"; }
        // TODO
    }
    else if( shape == "ellipse" )
    {
        if( param.options.fields == "" ) { param.options.fields="center,orientation,major,minor"; }
        std::vector< std::string > v = comma::split( param.options.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i )
        {
            if( v[i] == "x" || v[i] == "y" || v[i] == "z" ) { v[i] = "center/" + v[i]; }
            else if( v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "orientation/" + v[i]; }
        }
        param.options.fields = comma::join( v, ',' );
    }
    else if( shape == "arc" )
    {
        if( param.options.fields == "" ) { param.options.fields="begin,end"; }
        std::vector< std::string > v = comma::split( param.options.fields, ',' );
    }
    else if( shape == "extents" )
    {
        if( param.options.fields == "" ) { param.options.fields="min,max"; }
    }
    else if( shape == "line" )
    {
        if( param.options.fields == "" ) { param.options.fields="first,second"; }
    }
    else if( shape == "triangle" )
    {
        if( param.options.fields == "" ) { param.options.fields="corners"; }
    }
    else if(shape=="axis")
    {
        if( param.options.fields == "" ) { param.options.fields="position,orientation"; }
        std::vector< std::string > v = comma::split( param.options.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i )
        {
            if( v[i] == "x" || v[i] == "y" || v[i] == "z" ) { v[i] = "position/" + v[i]; }
            else if( v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "orientation/" + v[i]; }
        }
        param.options.fields = comma::join( v, ',' );
    }
    else
    {
        if( param.options.fields == "" ) { param.options.fields="point,orientation"; param.options.full_xpath = true; }
#if Qt3D_VERSION==1
            std::vector< snark::graphics::view::TextureReader::image_options > image_options;
            std::vector< std::string > v = comma::split( shape, ':' );
            for( unsigned int i = 0; i < v.size(); ++i )
            {
                std::vector< std::string > w = comma::split( v[i], ',' );
                std::string e = comma::split( w[0], '.' ).back();
                if( e != "png" && e != "jpg" && e != "jpeg" && e != "bmp" && e != "gif" ) { break; }
                switch( w.size() )
                {
                    case 1: image_options.push_back( snark::graphics::view::TextureReader::image_options( w[0] ) ); break;
                    case 2: image_options.push_back( snark::graphics::view::TextureReader::image_options( w[0], boost::lexical_cast< double >( w[1] ) ) ); break;
                    case 3: image_options.push_back( snark::graphics::view::TextureReader::image_options( w[0], boost::lexical_cast< double >( w[1] ), boost::lexical_cast< double >( w[2] ) ) ); break;
                    default: COMMA_THROW( comma::exception, "expected <image>[,<width>,<height>]; got: " << shape );
                }
            }
            if( image_options.empty() )
            {
                model_options m = comma::name_value::parser( ';', '=' ).get< model_options >( properties );
                m.filename = shape;
                if( !boost::filesystem::exists( m.filename ) ) { COMMA_THROW( comma::exception, "file does not exist: " << m.filename ); }
                std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ModelReader( param, shape, m.flip, m.scale, colored, label ) );
                reader->show( show );
                return reader;
            }
            else
            {
                std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::TextureReader( param, image_options ) );
                reader->show( show );
                return reader;
            }
#else
            std::cerr << "view-points: cad models and images are not supported yet for qt version " << Qt3D_VERSION << std::endl;
            exit( 1 );
#endif
    }
    std::vector< std::string > v = comma::split( param.options.fields, ',' );
    for( std::size_t i = 0; i < v.size(); ++i )
    {
        if(    v[i] != "id"
            && v[i] != "block"
            && v[i] != "colour"
            && v[i] != "label"
            && v[i] != "scalar"
            && v[i] != "r"
            && v[i] != "g"
            && v[i] != "b"
            && v[i] != "a"
            && v[i] != "" ) { v[i] = "shape/" + v[i]; }
        if( v[i] == "r" || v[i] == "g" || v[i] == "b" || v[i] == "a" ) { v[i] = "colour/" + v[i]; }
    }
    param.options.fields = comma::join( v, ',' );
    param.options.full_xpath = true;
    if( shape == "extents" )
    {
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< snark::math::closed_interval< double, 3 > >( param, colored, label, snark::math::closed_interval< double, 3 >( Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ) ) ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "line" )
    {
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< std::pair< Eigen::Vector3d, Eigen::Vector3d > >( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "triangle" )
    {
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< snark::graphics::view::loop< 3 > >( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "ellipse" )
    {
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< snark::graphics::view::Ellipse< 25 > >( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "arc" )
    {
        snark::graphics::view::arc< 20 > sample; // quick and dirty
        if( param.options.has_field( "middle" ) || param.options.has_field( "middle/x" ) || param.options.has_field( "middle/y" ) || param.options.has_field( "middle/z" ) ) { sample.middle = Eigen::Vector3d(); }
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< snark::graphics::view::arc< 20 > >( param, colored, label, sample ) );
        reader->show( show );
        return reader;
    }
    else if(shape=="axis")
    {
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader<snark::graphics::view::axis>( param, colored, label ) );
        reader->show( show );
        return reader;
    }
    COMMA_THROW( comma::exception, "expected shape, got \"" << shape << "\"" ); // never here
}

void version()
{
    std::cerr << "Using Qt version " << QT_VERSION_STR << std::endl;
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        if( options.exists( "--version" )) { version(); exit(0); }
        if( options.exists( "--help" ) || options.exists( "-h" ) ) { usage(); }
        comma::csv::options csv_options( argc, argv );
        std::vector< std::string > properties = options.unnamed( "--z-is-up,--orthographic,--flush,--no-stdin,--output-camera-config,--output-camera,--pass-through,--pass,--exit-on-end-of-input"
                , "--binary,--bin,-b,--fields,--size,--delimiter,-d,--colour,--color,-c,--point-size,--weight,--background-colour,--scene-center,--center,--scene-radius,--radius,--shape,--label,--title,--camera,--camera-position,--camera-config,--fov,--model,--full-xpath,--labels,--length" );

#if Qt3D_VERSION==1
        QColor4ub background_color( QColor( QString( options.value< std::string >( "--background-colour", "#000000" ).c_str() ) ) );
#endif
        boost::optional< comma::csv::options > camera_csv;
        boost::optional< Eigen::Vector3d > cameraposition;
        boost::optional< Eigen::Vector3d > cameraorientation;

        snark::graphics::qt3d::camera_options camera_options
            ( options.exists( "--orthographic" )
            , options.value< double >( "--fov", 45 )
            , options.exists( "--z-is-up" ));

        if( options.exists( "--camera" ) )
        {
            std::string camera = options.value< std::string >( "--camera" );
            std::vector< std::string > v = comma::split( camera, ';' );
            for( std::size_t i = 0; i < v.size(); ++i )
            {
                if( v[i] == "orthographic" )
                {
                    camera_options.orthographic = true;
                }
                else if( v[i] == "perspective" )
                {
                    camera_options.orthographic = false;
                }
                else
                {
                    std::vector< std::string > vec = comma::split( v[i], '=' );
                    if( vec.size() == 2 && vec[0] == "fov" )
                    {
                        camera_options.field_of_view = boost::lexical_cast< double >( vec[1] );
                    }
                }
            }
        }

        bool camera_position_from_stdin = false;
#if Qt3D_VERSION==1
        QApplication application( argc, argv );
        if( options.exists( "--camera-position" ) )
        {
            std::string position = options.value< std::string >( "--camera-position" );
            comma::name_value::parser parser( "x,y,z,roll,pitch,yaw", ',', '=', false );
            snark::graphics::view::point_with_orientation pose;
            try
            {
                pose = parser.get< snark::graphics::view::point_with_orientation >( position );
                cameraposition = pose.point;
                cameraorientation = pose.orientation;
            }
            catch( ... ) {}
            if( !cameraposition )
            {
                comma::name_value::parser parser( "filename", ';', '=', false );
                try
                {
                    std::cerr << " parse " << position << std::endl;
                    camera_csv = parser.get< comma::csv::options >( position );
                    camera_csv->full_xpath = false;
                    if( camera_csv->fields.empty() ) { camera_csv->fields = "x,y,z,roll,pitch,yaw"; }
                    camera_position_from_stdin = camera_csv->filename == "-";
                }
                catch( ... ) {}
            }
        }
        boost::optional< double > scene_radius = options.optional< double >( "--scene-radius,--radius" );
        boost::optional< Eigen::Vector3d > scene_center;
        boost::optional< std::string > s = options.optional< std::string >( "--scene-center,--center" );
        if( s ) { scene_center = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", ',' ).get( *s ); }
#endif
        boost::property_tree::ptree camera_config; // quick and dirty
        if( options.exists( "--camera-config" ) ) { boost::property_tree::read_json( options.value< std::string >( "--camera-config" ), camera_config ); }

#if Qt3D_VERSION==1
        snark::graphics::view::Viewer* viewer = new snark::graphics::view::Viewer( background_color
                                                                                 , camera_options
                                                                                 , options.exists( "--exit-on-end-of-input" )
                                                                                 , camera_csv
                                                                                 , cameraposition
                                                                                 , cameraorientation
                                                                                 , options.exists( "--camera-config" ) ? &camera_config : NULL
                                                                                 , scene_center
                                                                                 , scene_radius
                                                                                 , options.exists( "--output-camera-config,--output-camera" ) );
        bool stdin_explicitly_defined = false;
        for( unsigned int i = 0; i < properties.size(); ++i )
        {
            if( comma::split( properties[i], ';' )[0] == "-" ) { stdin_explicitly_defined = true; }
            viewer->readers.push_back( make_reader( options, csv_options, properties[i] ) );
        }
        if( !stdin_explicitly_defined && !options.exists( "--no-stdin" ) && !camera_position_from_stdin )
        {
            csv_options.filename = "-";
            viewer->readers.push_back( make_reader( options, csv_options ) );
        }
        if( data_passed_through )
        {
            viewer->inhibit_stdout();
            if( options.exists( "--output-camera-config,--output-camera" ) ) { COMMA_THROW( comma::exception, "cannot use --output-camera-config whilst \"pass-through\" option is in use" ); }
        }
        snark::graphics::view::MainWindow main_window( comma::join( argv, argc, ' ' ), viewer );
        main_window.show();
        application.exec();
        delete viewer;
        return 0;       // We never actually reach this line because we raise SIGINT when closing

#elif Qt3D_VERSION==2

        color_t background_color;
        double scene_radius=options.value<double>("--scene-radius,--radius",10);
        QVector3D scene_center(0,0,0);
        boost::optional< std::string > s = options.optional< std::string >( "--scene-center,--center" );
        if( s ) { scene_center = comma::csv::ascii< QVector3D >( "x,y,z", ',' ).get( *s ); }

        QApplication app(argc, argv);
        snark::graphics::view::main_window main_window;
        snark::graphics::view::controller controller(&main_window, background_color, camera_options, options.exists( "--exit-on-end-of-input" ), camera_csv, cameraposition, cameraorientation, 
                                                     options.exists( "--camera-config" ) ? &camera_config : NULL, scene_center, scene_radius, options.exists( "--output-camera-config,--output-camera" ));
        bool stdin_explicitly_defined = false;
        for( unsigned int i = 0; i < properties.size(); ++i )
        {
            if( comma::split( properties[i], ';' )[0] == "-" ) { stdin_explicitly_defined = true; }
            controller.add(make_reader( options, csv_options, properties[i] ));
        }
        if( !stdin_explicitly_defined && !options.exists( "--no-stdin" ) && !camera_position_from_stdin )
        {
            csv_options.filename = "-";
            controller.add(make_reader( options, csv_options ));
        }
        if( data_passed_through )
        {
            controller.inhibit_stdout();
            if( options.exists( "--output-camera-config,--output-camera" ) ) { COMMA_THROW( comma::exception, "cannot use --output-camera-config whilst \"pass-through\" option is in use" ); }
        }
        main_window.resize( main_window.sizeHint() );
        main_window.show();
        return app.exec();

#else
#error Qt3D_VERSION must be 1 or 2
#endif
    }
    catch( std::exception& ex ) { std::cerr << "view-points: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "view-points: unknown exception" << std::endl; }
    return 1;
}
