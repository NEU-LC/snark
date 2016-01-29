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
#include <snark/graphics/applications/view_points/MainWindow.h>
#include <snark/graphics/applications/view_points/Viewer.h>
#include <snark/graphics/applications/view_points/ShapeReader.h>
#include <snark/graphics/applications/view_points/ModelReader.h>
#include <snark/graphics/applications/view_points/TextureReader.h>
#include <QApplication>

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char * completion_options =
        " --help -h"
        " --colour --color -c"
        " --label"
        " --no-stdin"
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
        " --z-is-up";
   
    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage()
{
    static const char * const usage_synopsis = 
        "\n"
        "\nview 3D point clouds:"
        "\nview points from given files/streams and stdin"
        "\n(see examples below for a quick start)"
        "\n"
        "\nnote: scene radius, centre and point of view will be decided"
        "\n      depending on the extents of the first data source"
        "\n      (if you want it to be stdin, specify \"-\" as the"
        "\n      explicitly as the first data source); see also"
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
        "\n      hide: e.g. \"test.csv;hide\": hide the source, when shown first time (useful, when there are very many inputs"
        "\n    --label <label>: text label displayed next to the latest point"
        "\n    --no-stdin: do not read from stdin"
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
        "\n                     \"line\": e.g. --shape=line --fields=,,first,second,,,"
        "\n                     \"lines\": connect all points of a block from first to the last; fields same as for 'point'"
        "\n                     \"loop\": connect all points of a block; fields same as for 'point'"
        "\n                     \"label\": e.g. --shape=label --fields=,x,y,z,,,label"
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
        "\n    --size <size>: render last <size> points (or other shapes)"
        "\n                   default 2000000 for points, for 200000 for other shapes"
        "\n"
        "\ncamera options"
        "\n    --camera=\"<options>\""
        "\n          <options>: [<fov>];[<type>]"
        "\n          <fov>: field of view in degrees, default 45 degrees"
        "\n          <type>: orthographic | perspective"
        "\n              default: perspective"
        "\n    --fov=<fov>: set camera field of view in degrees"
        "\n    --camera-config=<filename>: camera config in json; to see an example, run --output-camera-config"
        "\n    --camera-position=\"<options>\""
        "\n          <options>: <position>|<stream>"
        "\n          <position>: <x>,<y>,<z>,<roll>,<pitch>,<yaw>"
        "\n          <stream>: position csv stream with options; default fields: x,y,z,roll,pitch,yaw"
        "\n    --orthographic: use orthographic projection instead of perspective"
        "\n"
        "\nmore options"
        "\n    --background-colour <colour> : default: black"
        "\n    --output-camera-config,--output-camera: output camera position as t,x,y,z,r,p,y to stdout"
        "\n    --scene-center,--center=<value>: fixed scene center as \"x,y,z\""
        "\n    --scene-radius,--radius=<value>: fixed scene radius in metres, since sometimes it is hard to imply"
        "\n                            scene size from the dataset (e.g. for streams)"
        "\n    --z-is-up : z-axis is pointing up, default: pointing down ( north-east-down system )"
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
        "\n                  default: 0:1,red:blue"
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
        "\n    double right click: output to stdout approximate coordinates of the clicked point"
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
        "\n    view a cad model"
        "\n        echo \"0,0,0\" | view-points --shape /usr/local/etc/segway.shrimp.obj --z-is-up --orthographic"
        "\n"
        "\n    use stdin as the primary source for scene radius:"
        "\n        cat xyz.csv | view-points \"-\" scan.csv"
        "\n"
        "\n    specify fixed scene radius explicitly:"
        "\n        cat xyz.csv | view-points --scene-radius=100"
        "\n"
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
        "\n";

    std::cerr
        << usage_synopsis
        << usage_options
        << "\ncsv options\n"
        << comma::csv::options::usage()
        << usage_csv_options
        << usage_examples 
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

// quick and dirty, todo: a proper structure, as well as a visitor for command line options
boost::shared_ptr< snark::graphics::View::Reader > makeReader( QGLView& viewer
                                                             , const comma::command_line_options& options
                                                             , const comma::csv::options& csvOptions
                                                             , const std::string& properties = "" )
{
    QColor4ub backgroundcolour( QColor( QString( options.value< std::string >( "--background-colour", "#000000" ).c_str() ) ) );
    comma::csv::options csv = csvOptions;
    std::string shape = options.value< std::string >( "--shape", "point" );
    std::size_t size = options.value< std::size_t >( "--size", shape == "point" ? 2000000 : 200000 );
    unsigned int point_size = options.value( "--point-size,--weight", 1u );
    std::string colour = options.exists( "--colour" ) ? options.value< std::string >( "--colour" ) : options.value< std::string >( "-c", "-10:10" );
    std::string label = options.value< std::string >( "--label", "" );
    bool show = true;
    if( properties != "" )
    {
        comma::name_value::parser nameValue( "filename", ';', '=', false );
        csv = nameValue.get( properties, csvOptions );
        comma::name_value::map m( properties, "filename", ';', '=' );
        size = m.value( "size", size );
        point_size = m.value( "point-size", point_size );
        point_size = m.value( "weight", point_size );
        shape = m.value( "shape", shape );
        if( m.exists( "colour" ) ) { colour = m.value( "colour", colour ); }
        else if( m.exists( "color" ) ) { colour = m.value( "color", colour ); }
        label = m.value( "label", label );
        show = !m.exists( "hide" );
    }
    if( !show ) { std::cerr << "view-points: " << csv.filename << " will be hidden on startup; tick the box next to filename to make it visible" << std::endl; }
    snark::graphics::View::coloured* coloured = snark::graphics::View::colourFromString( colour, csv.fields, backgroundcolour );
    if( shape == "point" )
    {
        if( csv.fields == "" ) { csv.fields="x,y,z"; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        bool has_orientation = false;
        for( unsigned int i = 0; !has_orientation && i < v.size(); ++i ) { has_orientation = v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw"; }
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< Eigen::Vector3d >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label ) );
        reader->show( show );
        return reader;
    }
    if( shape == "loop" )
    {
        if( csv.fields == "" ) { csv.fields="x,y,z"; }
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< Eigen::Vector3d, snark::graphics::View::how_t::loop >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label ) );
        reader->show( show );
        return reader;
    }
    if( shape == "lines" ) // todo: get a better name
    {
        if( csv.fields == "" ) { csv.fields="x,y,z"; }
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< Eigen::Vector3d, snark::graphics::View::how_t::connected >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label ) );
        reader->show( show );
        return reader;
    }
    if( shape == "label" )
    {
        if( csv.fields == "" ) { csv.fields="x,y,z,label"; }
        // TODO
    }
    else if( shape == "ellipse" )
    {
        if( csv.fields == "" ) { csv.fields="center,orientation,major,minor"; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i )
        {
            if( v[i] == "x" || v[i] == "y" || v[i] == "z" ) { v[i] = "center/" + v[i]; }
            else if( v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw" ) { v[i] = "orientation/" + v[i]; }
        }
        csv.fields = comma::join( v, ',' );
    }
    else if( shape == "arc" )
    {
        if( csv.fields == "" ) { csv.fields="begin,end"; }
        std::vector< std::string > v = comma::split( csv.fields, ',' );
    }
    else if( shape == "extents" )
    {
        if( csv.fields == "" ) { csv.fields="min,max"; }
    }
    else if( shape == "line" )
    {
        if( csv.fields == "" ) { csv.fields="first,second"; }
    }
    else
    {
        if( csv.fields == "" ) { csv.fields="point,orientation"; csv.full_xpath = true; }
        std::vector< snark::graphics::View::TextureReader::image_options > image_options;
        std::vector< std::string > v = comma::split( shape, ':' );
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            std::vector< std::string > w = comma::split( v[i], ',' );
            std::string e = comma::split( w[0], '.' ).back();
            if( e != "png" && e != "jpg" && e != "jpeg" && e != "bmp" && e != "gif" ) { break; }
            switch( w.size() )
            {
                case 1: image_options.push_back( snark::graphics::View::TextureReader::image_options( w[0] ) ); break;
                case 2: image_options.push_back( snark::graphics::View::TextureReader::image_options( w[0], boost::lexical_cast< double >( w[1] ) ) ); break;
                case 3: image_options.push_back( snark::graphics::View::TextureReader::image_options( w[0], boost::lexical_cast< double >( w[1] ), boost::lexical_cast< double >( w[2] ) ) ); break;
                default: COMMA_THROW( comma::exception, "expected <image>[,<width>,<height>]; got: " << shape );
            }
        }
        if( image_options.empty() )
        {
            model_options m = comma::name_value::parser( ';', '=' ).get< model_options >( properties );
            m.filename = shape;
            if( !boost::filesystem::exists( m.filename ) ) { COMMA_THROW( comma::exception, "file does not exist: " << m.filename ); }
            boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ModelReader( viewer, csv, shape, m.flip, m.scale, coloured, label ) );
            reader->show( show );
            return reader;
        }
        else
        {
            boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::TextureReader( viewer, csv, image_options ) );
            reader->show( show );
            return reader;
        }
    }
    std::vector< std::string > v = comma::split( csv.fields, ',' );
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
    csv.fields = comma::join( v, ',' );
    csv.full_xpath = true;
    if( shape == "extents" )
    {
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< snark::math::closed_interval< double, 3 > >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label, snark::math::closed_interval< double, 3 >( Eigen::Vector3d( 0, 0, 0 ), Eigen::Vector3d( 0, 0, 0 ) ) ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "line" )
    {
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< std::pair< Eigen::Vector3d, Eigen::Vector3d > >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "ellipse" )
    {
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< snark::graphics::View::Ellipse< 25 > >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label ) );
        reader->show( show );
        return reader;
    }
    else if( shape == "arc" )
    {
        snark::graphics::View::arc< 20 > sample; // quick and dirty
        if( csv.has_field( "middle" ) || csv.has_field( "middle/x" ) || csv.has_field( "middle/y" ) || csv.has_field( "middle/z" ) ) { sample.middle = Eigen::Vector3d(); }
        boost::shared_ptr< snark::graphics::View::Reader > reader( new snark::graphics::View::ShapeReader< snark::graphics::View::arc< 20 > >( viewer, snark::graphics::View::Reader::reader_parameters( csv, size, point_size ), coloured, label, sample ) );
        reader->show( show );
        return reader;
    }
    COMMA_THROW( comma::exception, "expected shape, got \"" << shape << "\"" ); // never here
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        if( options.exists( "--help" ) || options.exists( "-h" ) ) { usage(); }
        comma::csv::options csvOptions( argc, argv );
        std::vector< std::string > properties = options.unnamed( "--z-is-up,--orthographic,--flush,--no-stdin,--output-camera-config,--output-camera"
                , "--binary,--bin,-b,--fields,--size,--delimiter,-d,--colour,-c,--point-size,--weight,--background-colour,--scene-center,--center,--scene-radius,--radius,--shape,--label,--camera,--camera-position,--camera-config,--fov,--model,--full-xpath" );
        QColor4ub backgroundcolour( QColor( QString( options.value< std::string >( "--background-colour", "#000000" ).c_str() ) ) );
        boost::optional< comma::csv::options > camera_csv;
        boost::optional< Eigen::Vector3d > cameraposition;
        boost::optional< Eigen::Vector3d > cameraorientation;

        bool camera_orthographic = options.exists( "--orthographic" );
        double fieldOfView = options.value< double >( "--fov" , 45 );
        if( options.exists( "--camera" ) )
        {
            std::string camera = options.value< std::string >( "--camera" );
            std::vector< std::string > v = comma::split( camera, ';' );
            for( std::size_t i = 0; i < v.size(); ++i )
            {
                if( v[i] == "orthographic" )
                {
                    camera_orthographic = true;
                }
                else if( v[i] == "perspective" )
                {
                    camera_orthographic = false;
                }
                else
                {
                    std::vector< std::string > vec = comma::split( v[i], '=' );
                    if( vec.size() == 2 && vec[0] == "fov" )
                    {
                        fieldOfView = boost::lexical_cast< double >( vec[1] );
                    }
                }
            }
        }

        QApplication application( argc, argv );
        bool z_up = options.exists( "--z-is-up" );
        if( options.exists( "--camera-position" ) )
        {
            std::string position = options.value< std::string >( "--camera-position" );
            comma::name_value::parser parser( "x,y,z,roll,pitch,yaw", ',', '=', false );
            snark::graphics::View::point_with_orientation pose;
            try
            {
                pose = parser.get< snark::graphics::View::point_with_orientation >( position );
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
                }
                catch( ... ) {}
            }
        }
        boost::optional< double > scene_radius = options.optional< double >( "--scene-radius,--radius" );
        boost::optional< Eigen::Vector3d > scene_center;
        boost::optional< std::string > s = options.optional< std::string >( "--scene-center,--center" );
        if( s ) { scene_center = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", ',' ).get( *s ); }
        boost::property_tree::ptree camera_config; // quick and dirty
        if( options.exists( "--camera-config" ) ) { boost::property_tree::read_json( options.value< std::string >( "--camera-config" ), camera_config ); }
        snark::graphics::View::Viewer* viewer = new snark::graphics::View::Viewer( backgroundcolour
                                                                                 , fieldOfView
                                                                                 , z_up
                                                                                 , camera_orthographic
                                                                                 , camera_csv
                                                                                 , cameraposition
                                                                                 , cameraorientation
                                                                                 , options.exists( "--camera-config" ) ? &camera_config : NULL
                                                                                 , scene_center
                                                                                 , scene_radius
                                                                                 , options.exists( "--output-camera-config,--output-camera" ) );
        bool stdinAdded = false;
        for( unsigned int i = 0; i < properties.size(); ++i )
        {
            if( comma::split( properties[i], ';' )[0] == "-" ) { stdinAdded = true; }
            viewer->readers.push_back( makeReader( *viewer, options, csvOptions, properties[i] ) );
        }
        if( !stdinAdded && !options.exists( "--no-stdin" ) )
        {
            csvOptions.filename = "-";
            viewer->readers.push_back( makeReader( *viewer, options, csvOptions ) );
        }
        snark::graphics::View::MainWindow mainWindow( comma::join( argv, argc, ' ' ), viewer );
        mainWindow.show();
        /*return*/ application.exec();
        delete viewer;
    }
    catch( std::exception& ex )
    {
        std::cerr << "view-points: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "view-points: unknown exception" << std::endl;
    }
}
