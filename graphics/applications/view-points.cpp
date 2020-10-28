// Copyright (c) 2011 The University of Sydney

#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <QApplication>
#if Qt3D_VERSION==1
#include "view_points/qt3d_v1/viewer.h"
#include "view_points/qt3d_v1/texture_reader.h"
#else
#include "view_points/controller.h"
#include "view_points/image_reader.h"
#endif
#include "view_points/model_reader.h"
#if Qt3D_VERSION>=2
#include "view_points/traits.h"
#include "../qt5.5/qopengl/labels.h"
#endif
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include "../../visiting/eigen.h"
#include "../qt3d/camera_options.h"
#include "view_points/click_mode.h"
#include "view_points/shape_reader.h"
#include "view_points/main_window.h"
#include "view_points/types.h"

static void bash_completion( unsigned const ac, char const * const * av )
{
    static const char * completion_options =
        " --help -h"
        " --version"
        " --colour --color -c"
//         " --exit-on-end-of-input"
        " --click-mode"
        " --fill"
        " --font-size"
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
        " --background-colour --background-color"
        " --output-camera-config --output-camera"
        " --scene-center --center"
        " --scene-radius --radius"
        " --title"
        " --z-is-up";

    std::cout << completion_options << std::endl;
    exit( 0 );
}

static void usage( bool )
{
#if Qt3D_VERSION>=2
    static const char * const usage_qt55_warning =
        "\nWARNING: this version of view-points is compiled against Qt5.5+"
        "\n         it is not be fully functional (yet)"
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
    #define qtold_unsupported_marker_start
    #define qtold_unsupported_marker_end
#else
    #define qt55_unsupported_marker_start ""
    #define qt55_unsupported_marker_end ""
    #define qtold_unsupported_marker_start "\x1B[0;90m"
    #define qtold_unsupported_marker_end "\x1B[0m"
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
        qtold_unsupported_marker_start
        "\n    --click-mode=<mode>[;<options>]; click mode, currently only double right click behaviour is configurable"
        "\n        <mode>: mode; default: none; choices: 'block', 'label'"
        "\n        <options>: semicolon-separated options"
        "\n            labels=<choices>: comma-separated label values, first element will be the initial label value; default: any"
        "\n            blocks=<choices>: comma-separated block values, first element will be the initial block value; default: any"
        "\n        example: --click-mode='label;labels:hello,world'"
        qtold_unsupported_marker_end
        "\n    --colour,--color,-c <how>: how to colour points"
        "\n        <how>:"
        "\n            colour maps"
        "\n                <min>:<max>: if scalar field present, stretch by scalar (see section <fields>)"
        "\n                <min>:<max>,<from-colour>:<to-colour>: if scalar field present, stretch by scalar (see section <fields>)"
        "\n                <min>:<max>,<colourmap>: if scalar field present, stretch by scalar (see section <fields>)"
        "\n                    <colourmap>: jet, hot, red, green"
        "\n"
        "\n            fixed colour"
        "\n                fixed colour: white, black, red, green, blue, yellow, cyan, magenta, grey, pink, sky, salad"
        "\n                              or color as <r>,<g>,<b>[,<alpha>], e.g: --color 255,0,0 for red"
        "\n"
        "\n            colour by id"
        "\n                - if there is \"id\" field in --fields, colour by id, with or without scalar (see section <fields>)"
        "\n                - or specify cyclic colour wheel explicitly (see fixed colour above), e.g: if --color=red,green,blue"
        "\n                  e.g: if --color=red,green,blue, then 0 id will be red, 1 green, 2 blue, 3 red again"
        "\n                  you can specify colours as RGB, too, e.g: --color=red,255|255|0,blue"
        "\n                - or specify color map explicitly: e.g. --color=0:red,5:blue,default:grey"
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
        "\n    --exit-on-end-of-input: exit immediately on end of input stream"
        "\n    --fill: fill the shape; currently implemented only for triangles"
        "\n    --font-size=[<font-size>]: label font size; default: 16; also can be used with individual streams, e.g:"
        "\n          e.g: view-points <( echo 0,0,hello )\";fields=x,y,label\" <( echo 1,1,world )\";fields=x,y,label;font-size=32\" --scene-radius 2"
        "\n    --label=<label>: text label displayed next to the latest point"
        "\n    --no-stdin: do not read from stdin"
        "\n    --pass-through,--pass; pass input data to stdout"
        "\n    --point-size,--weight <point size>: default: 1"
        "\n    --shape <shape>: \"point\", \"extents\", \"line\", \"label\"; default \"point\""
        "\n                     \"arc\": e.g: --shape=arc --fields=,,begin,end,centre,"
        "\n                               or: --shape=arc --fields=,,begin,middle,end,,,"
        "\n                                   where 'begin' and 'end' are x,y,z points"
        "\n                                   and 'middle' is a point between begin and end on the arc"
        "\n                                   default: 'begin,end', with centre 0,0,0"
        "\n                     \"axis\": draws three axis lines per record, in red/green/blue corresponding to x/y/z axes, using position and orientation e.g. --shape=axis --fields=position,orientation"
        "\n                               default fields: position,orientation"
        "\n                               fields: position: x,y,z or position/x,position/y,position/z"
        "\n                                       orientation: roll,pitch,yaw or orientation/roll,orientation/pitch,orientation/yaw"
        "\n                               options: options can be specified as command line option and/or in each stream (without -- prefix)"
        "\n                                        --length=<d>: length of each axis line"
        "\n                                        --labels=\"<x>:<y>:<z>\" colon separated list of axis labels, leave empty for no labels."
#if Qt3D_VERSION>=2
        "\n                                        --weight or --point-size can be used for line thickness"
#endif
        "\n                                        by default each axis is painted with different colors x:red y:green z:blue; unless"
        "\n                                        if --color option exists or any of id or scalar fields are present it will use the normal coloring for all axis lines"
        "\n                     \"ellipse\": e.g. --shape=ellipse --fields=,,center,orientation,minor,major,"
        "\n                                  default fields:center/x,center/y,center/z,roll,pitch,yaw,minor,major"
        "\n                                  orientation: roll,pitch,yaw; default: in x,y plane"
        "\n                     \"extents\": e.g. --shape=extents --fields=,,min,max,,,"
        "\n                                  default fields: min,max"
        qt55_unsupported_marker_start
        "\n                     \"label\": e.g. --shape=label --fields=,x,y,z,,,label"
        "\n                                default fields: x,y,z,label"
        qt55_unsupported_marker_end
        "\n                     \"line\": e.g. --shape=line --fields=,,first,second,,,"
        "\n                               default fields: first,second"
        "\n                     \"lines\": connect all points of a block from first to the last; fields same as for 'point'"
        "\n                                default fields: x,y,z"
        "\n                     \"loop\": connect all points of a block; fields same as for 'point'"
        "\n                               default fields: x,y,z"
        "\n                     \"triangle\": e.g. --shape=triangle --fields=,,corners,,,"
        "\n                                   or --shape=triangle --fields=,,corners[0],,,corners[1],,,corners[2],,,"
        "\n                                   or --shape=triangle --fields=,,corners[0]/x,,corners[0]/y,,corners[0]/z,,,,corners[1],,,corners[2],,, etc"
        "\n                                   default fields: corners"
        qt55_unsupported_marker_start
        "\n                     \"model file ( obj, ply... )>[;<options>]\": e.g. --shape=vehicle.obj"
        "\n                          <options>"
        "\n                              flip\": flip the model around the x-axis"
        "\n                              scale=<value>\": resize model (ply only, todo), e.g. show model half-size: scale=0.5"
        qt55_unsupported_marker_end
        "\n                     \"<image file>[,<image options>]:<image file>[,<image options>]\": show image, e.g. --shape=\"vehicle-lights-on.jpg:vehicle-lights-off.jpg\""
        "\n                            <image file>: e.g. either: images/cat.png"
        "\n                                                   or: images/*.png; images can be displayed by id (see note 2 for the id field description"
        "\n                            <image options>: <width>,<height> or <pixel-size>"
        "\n                                <width>,<height>: image width and height in meters when displaying images in the scene; default: 1,1"
        "\n                                <pixel size>: single pixel size in metres"
        "\n                            note 1: just like for the cad models, the images will be pinned to the latest point in the stream"
        "\n                            note 2: specify id in fields to switch between multiple images, see examples below"
        "\n    --size <size>: render last <size> points (or other shapes)"
        "\n                   default 2000000 for points, for 10000 for other shapes"
        "\n    --groups <csv>: include in group/s named in csv"
        "\n    --title <title>: title for source, defaults to filename"
        "\n                     if set to \"none\" don't show source in selection box"
        "\n                     (but still display data and checkbox)"
        "\n    --version: print Qt version and exit"
        "\n"
        "\ncamera options"
        "\n    --camera=\"<options>\""
        "\n          <options>: [fov=<fov>];[<type>]"
        "\n          <fov>: field of view in degrees, default 45 degrees"
        "\n          <type>: orthographic | perspective"
        "\n              default: perspective"
        "\n    --fov=<fov>: set camera field of view in degrees"
        "\n    --camera-config=<filename>: camera config in json; to see an example, run --output-camera-config"
        qt55_unsupported_marker_start
        "\n    --camera-position=\"<options>\": todo: fix: broken for qt5.5 and higher"
        "\n          <options>: <position>|<stream>"
        "\n          <position>: <x>,<y>,<z>,<roll>,<pitch>,<yaw>"
        "\n          <stream>: position csv stream with options; default fields: x,y,z,roll,pitch,yaw"
        qt55_unsupported_marker_end
        "\n    --orthographic: use orthographic projection instead of perspective"
        "\n"
        "\nmore options"
        "\n    --background-colour,--background-color=<colour> : default: black"
        "\n    --show-maximized,--maximize; start view-points in full-screen"
        "\n    --output-camera-config,--output-camera: output camera config to stdout as stream of json structures"
        "\n    --scene-center,--center=<value>: fixed scene center as \"x,y,z\""
        "\n    --scene-radius,--radius=<value>: fixed scene radius in metres, since sometimes it is hard to imply"
        "\n                                     scene size from the dataset (e.g. for streams)"
        qt55_unsupported_marker_start
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
        "\nkey presses and mouse clicks:"
        "\n    general controls"
        "\n        left press and hold: rotate the scene around the centre"
        "\n        right press and hold: translate the scene"
        "\n        double left click: change the centre of the scene"
        "\n        scroll wheel: zoom"
        "\n"
        qtold_unsupported_marker_start
        "\n    console/stdout output"
        "\n        click modes"
        "\n            default"
        "\n                double right click: output to stdout approximate coordinates of the clicked point as x,y,z"
        "\n            block"
        "\n                control + b: toggle block mode; if on, double right click always appends current block id"
        "\n                double right click: output to stdout approximate coordinates of the clicked point as x,y,z,block"
        "\n                if block choices not given in --click-mode"
        "\n                    ctrl + shift + '+' or up-arrow key: increment block id"
        "\n                    ctrl + '-' or down-arrow key: decrement block id"
        "\n                    ctrl + 'c': reset block id"
        "\n                    '1' - '9': append digit to block id"
        "\n                    '-': toggle the sign of block id"
        "\n                    backspace: delete last digit"
        "\n                if block choices given in --click-mode"
        "\n                    arrow up: move to the next block value"
        "\n                    arrow down: move to the previous block value"
        "\n            label"
        "\n                control + l: toggle label mode; if on, double right click always appends current label"
        "\n                double right click: output to stdout approximate coordinates of the clicked point as x,y,z,label"
        "\n                if label choices not given in --click-mode"
        "\n                    ctrl + 'c': reset label to empty string"
        "\n                    alpha-numeric and punctuation keys: append character to label"
        "\n                    backspace: delete last charact"
        "\n                if label choices given in --click-mode"
        "\n                    arrow up: move to the next label value"
        "\n                    arrow down: move to the previous label value"
        qtold_unsupported_marker_end
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
        "\n    view multiple files with titles"
        "\n        view-points \"raw.csv;colour=0:20;title=raw\" \"partitioned.csv;fields=x,y,z,id;point-size=2;title=partitioned\""
        "\n"
        qt55_unsupported_marker_start
        "\n    view a cad model"
        "\n        echo \"0,0,0\" | view-points --shape /usr/local/etc/segway.shrimp.obj --z-is-up --orthographic"
        qt55_unsupported_marker_end
        "\n"
        "\n    use stdin as the primary source for scene radius:"
        "\n        cat xyz.csv | view-points \"-\" scan.csv"
        "\n"
        "\n    specify fixed scene radius explicitly:"
        "\n        cat xyz.csv | view-points --scene-radius=100"
        "\n"
        "\n    passing input data through:"
        "\n        cat xyz.csv | view-points \"-;pass-through\" scan.csv"
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
        "\n"
        "\n    an example of many of the supported shapes"
        "\n        for i in {0..15}; do echo \"a=2*3.1415926532/16*$i;s(a)*3;c(a)*3;s(a)*3\" | bc -l | paste -s -d,; done \\"
        "\n            | view-points \"-;weight=5;color=cyan;label=point\" \\"
        "\n                 <( echo 0,0,0,1,1,1 )\";shape=extents;label=extents;color=blue\" \\"
        "\n                 <( echo 0,0,2,0,0,0,0.5,2 )\";shape=ellipse;label=ellipse;color=salad\" \\"
        "\n                 <( echo -e \"0,0,-2,0\\n0,1,-2,1\\n0.5,1.5,-2,2\\n1,1,-2,3\\n1,0,-2,4\\n0.5,-0.5,-2,5\" )\";shape=loop;fields=x,y,z,id;label=loop\" \\"
        "\n                 <( echo 2,2,-1,-2,-1,-1 )\";shape=arc;label=arc;color=magenta\"\\"
        "\n                 <( echo '-3,-3,-3,0,0,0' )\";shape=axis;fields=position,orientation;length=6;labels=X:Y:Z;label=axis\"\\"
        "\n                 <( echo 0,0,3,0,1,3; echo 1,0,3,1,1,3; echo 2,0,3,2,1,3)\";shape=line;fields=first,second;label=line;color=red\"\\"
        "\n                 <( echo 2,0,4; echo 2,1,4; echo 3,1,4; echo 3,0,4)\";shape=lines;label=lines;color=green\"\\"
        "\n                 <( echo -1,-1,5,0,-2,5,1,-1,5 )\";shape=triangle;label=triangle;color=yellow\""
        "\n"
        qtold_unsupported_marker_start
        "\n    double right click modes"
        "\n        > mkfifo pipe"
        "\n        > cat pipe | view-points <( csv-random make --type 2f | head -n10000 )';fields=x,y;color=yellow' \\"
        "\n                                 '-;fields=x,y,z,label;weight=10;color=red' --click-mode='label;labels=hello,world,moon' | tee pipe"
        "\n        in view-points window"
        "\n            - double right click on a couple of points; observe output on stdout and large red dots"
        "\n              with the label 'hello' appear where you clicked"
        "\n            - use up/down arrows to select labels"
        qtold_unsupported_marker_end
        "\n";

    std::cerr
        #if Qt3D_VERSION>=2
        << usage_qt55_warning
        #endif
        << usage_synopsis
        << usage_options
        << "\ncsv options\n"
        << comma::csv::options::usage()
        << usage_csv_options
        << usage_examples
        #if Qt3D_VERSION>=2
        << usage_qt55_warning
        #endif
        << std::endl;
    exit( 1 );
}

template < typename Options >
static std::vector< Options > make_image_options( const std::string& shape )
{
    std::vector< Options > image_options;
    const std::vector< std::string >& v = comma::split( shape, ':' );
    for( unsigned int i = 0; i < v.size(); ++i )
    {
        const std::vector< std::string >& w = comma::split( v[i], ',' );
        if( w.empty() || w[0].empty() ) { COMMA_THROW( comma::exception, "got empty image options in '" << shape << "'" ); }
        boost::optional< double > p1;
        boost::optional< double > p2;
        switch( w.size() ) // quick and dirty
        {
            case 1: break;
            case 2: p1 = boost::lexical_cast< double >( w[1] ); break;
            case 3: p1 = boost::lexical_cast< double >( w[1] ); p2 = boost::lexical_cast< double >( w[2] ); break;
            default: COMMA_THROW( comma::exception, "expected <image>[,<width>,<height>]; got: '" << shape << "'" );
        }
        boost::filesystem::path path( w[0] );
        std::string e = path.extension().string();
        if( e != ".png" && e != ".jpg" && e != ".jpeg" && e != ".bmp" && e != ".gif" ) { break; } // quick and dirty
        std::vector< std::string > filenames;
        if( path.leaf().string() == ( "*" + e ) ) // quick and dirty; proper regex: todo
        {
            std::string dir = path.branch_path().string();
            if( dir.empty() ) { dir = "."; }
            for( boost::filesystem::directory_iterator it( dir ); it != boost::filesystem::directory_iterator(); ++it ) { if( it->path().extension() == e ) { filenames.push_back( it->path().string() ); } }
            if( filenames.empty() ) { COMMA_THROW( comma::exception, "no " << e << " files found in " << dir ); }
        }
        else
        {
            filenames.push_back( w[0] );
        }
        std::sort( filenames.begin(), filenames.end() ); // quick and dirty
        for( const auto& filename: filenames )
        {
            switch( w.size() )
            {
                case 1: image_options.push_back( Options( filename ) ); break;
                case 2: image_options.push_back( Options( filename, *p1 ) ); break;
                case 3: image_options.push_back( Options( filename, *p1, *p2 ) ); break;
                default: break; // never here
            }
        }
    }
    return image_options;
}

static bool data_passed_through = false;

// quick and dirty, todo: a proper structure, as well as a visitor for command line options
std::unique_ptr< snark::graphics::view::Reader > make_reader( const comma::command_line_options& options
                                                            , const comma::csv::options& csv_options
                                                            , const std::string& properties = "" )
{
    //snark::graphics::view::color_t
    QColor background_color( QColor( QString( options.value< std::string >( "--background-colour", "#000000" ).c_str() ) ) );
    std::string color = options.value( "--color,--colour,-c", std::string() );
    std::string label = options.value( "--label", std::string() );
    std::string shape = options.value< std::string >( "--shape", "point" );
    bool show = true;
    snark::graphics::view::Reader::reader_parameters param( csv_options
                                                          , options.value( "--title", csv_options.filename )
                                                          , options.value( "--groups", std::string() )
                                                          , options.value( "--size", 0 ) // will be sorted out below based on shape
                                                          , options.value( "--point-size,--weight", 1u )
                                                          , options.exists( "--pass-through,--pass" )
                                                          , options.exists( "--fill" )
                                                          , options.value( "--labels", std::string() )
                                                          , options.value( "--length", 1. )
                                                          , options.exists( "--colour,--color,-c" )
                                                          , options.value< unsigned int>( "--font-size", 16 ) );
    if( !properties.empty() )
    {
        comma::name_value::parser nameValue( "filename", ';', '=', false );
        param.options = nameValue.get( properties, csv_options );
        param.options.full_xpath = false;
        comma::name_value::map m( properties, "filename", ';', '=' );
        shape = m.value( "shape", shape );
        param.size = m.value( "size", param.size );
        param.point_size = m.value( "point-size", param.point_size );
        param.point_size = m.value( "weight", param.point_size );
        param.title = m.value( "title", param.title.empty() ? param.options.filename : param.title );
        param.groups = m.value( "groups", param.groups );
        if( m.exists( "colour" ) ) { color = m.value( "colour", color ); param.has_color=true; }
        else if( m.exists( "color" ) ) { color = m.value( "color", color ); param.has_color=true; }
        label = m.value( "label", label );
        show = !m.exists( "hide" );
        param.pass_through = param.pass_through || ( m.exists( "pass-through" ) || m.exists( "pass" ));
        param.fill = param.fill || m.exists( "fill" );
        param.labels=m.value("labels",param.labels);
        param.length=m.value("length",param.length);
        if( param.options.has_field( "id,scalar" ) ) { param.has_color = true; }
        param.font_size=m.value("font-size",param.font_size);
    }
    if( param.pass_through )
    {
        if( data_passed_through ) { COMMA_THROW( comma::exception, "only one input stream can be given \"pass-through\" option" ); }
        data_passed_through = true;
    }
    if( param.size == 0 ) { param.size = shape == "point" ? 2000000 : 10000; }
    if( param.title == "none" ) { param.title = ""; }
    if( !show ) { std::cerr << "view-points: " << ( param.title.empty() ? param.options.filename : param.title )<< " will be hidden on startup; tick the box next to the name to make it visible" << std::endl; }
    snark::graphics::view::colored* colored = snark::graphics::view::color_from_string( color, param.options.fields, background_color );
    if( shape == "point" )
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z"; }
        std::vector< std::string > v = comma::split( param.options.fields, ',' );
        bool has_orientation = false;
        for( unsigned int i = 0; !has_orientation && i < v.size(); ++i ) { has_orientation = v[i] == "roll" || v[i] == "pitch" || v[i] == "yaw"; }
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< Eigen::Vector3d >( param, colored, label, Eigen::Vector3d::Zero() ) );
        reader->show( show );
        return reader;
    }
    if( shape == "loop" )
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z"; }
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< Eigen::Vector3d, snark::graphics::view::how_t::loop >( param, colored, label, Eigen::Vector3d::Zero() ) );
        reader->show( show );
        return reader;
    }
    if( shape == "lines" ) // todo: get a better name
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z"; }
        std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::ShapeReader< Eigen::Vector3d, snark::graphics::view::how_t::lines >( param, colored, label, Eigen::Vector3d::Zero() ) );
        reader->show( show );
        return reader;
    }
    if( shape == "label" )
    {
        if( param.options.fields == "" ) { param.options.fields="x,y,z,label"; }
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
    else if( shape=="axis" )
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
        auto image_options = make_image_options< snark::graphics::view::texture_reader::image_options >( shape );
#elif Qt3D_VERSION>=2
        auto image_options = make_image_options< snark::graphics::view::image_options >( shape );
#endif
        if( image_options.empty() ) // quick and dirty
        {
            snark::graphics::view::model_options m = comma::name_value::parser( ';', '=' ).get< snark::graphics::view::model_options >( properties );
            m.filename = shape;
            if( !boost::filesystem::exists( m.filename ) ) { COMMA_THROW( comma::exception, "file does not exist: " << m.filename ); }
            std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::model_reader( param, m, colored, label ) );
            reader->show( show );
#if Qt3D_VERSION>=2
            std::cerr << "view-points: cad models are not supported yet for qt 5.5+ " << Qt3D_VERSION << " yet; todo" << std::endl;
            exit(1);
#endif
            return reader;
        }
        else
        {
#if Qt3D_VERSION==1
            std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::texture_reader( param, image_options ) );
#elif Qt3D_VERSION>=2
            std::unique_ptr< snark::graphics::view::Reader > reader( new snark::graphics::view::image_reader( param, image_options ) );
#endif
            reader->show( show );
            return reader;
        }
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
    else if( shape == "axis" )
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
        comma::command_line_options options( argc, argv, usage );
        if( options.exists( "--bash-completion" ) ) bash_completion( argc, argv );
        if( options.exists( "--version" ) ) { version(); exit( 0 ); }
        comma::csv::options csv_options( argc, argv, "", false );
        std::vector< std::string > properties = options.unnamed( "--show-maximized,--maximize,--z-is-up,--orthographic,--flush,--no-stdin,--output-camera-config,--output-camera,--pass-through,--pass,--exit-on-end-of-input,--fill", "-[^;].*" );
        snark::graphics::view::color_t  background_color( QColor( QString( options.value< std::string >( "--background-colour,--background-color", "#000000" ).c_str() ) ) );
        boost::optional< comma::csv::options > camera_csv;
        boost::optional< Eigen::Vector3d > camera_position;
        boost::optional< Eigen::Vector3d > camera_orientation;
        snark::graphics::qt3d::camera_options camera_options( options.exists( "--orthographic" ), options.value< double >( "--fov", 45.0 ), options.exists( "--z-is-up" ) );
        if( options.exists( "--camera" ) )
        {
            std::string camera = options.value< std::string >( "--camera" );
            std::vector< std::string > camera_fields = comma::split( camera, ';' );
            for( const auto& field : camera_fields )
            {
                if( field == "orthographic" )
                {
                    camera_options.orthographic = true;
                }
                else if( field == "perspective" )
                {
                    camera_options.orthographic = false;
                }
                else
                {
                    std::vector< std::string > vec = comma::split( field, '=' );
                    if( vec.size() == 2 && vec[0] == "fov" ) { camera_options.field_of_view = boost::lexical_cast< double >( vec[1] ); }
                }
            }
        }
        bool camera_position_from_stdin = false;
        QApplication application( argc, argv );
        if( options.exists( "--camera-position" ) )
        {
            std::string position = options.value< std::string >( "--camera-position" );
            comma::name_value::parser camera_position_parser( "x,y,z,roll,pitch,yaw", ',', '=', false );
            snark::graphics::view::point_with_orientation pose;
            try
            {
                pose = camera_position_parser.get< snark::graphics::view::point_with_orientation >( position );
                camera_position = pose.point;
                camera_orientation = pose.orientation;
            }
            catch( ... ) {}
            if( !camera_position )
            {
                comma::name_value::parser file_parser( "filename", ';', '=', false );
                try
                {
                    std::cerr << " parse " << position << std::endl;
                    camera_csv = file_parser.get< comma::csv::options >( position );
                    camera_csv->full_xpath = false;
                    if( camera_csv->fields.empty() ) { camera_csv->fields = "x,y,z,roll,pitch,yaw"; }
                    camera_position_from_stdin = camera_csv->filename == "-";
                }
                catch( ... ) {}
            }
        }
        #if Qt3D_VERSION==1
        auto scene_radius = options.optional< double >( "--scene-radius,--radius" ); // todo: do something with the magic default
        boost::optional< Eigen::Vector3d > scene_center;
        if( options.exists( "--scene-center,--center") ) { scene_center = comma::csv::ascii< Eigen::Vector3d >( "x,y,z", ',').get( options.value< std::string >( "--scene-center,--center" ) ); }
        std::shared_ptr< snark::graphics::view::Viewer > controller( new snark::graphics::view::Viewer( background_color
                                                                                                      , camera_options
                                                                                                      , options.exists( "--exit-on-end-of-input" )
                                                                                                      , camera_csv
                                                                                                      , camera_position
                                                                                                      , camera_orientation
                                                                                                      , options.value< std::string >( "--camera-config", "" )
                                                                                                      , scene_center
                                                                                                      , scene_radius
                                                                                                      , options.exists( "--output-camera-config,--output-camera" ) ) );
        #elif Qt3D_VERSION>=2
        double scene_radius = options.value( "--scene-radius,--radius", 10. );
        QVector3D scene_center = comma::csv::ascii< QVector3D >( "x,y,z", ',' ).get( options.value< std::string >( "--scene-center,--center", "0,0,0" ) );
        std::shared_ptr< snark::graphics::view::controller > controller( new snark::graphics::view::controller( background_color
                                                                                                              , camera_options
                                                                                                              , options.exists( "--exit-on-end-of-input" )
                                                                                                              , camera_csv
                                                                                                              , camera_position
                                                                                                              , camera_orientation
                                                                                                              , options.value< std::string >( "--camera-config", "" )
                                                                                                              , scene_center
                                                                                                              , scene_radius
                                                                                                              , options.exists( "--output-camera-config,--output-camera" )
                                                                                                              , snark::graphics::view::click_mode( options.value< std::string >( "--click-mode", "none" ) ) ) );
        controller->viewer->scene_radius_fixed = options.exists( "--scene-radius,--radius" );
        controller->viewer->scene_center_fixed = options.exists( "--scene-center,--center" );
        #endif
        bool stdin_explicitly_defined = false;
        for( const auto& property : properties )
        {
            if( comma::split( property, ';' )[0] == "-" ) { stdin_explicitly_defined = true; }
            controller->add( make_reader( options, csv_options, property ) );
        }
        if( !stdin_explicitly_defined && !options.exists( "--no-stdin" ) && !camera_position_from_stdin )
        {
            csv_options.filename = "-";
            controller->add( make_reader( options, csv_options ) );
        }
        if( data_passed_through )
        {
            controller->inhibit_stdout();
            if( options.exists( "--output-camera-config,--output-camera" ) ) { COMMA_THROW( comma::exception, "cannot use --output-camera-config whilst \"pass-through\" option is in use" ); }
        }
        snark::graphics::view::MainWindow main_window( comma::join( argv, argc, ' ' ), controller );
        options.exists( "--show-maximized,--maximize" ) ? main_window.showMaximized() : main_window.show();
        QApplication::exec();
        return 0;       // We never actually reach this line because we raise SIGINT when closing

    }
    catch( std::exception& ex ) { std::cerr << "view-points: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "view-points: unknown exception" << std::endl; }
    return 1;
}
