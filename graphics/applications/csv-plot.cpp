// Copyright (c) 2021 Vsevolod Vlaskine

// todo! add QtCharts licence

/// @author Vsevolod Vlaskine

#include <iostream>
#include <map>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>
#include <QtWidgets/QApplication>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/string/string.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include "csv_plot/main_window.h"
#include "csv_plot/traits.h"

static void usage( bool verbose = false )
{
    std::cerr << "plot points from csv files or streams" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat xy.csv | csv-plot [<sources>] [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "sources: input sources as <name>[;<options>]" << std::endl;
    std::cerr << "    <name>: e.g. points.csv, tcp:localhost:12345, etc" << std::endl;
    std::cerr << "    <options>: csv options, stream, and series-specific options (see below)" << std::endl;
    std::cerr << "               e.g: csv-plot \"points.bin;fields=x,y;binary=2d;color=red;weight=2\"" << std::endl;
    std::cerr << "               an option in form --xxx=<n> applies to all streams or series" << std::endl;
    std::cerr << "               or it can be defined per stream or series as 'data.csv;xxx=<n>'" << std::endl;
    std::cerr << "               e.g: csv-plot 'a.csv;fields=,,x,,y;color=red' 'b.csv' --color=blue" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help, --help --verbose: more help" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;
    std::cerr << "stream options" << std::endl;
    std::cerr << "    --input-fields; print possible input fields to stdout and exit" << std::endl;
    std::cerr << "    --input-fields-example; print input fields example to stdout and exit" << std::endl;
    std::cerr << "    --fields: t,series,block where series is an array; also see --number-of-series" << std::endl;
    std::cerr << "              x,y: aliases for series[0]/x and series[0]/y" << std::endl;
    std::cerr << "              series fields: x,y,z" << std::endl;
    std::cerr << "              default: x,y" << std::endl;
    std::cerr << "              if x is not present in series n, x will be same as in series 0" << std::endl;
    std::cerr << "              examples" << std::endl;
    std::cerr << "                  --fields=series[0]/x,series[0]/y,series[1]/x,series[1]/y" << std::endl;
    std::cerr << "                  --fields=series[0],series[1]: means same as above" << std::endl;
    std::cerr << "                  --fields=x,y,series[1]: means same as above" << std::endl;
    std::cerr << "                  --fields=series[0],series[1]/y,series[2]/y: series 1 and 2 take x from series 0" << std::endl;
    std::cerr << "    --no-stdin: don't try to read from stdin" << std::endl;
    std::cerr << "    --number-of-series,-n=<n>; default=1; how many series each stream has; a convenience option" << std::endl;
    std::cerr << "                               if --fields have 'series' field without series indices" << std::endl;
    std::cerr << "    --pass-through,--pass; todo: output to stdout the first stream on the command line" << std::endl;
    std::cerr << "    --size,-s,--tail=<n>: plot last <n> records of stream; default 10000" << std::endl;
    std::cerr << std::endl;
    std::cerr << std::endl << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << std::endl;
    std::cerr << "chart options" << std::endl;
    std::cerr << "    --chart=<properties>; todo: semicolon-separated chart properties; multiple --chart options allowed" << std::endl;
    std::cerr << "        <properties>" << std::endl;
    std::cerr << "            animate: todo" << std::endl;
    std::cerr << "            legend: todo" << std::endl;
    std::cerr << "            max: todo" << std::endl;
    std::cerr << "            min: todo" << std::endl;
    std::cerr << "            name: todo" << std::endl;
    std::cerr << "            range: todo?" << std::endl;
    std::cerr << "            scroll: todo" << std::endl;
    std::cerr << "            title: todo" << std::endl;
    std::cerr << "    --scroll: if present, chart axes get adjusted to where the data is" << std::endl;
    std::cerr << "    --title=[<title>]: chart title" << std::endl;
    std::cerr << std::endl;
    std::cerr << "series options" << std::endl;
    std::cerr << "    --color=<color>: plot color: black, white, red, green, blue" << std::endl;
    std::cerr << "                                 yellow, cyan, magenta, grey" << std::endl;
    std::cerr << "                                 or #rrggbb, e.g. #ff00ff" << std::endl;
    std::cerr << "    --shape=<what>: line (default)" << std::endl;
    std::cerr << "                    todo: more shapes" << std::endl;
    std::cerr << "    --weight=<weight>: point or line weight" << std::endl;
    std::cerr << std::endl;
    std::cerr << "    for multiple series per stream, individual series options look like: 'series[2]=color:green|chart:test'" << std::endl;
    std::cerr << "    i.e. options are |-separated <name>:<value> pairs, for example, in the following command line:" << std::endl;
    std::cerr << "        csv-plot '-;fields=series;number-of-series=4;color=red;chart=one;series[1]=color:blue;series[2]=color:green|chart:two'" << std::endl;
    std::cerr << "        - there are 4 series on stdin with fields: series[0]/x,series[0]/y,series[1]/x,series[1]/y,series[2]/x,series[2]/y,series[3]/x,series[3]/y" << std::endl;
    std::cerr << "        - series 2 will be shown in chart 'two'; series 0, 1, and 3 in chart 'one'" << std::endl;
    std::cerr << "        - series 1 will be blue; series 2 green; series 0 and 3 red" << std::endl;
    std::cerr << std::endl;
    std::cerr << "window options" << std::endl;
    std::cerr << "    --frames-per-second,--fps=<value>; default=10; how often to update chart(s)" << std::endl;
    std::cerr << "    --full-screen,--maximize: todo: initially, create full screen windows" << std::endl;
    std::cerr << "    --layout=<layout>; default=grid; layouts for multiple charts" << std::endl;
    std::cerr << "        <layout>" << std::endl;
    std::cerr << "            grid[;<shape>]: charts are arranged in single window as grid" << std::endl;
    std::cerr << "                <shape>: default: 1" << std::endl;
    std::cerr << "                    cols=<cols>: grid with <cols> columns (rows calculated from number of charts)" << std::endl;
    std::cerr << "                    rows=<rows>: grid with <rows> rows (columns calculated from number of charts)" << std::endl;
    std::cerr << "                    default: cols=1" << std::endl;
    std::cerr << "                stacked: todo: each chart is in its own window" << std::endl;
    std::cerr << "                tabs: charts are arranged in single window as tabs" << std::endl;
    std::cerr << "                windows: todo: each chart is in its own window" << std::endl;
    std::cerr << "    --timeout=<seconds>; how often to update, overrides --fps" << std::endl;
    std::cerr << "    --window-size=<x>,<y>: initial window size; default=800,600" << std::endl;
    std::cerr << std::endl;
    if( verbose )
    {
        std::cerr << "examples" << std::endl;
        std::cerr << "    plot points on stdin" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot --color=red" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot --color=red --weight=5" << std::endl;
        //std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 | csv-plot --color=red --weight=5 --style=dots" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    use point number as x" << std::endl;
        std::cerr << "        echo -e 10\\\\n5\\\\n8 | csv-plot --fields=y" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    use point number as x" << std::endl;
        std::cerr << "        echo -e 10\\\\n5\\\\n8 | csv-plot --fields y" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    several plots at once" << std::endl;
        std::cerr << "        echo -e 1,10\\\\n2,5\\\\n3,8 > test.csv" << std::endl;
        std::cerr << "        csv-plot \"test.csv;color=red\" \"test.csv;color=blue;fields=y,x\"" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    point streams: show last 100 points" << std::endl;
        std::cerr << "        netcat localhost 12345 | csv-plot --size=100" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    several streams" << std::endl;
        std::cerr << "        csv-plot \"tcp:localhost:8888\" \"tcp:localhost:9999\"" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    plot block by block" << std::endl;
        std::cerr << "        netcat localhost 12345 | csv-plot --fields=x,y,block" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    xy chart with multiple inputs block by block with different shapes" << std::endl;
        std::cerr << "        - all plots on the same chart" << std::endl;
        std::cerr << "            csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "                | csv-plot '-;fields=block,x,y;color=red;weight=2' \\" << std::endl;
        std::cerr << "                           <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=blue;weight=2;shape=spline' \\" << std::endl;
        std::cerr << "                           <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=green;weight=2;shape=scatter'" << std::endl;
        std::cerr << "        - plots on different charts with grid layout: same command as above, but add 'chart=...'" << std::endl;
        std::cerr << "            - default layout: charts are stacked in a single column" << std::endl;
        std::cerr << "                ... | csv-plot ... '...;fields=...;chart=A' '...;fields=...;chart=B' '...;fields=...;chart=C'" << std::endl;
        std::cerr << "            - two columns:" << std::endl;
        std::cerr << "                ... | csv-plot --layout='grid;cols=2' ... '...;fields=...;chart=A' '...;fields=...;chart=B' '...;fields=...;chart=C'" << std::endl;
        std::cerr << "            - one row:" << std::endl;
        std::cerr << "                ... | csv-plot --layout='grid;rows=1' ... '...;fields=...;chart=A' '...;fields=...;chart=B' '...;fields=...;chart=C'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    xy chart with sliding window on endless series with 2 frames per second refresh, using --scroll" << std::endl;
        std::cerr << "        csv-random make --type f --range=0,20 | csv-paste line-number - | csv-repeat --pace --period 0.1 \\" << std::endl;
        std::cerr << "            | csv-plot --fps 2 --scroll '-;color=red;weight=2;size=40' \\" << std::endl;
        std::cerr << "                       <( csv-random make --seed 1234 --type f --range=0,30 | csv-paste line-number - | csv-repeat --pace --period 0.1 )';color=blue;weight=2;size=50'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    multiple series per stream" << std::endl;
        std::cerr << "        basics" << std::endl;
        std::cerr << "            csv-random make --type 4f --range=0,20 \\" << std::endl;
        std::cerr << "                | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "                | csv-plot '-;fields=block,x,y,series[0]/y,series[1]/y,series[2]/y' --fps 1" << std::endl;
        std::cerr << "        individual series options" << std::endl;
        std::cerr << "            csv-random make --type 4f --range=0,20 \\" << std::endl;
        std::cerr << "                | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "                | csv-plot '-;fields=block,x,y,series[0]/y,series[1]/y,series[2]/y;series[1]=color:blue;series[2]=color:green|chart:test2' --fps 1" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    display image histogram from the laptop camera" << std::endl;
        std::cerr << "        cv-cat --camera \\" << std::endl;
        std::cerr << "            | cv-calc histogram --interleave --output no-header \\" << std::endl;
        std::cerr << "            | csv-paste 'line-number;size=256;binary=ui' \\" << std::endl;
        std::cerr << "                        'line-number;size=256;index;binary=ui' \\" << std::endl;
        std::cerr << "                        '-;binary=3ui' \\" << std::endl;
        std::cerr << "            | csv-plot '-;fields=block,x,series[0]/y,series[1]/y,series[2]/y;binary=5ui;series[0]=color:blue;series[1]=color:green;series[2]=color:red'" << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "examples: use --help --verbose..." << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

// todo
// ! performance: struggles with more than 10000 points; find bottlenecks
// ! --chart=<config>
// ! chart axes: fixed range
// - application/examples/csv-plot/...: example command lines
// - gitlab: tutorial
// - --stream-config
// ? extents -> separate generic class
// - input
//   - t as x axis (QtCharts::QDateTimeAxis?)
//   - label: optional field
//   - multiple x,y fields in a single record
//     - -> multiple series with different properties (also different targets)
//     - allow common x, e.g. if series[0]/x not present, look for x field; series[1]/x present, overrules common x
//     - support t field as well
// - zoom
// - save as
//   - png
//   ? save all charts
// - chart
//   - types
//     - 2.5d charts
//     ? polar charts
//     ? chart types other than xy, e.g. pie chart
//   - properties
//     - title
//     ? legend (especially if multiple series on the same chart
//   - axes
//     - handle range of zero length
//     - add configurable margins
//     - handle various range policies
//     - fixed range
//   - --chart-config-fields
// - axis properties
//   - labels
//   - extents policies
//     - fixed
//     - auto-adjust
//     ? ignore outliers?
//     - label
//   - t markers on x axis (QtCharts::QDateTimeAxis?)
// - series properties
//   - properties as policy templated on qt series?
//   - spline: style? parametrization?
//   - scatter: style; derive from series? series -> base class?
//     - marker color
//     - marker shape
//   - --series-config-fields
// - layouts
//   - multi-window
//   - stacked
// - span policies
//   ? better autoscaling
//   ? better autoscrolling
// - main window
//   - add signal to update? currently, updates only after first timeout
// - building
//   ? move into a separate repository or add a separate cmake for cpack packaging
//   ? copy-paste block_buffer
//   ? package
//   ? expose on ppa
// ! qtcharts licence
// ! don't use block buffer as is? use double-buffered QList and pop front if exceeds size? (so far performance looks ok)
// ? qt, qtcharts: static layout configuration files?

QT_USE_NAMESPACE

struct blah
{ 
    boost::optional< double > x;
    boost::optional< double > y;
};

namespace comma { namespace visiting {

template <> struct traits< blah >
{
    template< typename K, typename V > static void visit( const K&, blah& t, V& v )
    {
        if( t.x ) { v.apply( "x", *t.x ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
        if( t.y ) { v.apply( "y", *t.y ); } // todo? what should be the behaviour in comma::csv::from_ascii on optional?
    }

    template< typename K, typename V > static void visit( const K&, const blah& t, V& v )
    {
        if( t.x ) { v.apply( "x", *t.x ); }
        if( t.y ) { v.apply( "y", *t.y ); }
    }
};

} }

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--input-fields" ) ) { std::cout << "t,series,block" << std::endl; return 0; } // quick and dirty
        if( options.exists( "--input-fields-example" ) ) { std::cout << comma::join( comma::csv::names< snark::graphics::plotting::record >( true, snark::graphics::plotting::record::sample( "series", 2 ) ), ',' ) << std::endl; return 0; }
        const std::vector< std::string >& unnamed = options.unnamed( "--no-stdin,--verbose,-v,--flush,--full-screen,--maximize,--pass-through,--pass,--scroll", "--.*,-[a-z].*" );
        boost::optional< unsigned int > stdin_index = boost::optional< unsigned int >();
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { if( unnamed[i] == "-" || unnamed[i].substr( 0, 2 ) == "-;" ) { stdin_index = i; break; } }
        snark::graphics::plotting::stream::config_t stream_config( options );
        std::vector< snark::graphics::plotting::stream::config_t > stream_configs;
        if( stdin_index ) { if( options.exists( "--no-stdin" ) ) { std::cerr << "csv-plot: due to --no-stdin, expected no stdin options; got: \"" << unnamed[ *stdin_index ] << "\"" << std::endl; return 1; } }
        else { stream_config.csv.filename = "-"; stream_configs.push_back( stream_config ); stream_config.pass_through = false; }
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { stream_configs.push_back( snark::graphics::plotting::stream::config_t( unnamed[i], stream_config ) ); stream_config.pass_through = false; }
        if( verbose ) { std::cerr << "csv-plot: got " << stream_configs.size() << " input stream config(s)" << std::endl; }
        const auto& chart_properties = options.values< std::string >( "--chart" );
        float timeout = options.value( "--timeout", 1. / options.value( "--frames-per-second,--fps", 10 ) ); // todo? update streams and charts at variable rates?
        std::string layout = options.value< std::string >( "--layout", "grid" );
        auto window_size = comma::csv::ascii< std::pair< unsigned int, unsigned int > >().get( options.value< std::string >( "--window-size", "800,600" ) );
        QApplication a( ac, av );
        snark::graphics::plotting::main_window main_window( stream_configs, chart_properties, window_size, layout, timeout );
        if( verbose )
        {
            std::cerr << "csv-plot: created " << main_window.charts().size() << " chart(s)" << std::endl;
            std::cerr << "csv-plot: created " << main_window.streams().size() << " input stream(s)" << std::endl;
            for( unsigned int i = 0; i < main_window.streams().size(); ++i )
            {
                for( unsigned int j = 0; j < main_window.streams()[i].series.size(); ++j ) { std::cerr << "csv-plot: stream " << i << ": series " << j << " will be shown on chart named: '" << main_window.streams()[i].series[j].config().chart << "'" << std::endl; }
            }
            if( !main_window.pass_through_stream_name().empty() ) { std::cerr << "csv-plot: stream '" << main_window.pass_through_stream_name() << "' will be passed through" << std::endl; }
        }
        main_window.start();
        if( verbose ) { std::cerr << "csv-plot: started" << std::endl; }
        options.exists( "--full-screen,--maximize" ) ? main_window.showMaximized() : main_window.show();
        return a.exec();
    }
    catch( std::exception& ex ) { std::cerr << "csv-plot: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-plot: unknown exception" << std::endl; }
    return 1;
}
