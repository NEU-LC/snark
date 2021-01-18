// Copyright (c) 2021 Vsevolod Vlaskine

// todo! add QtCharts licence

/// @author Vsevolod Vlaskine

#include <iostream>
#include <map>
#include <set>
#include <boost/shared_ptr.hpp>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QLineSeries>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QSplineSeries>
#include <QtCharts/QChartView>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/string/string.h>
#include <comma/name_value/parser.h>
#include <comma/name_value/serialize.h>
#include "csv_plot/charts.h"
#include "csv_plot/traits.h"

static void usage( bool verbose = false )
{
    std::cerr << "plot points from csv files or streams" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: cat xy.csv | csv-plot [<sources>] [<options>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "sources: input sources as <name>[;<options>]" << std::endl;
    std::cerr << "    <name>: e.g. points.csv, tcp:localhost:12345, etc" << std::endl;
    std::cerr << "    <options>: csv options or stream specific options (see below)" << std::endl;
    std::cerr << "               e.g: csv-plot \"points.bin;fields=x,y;binary=2d;color=red;weight=2\"" << std::endl;
    std::cerr << std::endl;
    std::cerr << "input fields: " << comma::join( comma::csv::names< snark::graphics::plotting::point >( false ), ',' ) << std::endl;
    std::cerr << "              default: x,y" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --help,-h: help, --help --verbose: more help" << std::endl;
    std::cerr << "    --frames-per-second,--fps=<value>; default=10; how often to update chart(s)" << std::endl;
    std::cerr << "    --input-fields; output possible input fields and exit" << std::endl;
    std::cerr << "    --layout=<layout>; default=grid; todo: layouts for multiple charts" << std::endl;
    std::cerr << "    --pass-through,--pass; todo: output to stdout the first stream on the command line" << std::endl;
    std::cerr << "    --timeout=<seconds>; how often to update, overrides --fps" << std::endl;
    std::cerr << "    --no-stdin: don't try to read from stdin" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;    
    std::cerr << "stream options" << std::endl;
    std::cerr << "    --color=<color>: plot color: black, white, red, green, blue" << std::endl;
    std::cerr << "                                 yellow, cyan, magenta, grey" << std::endl;
    std::cerr << "                                 or #rrggbb, e.g. #ff00ff" << std::endl;
    std::cerr << "    --scroll: if present, chart axes get adjusted to where the data is" << std::endl;
    std::cerr << "    --shape=<what>: line (default)" << std::endl;
    std::cerr << "                    todo: more shapes" << std::endl;
    std::cerr << "    --size,-s,--tail=<n>: plot last <n> records of stream; default 10000" << std::endl;
    std::cerr << "        use --style=dots, otherwise it's buggy; todo: fix" << std::endl;
    std::cerr << "    --style=<style>: todo" << std::endl;
//     std::cerr << "    --style=<style>: plot style (mapped into qwt styles)" << std::endl;
//     std::cerr << "        curve: no-curve" << std::endl;
//     std::cerr << "               lines (default)" << std::endl;
//     std::cerr << "               sticks" << std::endl;
//     std::cerr << "               steps" << std::endl;
//     std::cerr << "               dots" << std::endl;
    std::cerr << "    --weight=<weight>: point or line weight" << std::endl;
    if( verbose ) { std::cerr << std::endl << comma::csv::options::usage() << std::endl; }
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
        std::cerr << "        csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "            | csv-plot '-;fields=block,x,y;color=red;weight=2' \\" << std::endl;
        std::cerr << "                       <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=blue;weight=2;shape=spline' \\" << std::endl;
        std::cerr << "                       <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=green;weight=2;shape=scatter'" << std::endl;
        std::cerr << std::endl;
        std::cerr << "    xy chart with sliding window on endless series with 2 frames per second refresh, using --scroll" << std::endl;
        std::cerr << "        csv-random make --type f --range=0,20 | csv-paste line-number - | csv-repeat --pace --period 0.1 \\" << std::endl;
        std::cerr << "            | csv-plot --fps 2 --scroll '-;color=red;weight=2;size=40' \\" << std::endl;
        std::cerr << "                       <( csv-random make --seed 1234 --type f --range=0,30 | csv-paste line-number - | csv-repeat --pace --period 0.1 )';color=blue;weight=2;size=50'" << std::endl;
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
// ! don't use block buffer as is? use double-buffered QList and pop front if exceeds size? (so far performance looks ok)
// ? qt, qtcharts: static layout configuration files?
// - move main window to separate file
// - --stream-config
// - input
//   - t as x axis (QtCharts::QDateTimeAxis?)
//   - multiple x,y fields in a single record
//     - -> multiple series with different properties (also different targets)
//     - allow common x, e.g. if series[0]/x not present, look for x field; series[1]/x present, overrules common x
//     - support t field as well
// - zoom
// - save as
//   - png
//   ? save all windows
// - chart
//   - types
//     - 2.5d charts
//     ? chart types other than xy, e.g. pie chart
//   - properties
//     - title
//     ? legend
//   - --chart-config-fields
// - axes properties
//   - extents policies
//     - fixed
//     - auto-adjust
//     - label
//   - t markers on x axis (QtCharts::QDateTimeAxis?)
// - series properties
//   - properties as policy templated on qt series?
//   - title
//   - target chart
//   - spline: style?
//   - scatter: style; derive from series? series -> base class?
//     - marker color
//     - marker shape
//   - --series-config-fields
// - layouts
//   - title
//   - multiple charts
//   - support multi-window
//   - grid layout
//   - tabs layout
// - span policies
//   - better autoscaling
//   - better autoscrolling

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

static bool verbose;

static snark::graphics::plotting::stream* make_stream( snark::graphics::plotting::stream::config_t config, QChart* chart )
{
    static std::string pass_through_stream_name;
    snark::graphics::plotting::stream* s = nullptr;
    if( config.series.shape == "line" || config.series.shape.empty() ) { s = new snark::graphics::plotting::stream( new QLineSeries( chart ), config ); }
    else if( config.series.shape == "spline" ) { s = new snark::graphics::plotting::stream( new QSplineSeries( chart ), config ); }
    else if( config.series.shape == "scatter" ) { s = new snark::graphics::plotting::stream( new QScatterSeries( chart ), config ); }
    else { std::cerr << "csv-plot: expected stream type as shape, got: \"" << config.series.shape << "\"" << std::endl; exit( 1 ); }
    if( s->config.pass_through )
    {
        if( !pass_through_stream_name.empty() ) { std::cerr << "csv-plot: expected pass-through only for one stream; got at least two: '" << pass_through_stream_name << "' and then '" << s->config.csv.filename << "'" << std::endl; exit( 1 ); }
        if( verbose ) { std::cerr << "csv-plot: stream '" << s->config.csv.filename << "' will be passed through" << std::endl; }
        pass_through_stream_name = s->config.csv.filename;
    }
    return s;
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::graphics::plotting::point >( false ), ',' ) << std::endl; return 0; }
        snark::graphics::plotting::stream::config_t config( options );
        config.csv.full_xpath = false;
        if( config.csv.fields.empty() ) { config.csv.fields = "x,y"; }
        const std::vector< std::string >& unnamed = options.unnamed( "--no-stdin,--verbose,-v,--flush,--pass-through,--pass,--scroll", "--.*,-[a-z].*" );
        boost::optional< unsigned int > stdin_index = boost::optional< unsigned int >();
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { if( unnamed[i] == "-" || unnamed[i].substr( 0, 2 ) == "-;" ) { stdin_index = i; break; } }
        std::vector< snark::graphics::plotting::stream::config_t > configs;
        if( stdin_index ) { if( options.exists( "--no-stdin" ) ) { std::cerr << "csv-plot: due to --no-stdin, expected no stdin options; got: \"" << unnamed[ *stdin_index ] << "\"" << std::endl; return 1; } }
        else { config.csv.filename = "-"; configs.push_back( config ); config.pass_through = false; }
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { configs.push_back( comma::name_value::parser( "filename", ';', '=', false ).get( unnamed[i], config ) ); config.pass_through = false; }
        if( verbose ) { std::cerr << "csv-plot: got " << configs.size() << " input stream(s)" << std::endl; }
        float timeout = options.value( "--timeout", 1. / options.value( "--frames-per-second,--fps", 10 ) ); // todo? update streams and charts at variable rates?
        QApplication a( ac, av );
        std::map< std::string, snark::graphics::plotting::chart* > charts;
        for( const auto& c: configs ) { if( charts.find( c.series.chart ) == charts.end() ) { charts[ c.series.chart ] = new snark::graphics::plotting::xy_chart( timeout, c.series.title ); } }
        if( verbose ) { std::cerr << "csv-plot: creates " << charts.size() << " chart(s)" << std::endl; }
        QMainWindow window;
        QChartView view( charts.begin()->second ); // todo: layouts
        view.setRenderHint( QPainter::Antialiasing );
        for( const auto& c: configs ) { charts[ c.series.chart ]->push_back( make_stream( c, charts[ c.series.chart ] ) ); }
        if( verbose ) { std::cerr << "csv-plot: created " << configs.size() << " input stream(s)" << std::endl; }
        window.setCentralWidget( &view );
        window.resize( 800, 600 ); // todo: make configurable
        for( auto c: charts ) { c.second->start(); }
        std::cerr << "csv-plot: started" << std::endl;
        window.show();
        return a.exec();
    }
    catch( std::exception& ex ) { std::cerr << "csv-plot: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-plot: unknown exception" << std::endl; }
    return 1;
}
