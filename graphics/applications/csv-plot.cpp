// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// Copyright (c) 2021 Vsevolod Vlaskine
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

// todo! add QtCharts licence

/// @author Vsevolod Vlaskine

#include <iostream>
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
    std::cerr << "    --pass; todo" << std::endl;
    std::cerr << "    --timeout=<seconds>; how often to update, overrides --fps" << std::endl;
    std::cerr << "    --no-stdin: don't try to read from stdin" << std::endl;
    std::cerr << "    --verbose,-v: more output" << std::endl;
    std::cerr << std::endl;    
    std::cerr << "stream options" << std::endl;
    std::cerr << "    --color=<color>: plot color: black, white, red, green, blue" << std::endl;
    std::cerr << "                                 yellow, cyan, magenta, grey" << std::endl;
    std::cerr << "                                 or #rrggbb, e.g. #ff00ff" << std::endl;
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
        std::cerr << "    xy chart with multiple inputs block by block with different shapes" << std::endl;
        std::cerr << "        csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - \\" << std::endl;
        std::cerr << "            | csv-plot '-;fields=block,x,y;color=red;weight=2' \\" << std::endl;
        std::cerr << "                       <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=blue;weight=2;shape=spline' \\" << std::endl;
        std::cerr << "                       <( csv-random make --type f --range=0,20 | csv-paste 'line-number;size=10' 'line-number;size=10;index' - )';fields=block,x,y;color=green;weight=2;shape=scatter'" << std::endl;
        std::cerr << std::endl;
    }
    else
    {
        std::cerr << "examples: use --help --verbose..." << std::endl;
    }
    std::cerr << std::endl;
    exit( 0 );
}

QT_USE_NAMESPACE
QT_CHARTS_USE_NAMESPACE

static snark::graphics::plotting::stream* make_streams( const snark::graphics::plotting::stream::config_t& config, QChart* chart )
{
    if( config.shape == "line" || config.shape.empty() ) { return new snark::graphics::plotting::stream( new QLineSeries( chart ), config ); }
    if( config.shape == "spline" ) { return new snark::graphics::plotting::stream( new QSplineSeries( chart ), config ); }
    if( config.shape == "scatter" ) { return new snark::graphics::plotting::stream( new QScatterSeries( chart ), config ); }
    std::cerr << "csv-plot: expected stream type as shape, got: \"" << config.shape << "\"" << std::endl;
    exit( 1 );
}

// todo
// ! don't use block buffer as is? use double-buffered QList and pop front if exceeds size? (so far performance looks ok)
// ? qt, qtcharts: static layout configuration files?
// - span policies
//   - autoscaling
//   - autoscrolling
// - move main window to separate file
// - --stream-config
// - input
//   - multiple x,y fields in a single record
//     - -> multiple series with different properties (also different targets)
//     - allow common x, e.g. if series[0]/x not present, look for x field; series[1]/x present, overrules common x
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
// - axes properties
//   - extents policies
//     - fixed
//     - auto-adjust
// - series properties
//   - properties as policy templated on qt series?
//   - title
//   - target chart
//   - spline: style?
//   - scatter: style; derive from series? series -> base class?
//     - marker color
//     - marker shape
// - layouts
//   - title
//   - multiple charts
//   - support multi-window
//   - grid layout
//   - tabs layout

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        bool verbose = options.exists( "--verbose,-v" );
        if( options.exists( "--input-fields" ) ) { std::cout << comma::join( comma::csv::names< snark::graphics::plotting::point >( false ), ',' ) << std::endl; return 0; }
        snark::graphics::plotting::stream::config_t config( options );
        config.csv.full_xpath = false;
        if( config.csv.fields.empty() ) { config.csv.fields = "x,y"; }
        const std::vector< std::string >& unnamed = options.unnamed( "--no-stdin,--verbose,-v,--flush", "--.*,-[a-z].*" );
        boost::optional< unsigned int > stdin_index = boost::optional< unsigned int >();
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { if( unnamed[i] == "-" || unnamed[i].substr( 0, 2 ) == "-;" ) { stdin_index = i; break; } }
        QApplication a( ac, av );
        QMainWindow window;
        snark::graphics::plotting::chart* chart = new snark::graphics::plotting::xy_chart( options.value( "--timeout", 1. / options.value( "--frames-per-second,--fps", 10 ) ) ); // todo? update streams and charts at variable rates?
        chart->setTitle( "test chart" );
        chart->legend()->hide();
        chart->setAnimationOptions( QChart::AllAnimations ); // todo? make configurable?
        QChartView chartView( chart );
        chartView.setRenderHint( QPainter::Antialiasing );
        if( options.exists( "--no-stdin" ) ) { if( stdin_index ) { std::cerr << "csv-plot: due to --no-stdin, expected no stdin options; got: \"" << unnamed[ *stdin_index ] << "\"" << std::endl; return 1; } }
        else if( !stdin_index ) { config.csv.filename = "-"; chart->push_back( make_streams( config, chart ) ); }
        for( unsigned int i = 0; i < unnamed.size(); ++i ) { chart->push_back( make_streams( comma::name_value::parser( "filename", ';', '=', false ).get( unnamed[i], config ), chart ) ); }
        if( verbose ) { std::cerr << "csv-plot: got " << chart->streams().size() << " input stream(s)" << std::endl; }
        window.setCentralWidget( &chartView );
        window.resize( 800, 600 ); // todo: make configurable
        chart->start();
        window.show();
        return a.exec();
    }
    catch( std::exception& ex ) { std::cerr << "csv-plot: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "csv-plot: unknown exception" << std::endl; }
    return 1;
}
