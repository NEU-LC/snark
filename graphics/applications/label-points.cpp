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


#ifdef WIN32
#include <WinSock2.h>
#endif

#include <vector>
#include <boost/shared_ptr.hpp>
#include <qapplication.h>
#include <Qt3D/qcolor4ub.h>
#include <comma/application/command_line_options.h>
#include <comma/csv/options.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/string/string.h>
#include <comma/csv/format.h>
//#include <comma/csv/stream.h>
//#include <comma/base/types.h>
//#include <comma/visiting/traits.h>
//#include <snark/math/traits.h>
//#include <Eigen/Core>
#include "snark/graphics/applications/label_points/MainWindow.h"

void usage( bool verbose )
{
    std::cerr << std::endl;
    std::cerr << "simple tool for manual labeling of point clouds" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: label-points [<options>] <filenames>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    --help,-h : show help; --help --verbose: show more help" << std::endl;
    std::cerr << "    --background-colour <colour> : e.g. #ff0000, default: #000000 (black)" << std::endl;
    std::cerr << "    --fix-duplicated : if present, re-label with the same id all duplicated points" << std::endl;
    std::cerr << "    --orthographic : if present, use orthographic projection instead of perspective projection" << std::endl;
    std::cerr << "    --fov <fov> : set camera field of view to <fov>, in degrees. Only has effect for perspective projection. Default: 45 degrees" << std::endl;
    std::cerr << "    --repair : if present, repair and save files without bringing up gui;" << std::endl;
    std::cerr << "               currently only re-label duplicated points" << std::endl;
    std::cerr << "    --verbose,-v : more info, e.g. show x,y,z,id of selected points" << std::endl;
    std::cerr << "                   warning: can be lots of output on large files" << std::endl;
    std::cerr << "    --output-fields: output fields for points selected with a pipette tool (default: x,y,z,id)" << std::endl;
    std::cerr << "    --show-output-format: show binary output format and exit" << std::endl;
    if( verbose )
    {
        std::cerr << "csv options" << std::endl;
        std::cerr << comma::csv::options::usage() << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "<fields>: x,y,z,id" << std::endl;
    std::cerr << "    default: x,y,z,id" << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr << "    label-points scan.csv" << std::endl;
    std::cerr << "    label-points scan.csv --fields=,,,x,y,z,,id" << std::endl;
    std::cerr << "    label-points semantic_labels.csv partitions.csv" << std::endl;
    std::cerr << "    label-points 'scan.csv;fields=x,y,z,,id' 'scan.csv;fields=x,y,z,id'" << std::endl;
    std::cerr << "    label-points scan.csv --output-fields=id,x,y > selected_points.csv" << std::endl;
    std::cerr << std::endl;
    exit( -1 );
}

int main( int argc, char** argv )
{
    try
    {
        comma::command_line_options options( argc, argv );
        bool verbose = options.exists( "--verbose,-v" );
        if( argc == 1 || options.exists( "--help" ) || options.exists( "-h" ) ) { usage( verbose ); }
        comma::csv::options csvOptions( argc, argv );
        QColor4ub backgroundcolour( QColor( QString( options.value< std::string >( "--background-colour", "#000000" ).c_str() ) ) );
        if( csvOptions.fields == "" ) { csvOptions.fields = "x,y,z,id"; }
        comma::csv::options csv_out( csvOptions );
        csv_out.fields = options.value< std::string >( "--output-fields", "x,y,z,id" );
        if( csvOptions.binary() ) { csv_out.format( comma::csv::format::value< snark::graphics::View::PointWithId >( csv_out.fields, csv_out.full_xpath ) ); }
        if( options.exists( "--show-output-format" ) ) { std::cout << comma::csv::format::value< snark::graphics::View::PointWithId >( csv_out.fields, csv_out.full_xpath ) << std::endl; return 0; }
        std::vector< std::string > files = options.unnamed( "--repair,--fix-duplicated,--flush,--verbose,-v,--show-output-format",
                                                            "--binary,--bin,-b,--fields,--delimiter,-d,--background-colour,--precision,--output-fields,--orthographic,--fov" );
        if( files.empty() ) { std::cerr << "label-points: please specify input files" << std::endl; return 1; }
        std::vector< comma::csv::options > dataset_csv_options;
        bool fixDuplicated = options.exists( "--fix-duplicated" );
        for( std::size_t i = 0; i < files.size(); ++i )
        {
            dataset_csv_options.push_back( comma::name_value::parser( "filename" ).get( files[i], csvOptions ) );
        }
        if( options.exists( "--repair" ) ) // quick and dirty
        {
            for( std::size_t i = 0; i < dataset_csv_options.size(); ++i ) { snark::graphics::View::Dataset::repair( dataset_csv_options[i] ); }
            return 0;
        }
        else
        {
            QApplication application( argc, argv );
            bool orthographic = options.exists( "--orthographic" );
            double fieldOfView = options.value< double >( "--fov", 45 );
            boost::scoped_ptr< snark::graphics::View::Viewer > viewer( new snark::graphics::View::Viewer( dataset_csv_options, csv_out, fixDuplicated, backgroundcolour, orthographic, fieldOfView, verbose ) );
            snark::graphics::View::MainWindow mainWindow( comma::join( argv, argc, ' ' ), viewer.get() );
            mainWindow.show();
            /*return*/ application.exec();
            viewer.reset(); // HACK to prevent segfault on exit TODO fix properly or maybe it is a bug in Qt3D ?
            return 1;
        }
    }
    catch( std::exception& ex )
    {
        std::cerr << "label-points: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "label-points: unknown exception" << std::endl;
    }
}
