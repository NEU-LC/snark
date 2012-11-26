// This file is part of snark, a generic and flexible library
// for robotics research.
//
// Copyright (C) 2011 The University of Sydney
//
// snark is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.
//
// snark is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License
// for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with snark. If not, see <http://www.gnu.org/licenses/>.

#include <iostream>
#include <boost/program_options.hpp>
#include <comma/application/signal_flag.h>
#include <opencv2/highgui/highgui.hpp>
#include "stereo/parameters.h"
#include "stereo/disparity.h"
#include <comma/csv/impl/program_options.h>

int main( int argc, char** argv )
{
    try
    {
        boost::program_options::options_description description( "options" );
        std::string configFile;
        std::string leftPath;
        std::string rightPath;
        std::string leftImage;
        std::string rightImage;
        description.add_options()
            ( "help,h", "display help message" )
            ( "config", boost::program_options::value< std::string >( &configFile ), "camera config file" )
            ( "left-path", boost::program_options::value< std::string >( &leftPath ), "left camera config file path" )
            ( "right-path", boost::program_options::value< std::string >( &rightPath ), "right camera config file path" )
            ( "left", boost::program_options::value< std::string >( &leftImage ), "left image" )
            ( "right", boost::program_options::value< std::string >( &rightImage ), "right image" );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) || vm.count( "long-help" ) )
        {
            std::cerr << "acquire image" << std::endl;
            std::cerr << description << std::endl;
            return 1;
        }

        snark::imaging::camera_parser leftParameters( configFile, leftPath );
        snark::imaging::camera_parser rightParameters( configFile, rightPath );

        cv::Mat left = cv::imread( leftImage, 1 );
        cv::Mat right = cv::imread( rightImage, 1 );

        comma::csv::options csv = comma::csv::program_options::get( vm );
        if( csv.fields.empty() )
        {
            csv.fields = "t,rows,cols,type";
            csv.format( "t,3ui" );
        }
        snark::imaging::disparity disparityPipeline( leftParameters, rightParameters, left.cols, left.rows, csv );
        disparityPipeline.process( left, right );

        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << argv[0] << ": " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << argv[0] << ": unknown exception" << std::endl;
    }
    return 1;
}
