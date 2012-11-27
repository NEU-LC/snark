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
#include "stereo/stereo.h"
#include "stereo/disparity.h"
#include "stereo/stereo_stream.h"
#include <comma/csv/impl/program_options.h>

template< typename T >
void run( const snark::imaging::camera_parser& left_parameters, const snark::imaging::camera_parser& right_parameters,
          unsigned int width, unsigned int height, const comma::csv::options& csv, const cv::Mat& left, const cv::Mat& right )
{
    T stereoPipeline( left_parameters, right_parameters, width, height, csv );
    stereoPipeline.process( left, right );
}

template< typename T >
void run_stream( const snark::imaging::camera_parser& left_parameters, const snark::imaging::camera_parser& right_parameters,
          const boost::array< unsigned int, 6 > roi, const comma::csv::options& input_csv, const comma::csv::options& output_csv )
{
    snark::imaging::stereo_stream< T > stream( left_parameters, right_parameters, roi, input_csv, output_csv );
    while( std::cin.good() && !std::cin.eof() )
    {
        stream.read();
    }
}

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
        std::string roi;
        description.add_options()
            ( "help,h", "display help message" )
            ( "config", boost::program_options::value< std::string >( &configFile ), "camera config file" )
            ( "left-path", boost::program_options::value< std::string >( &leftPath ), "left camera config file path" )
            ( "right-path", boost::program_options::value< std::string >( &rightPath ), "right camera config file path" )
            ( "left", boost::program_options::value< std::string >( &leftImage ), "left image" )
            ( "right", boost::program_options::value< std::string >( &rightImage ), "right image" )
            ( "roi", boost::program_options::value< std::string >( &roi ), "left and right images roi in pixel, arg=<left-pos-x,left-pos-y,right-pos-x,right-pos-y,width,height>" )
            ( "disparity,d", "output disparity image instead of point cloud" );
        description.add( comma::csv::program_options::description( "t,x,y,z,r,g,b,block" ) );
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
        if( vm.count( "disparity" ) )
        {
            csv.fields = "t,rows,cols,type";
            csv.format( "t,3ui" );
        }

        if( vm.count( "left" ) || vm.count( "right" ) )
        {
            if( !( vm.count( "left" ) && vm.count( "right" ) ) )
            {
                std::cerr << argv[0] << ": need both --left and --right" << std::endl;
                return 1;
            }
            if( vm.count( "disparity" ) == 0 )
            {
                run< snark::imaging::stereo >( leftParameters, rightParameters, left.cols, left.rows, csv, left, right );
            }
            else
            {
                run< snark::imaging::disparity >( leftParameters, rightParameters, left.cols, left.rows, csv, left, right );
            }
        }
        else
        {
            // read from std in
            if( !vm.count( "roi" ) )
            {
                std::cerr << argv[0] << ": please specify --roi" << std::endl;
                return 1;
            }
            std::vector< std::string > v = comma::split( roi, ',' );
            if( v.size() != 6 )
            {
                std::cerr << argv[0] << ": please specify --roi as <left-pos-x,left-pos-y,right-pos-x,right-pos-y,width,height>" << std::endl;
                return 1;
            }
            boost::array< unsigned int, 6 > roiArray;
            for( unsigned int i = 0; i < 6; i++ )
            {
                roiArray[i] = boost::lexical_cast< unsigned int >( v[i] );
            }
            comma::csv::options input_csv;
            input_csv.fields = "t,rows,cols,type";
            input_csv.format( "t,3ui" );
            if( vm.count( "disparity" ) == 0 )
            {
                run_stream< snark::imaging::stereo >( leftParameters, rightParameters, roiArray, input_csv, csv );
            }
            else
            {
                run_stream< snark::imaging::disparity >( leftParameters, rightParameters, roiArray, input_csv, csv );
            }
        }

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
