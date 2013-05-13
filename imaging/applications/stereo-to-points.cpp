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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#include <iostream>
#include <boost/program_options.hpp>
#include <comma/application/signal_flag.h>
#include <opencv2/highgui/highgui.hpp>
#include "stereo/parameters.h"
#include "stereo/stereo.h"
#include "stereo/disparity.h"
#include "stereo/rectified.h"
#include "stereo/stereo_stream.h"
#include <comma/csv/impl/program_options.h>

cv::StereoSGBM sgbm;    

template< typename T >
void run( const snark::imaging::camera_parser& left_parameters, const snark::imaging::camera_parser& right_parameters,
          unsigned int width, unsigned int height, const comma::csv::options& csv, const cv::Mat& left, const cv::Mat& right, bool input_rectified = false )
{
    if( left_parameters.has_map() )
    {
        T stereoPipeline( left_parameters, right_parameters,
                          left_parameters.map_x(), left_parameters.map_y(),
                          right_parameters.map_x(), right_parameters.map_y(),
                          csv, input_rectified );
        stereoPipeline.process( left, right, sgbm );
    }
    else
    {
        T stereoPipeline( left_parameters, right_parameters, width, height, csv, input_rectified );
        stereoPipeline.process( left, right, sgbm );
    }
}

template< typename T >
void run_stream( const snark::imaging::camera_parser& left_parameters, const snark::imaging::camera_parser& right_parameters,
          const boost::array< unsigned int, 6 > roi, const comma::csv::options& input_csv, const comma::csv::options& output_csv, bool input_rectified = false )
{
    if( left_parameters.has_map() )
    {
        snark::imaging::stereo_stream< T > stream( left_parameters, right_parameters, roi,
                          left_parameters.map_x(), left_parameters.map_y(),
                          right_parameters.map_x(), right_parameters.map_y(),
                          input_csv, output_csv, input_rectified );
        while( std::cin.good() && !std::cin.eof() )
        {
            stream.read( sgbm );
        }
    }
    else
    {
        snark::imaging::stereo_stream< T > stream( left_parameters, right_parameters, roi, input_csv, output_csv, input_rectified );
        while( std::cin.good() && !std::cin.eof() )
        {
            stream.read( sgbm );
        }
    }
}

int main( int argc, char** argv )
{
    try
    {
        #ifdef WIN32
        _setmode( _fileno( stdin ), _O_BINARY );
        _setmode( _fileno( stdout ), _O_BINARY );
        #endif

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
            ( "left", boost::program_options::value< std::string >( &leftImage ), "left image file name (file input only)" )
            ( "right", boost::program_options::value< std::string >( &rightImage ), "right image file name (file input only)" )
            ( "roi", boost::program_options::value< std::string >( &roi ),
              "left and right images roi in pixel (stdin input only), arg=<left-pos-x,left-pos-y,right-pos-x,right-pos-y,width,height>" )
            ( "disparity", "output disparity image instead of point cloud" )
            ( "output-rectified", "output rectified image pair instead of point cloud" )
            ( "input-rectified", "input images are already rectified" )
            ( "window-size,w", boost::program_options::value< int >( &sgbm.SADWindowSize )->default_value(5), "sgbm SADWindowSize (see OpenCV documentation)" )
            ( "min-disparity,m", boost::program_options::value< int >( &sgbm.minDisparity )->default_value(0), "sgbm minDisparity" )
            ( "num-disparity,n", boost::program_options::value< int >( &sgbm.numberOfDisparities )->default_value(80), "sgbm numberOfDisparities" )
            ( "uniqueness,u", boost::program_options::value< int >( &sgbm.uniquenessRatio )->default_value(10), "sgbm uniquenessRatio" )
            ( "speckle-window-size,s", boost::program_options::value< int >( &sgbm.speckleWindowSize )->default_value(1000), "sgbm speckleWindowSize" )
            ( "speckle-range,r", boost::program_options::value< int >( &sgbm.speckleRange )->default_value(16), "sgbm speckleRange" )
            ( "disp12-max", boost::program_options::value< int >( &sgbm.disp12MaxDiff )->default_value(1), "sgbm disp12MaxDiff" )
            ( "pre-filter-cap", boost::program_options::value< int >( &sgbm.preFilterCap )->default_value(63), "sgbm preFilterCap" )
            ( "full-dp,f", "use fullDP, uses a lot of memory" );
        description.add( comma::csv::program_options::description( "t,x,y,z,r,g,b,block" ) );
        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );
        if ( vm.count( "help" ) || vm.count( "long-help" ) )
        {
            std::cerr << "read stereo images from files or stdin, outputs point cloud or disparity image or rectified image pair to stdout" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << std::endl;
            std::cerr << "output format: t,x,y,z,r,g,b,block for point cloud " << std::endl;
            std::cerr << "               t,rows,cols,type for disparity image or rectified image pair " << std::endl;
            std::cerr << std::endl;
            std::cerr << "example config file with pre-computed rectify maps (eg. with the bumblebee camera factory calibration):\n" << std::endl;
            std::cerr << "left=\n{\n    focal-length=\"1604.556763,1604.556763\"\n    centre=\"645.448181,469.367188\"\n    image-size=\"1280,960\"" << std::endl;
            std::cerr << "    map=/usr/local/etc/shrimp.bumblebee-left.bin\n}\nright=\n{\n    focal-length=\"1604.556763,1604.556763\"" << std::endl;
            std::cerr << "    centre=\"645.448181,469.367188\"\n    translation=\"-0.239928,0,0\"\n    image-size=\"1280,960\"" << std::endl;
            std::cerr << "    map=/usr/local/etc/shrimp.bumblebee-right.bin\n}\n" << std::endl;
            std::cerr << std::endl;
            std::cerr << "example config file with intrinsic parameters (eg. from the matlab calibration toolbox):\n" << std::endl;
            std::cerr << "left=\n{\n    focal-length=\"534.7,534.7\"\n    centre=\"335.1,240.2\"\n    distortion=\"-0.274568,-0.018329,0,0,0\"\n}\n";
            std::cerr << "right=\n{\n    focal-length=\"534.7,534.7\"\n    centre=\"334,241.6\"\n    distortion=\"-0.274568,0.093,0,0,0\"\n";
            std::cerr << "    rotation=\"-0.0177686,-0.0214235,-0.00491403\"\n    translation=\"0.21,0,0\"\n}\n";
            std::cerr << std::endl;
            std::cerr << "examples: " << std::endl;
            std::cerr << "  output and view point cloud from 2 image files: " << std::endl;
            std::cerr << "    stereo-to-points --left left.bmp --right right.bmp --config bumblebee.config --left-path left --right-path right --binary t,3d,3ub,ui \\" << std::endl;
            std::cerr << "    | view-points --fields t,x,y,z,r,g,b,block --binary t,3d,3ub,ui" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  output and view disparity from 2 image files: " << std::endl;
            std::cerr << "    stereo-to-points --left left.bmp --right right.bmp --config bumblebee.config --left-path left --right-path right \\" << std::endl;
            std::cerr << "    --disparity | cv-cat --output=no-header encode=ppm | display" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  output and view rectified image pair from 2 image files: " << std::endl;
            std::cerr << "    stereo-to-points --left left.bmp --right right.bmp --config bumblebee.config --left-path left --right-path right \\" << std::endl;
            std::cerr << "    --output-rectified | cv-cat --output=no-header encode=ppm | display" << std::endl;
            std::cerr << std::endl;
            std::cerr <<  " split the stereo processing pipeline into two parts " << std::endl;
            std::cerr <<  "   cat 20130319T052306.593000.bin | cv-cat \"split;bayer=4\" | " << std::endl;
            std::cerr <<  "   stereo-to-points --config=bb.matlab.config --left-path=left --right-path=right --roi=\"0,1920,0,0,1280,960\" --output-rectified | \\ " << std::endl;
            std::cerr <<  "   stereo-to-points --config=bb.matlab.config --left-path=left --right-path=right --roi=\"0,0,1280,0,1280,960\" --input-rectified --binary t,3d,3ub,ui | \\ " << std::endl;
            std::cerr <<  "   csv-select --binary=t,3d,3ub,ui --fields=,,,,,,,block --to=0 --sorted > pointcloud.bin" << std::endl;

            std::cerr << std::endl;
            std::cerr << "  stream data in qlib log format and output disparity: " << std::endl;
            std::cerr << "    cat BumblebeeVideo*.bin | q-cat | cv-cat bayer=4 | stereo-to-points --config /usr/local/etc/shrimp.config --left-path bumblebee/camera-left\\" << std::endl;
            std::cerr << "    --right-path bumblebee/camera-right --roi 0,1920,0,0,1280,960 --disparity | cv-cat \"resize=640,480;view\" > /dev/null" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  stream data in qlib log format and output point cloud: " << std::endl;
            std::cerr << "    cat BumblebeeVideo*.bin | q-cat | cv-cat bayer=4 | stereo-to-points --config /usr/local/etc/shrimp.config --left-path bumblebee/camera-left\\" << std::endl;
            std::cerr << "    --right-path bumblebee/camera-right --roi 0,1920,0,0,1280,960 --binary t,3d,3ub,ui | view-points --fields t,x,y,z,r,g,b,block --binary t,3d,3ub,ui" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  view disparity live from shrimp: " << std::endl;
            std::cerr << "    netcat shrimp.server 55003 | cv-cat \"split;bayer=4\" | stereo-to-points --config /usr/local/etc/shrimp.config \\" << std::endl;
            std::cerr << "    --left-path bumblebee/camera-left --right-path bumblebee/camera-right --roi 0,1920,0,0,1280,960 --disparity  \\" << std::endl;
            std::cerr << "    | cv-cat \"resize=640,480;view\" > /dev/null" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  view point cloud live from shrimp: ( does not seem to work on shrimp, maybe due to OpenCV 2.3, works with OpenCV 2.4 )" << std::endl;
            std::cerr << "    netcat shrimp.server 55003 | cv-cat \"split;bayer=4\" | stereo-to-points --config /usr/local/etc/shrimp.config \\" << std::endl;
            std::cerr << "    --left-path bumblebee/camera-left --right-path bumblebee/camera-right --roi 0,1920,0,0,1280,960 --binary t,3d,3ub,ui  \\" << std::endl;
            std::cerr << "    | view-points --fields t,x,y,z,r,g,b,block --binary t,3d,3ub,ui" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  batch process images from files, assuming you have ppm images with the same names in left/ and right/ directories: " << std::endl;
            std::cerr << "    disparity: " << std::endl;
            std::cerr << "    find left -name '*.ppm' | sort | parallel  'stereo-to-points --left left/{/} --right right/{/} --config bumblebee.config \\" << std::endl;;
            std::cerr << "    --left-path left --right-path right --disparity --full-dp | cv-cat --output=no-header encode=ppm > disparity-{/}'" << std::endl;
            std::cerr << "    point cloud: " << std::endl;
            std::cerr << "    find left -name '*.ppm' | sort | parallel  'stereo-to-points --left left/{/} --right right/{/} --config bumblebee.config \\" << std::endl;;
            std::cerr << "    --left-path left --right-path right --binary t,3d,3ub,ui --full-dp > cloud-{/.}.bin" << std::endl;
            std::cerr << std::endl;
            std::cerr << "  known bugs: point cloud doesn't seem to work with opencv 2.3 ( e.g. on shrimp ), works with opencv 2.4 " << std::endl;
            std::cerr << std::endl;
            return 1;
        }

        sgbm.fullDP = ( vm.count( "full-dp" ) );
        
        snark::imaging::camera_parser leftParameters( configFile, leftPath );
        snark::imaging::camera_parser rightParameters( configFile, rightPath );

        cv::Mat left = cv::imread( leftImage, 1 );
        cv::Mat right = cv::imread( rightImage, 1 );

        comma::csv::options csv = comma::csv::program_options::get( vm );
        if( vm.count( "disparity" ) || vm.count( "output-rectified" ) )
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
            if( vm.count( "roi" ) )
            {
                std::cerr << argv[0] << ": specify either --roi when reading from stdin, or --left and --right when reading from files" << std::endl;
                return 1;
            }
            if( vm.count( "disparity" ) == 0 && vm.count( "output-rectified" ) == 0 )
            {
                run< snark::imaging::stereo >( leftParameters, rightParameters, left.cols, left.rows, csv, left, right, vm.count( "input-rectified" ) );
            }
            else
            {
                if( vm.count( "disparity" ) )
                    run< snark::imaging::disparity >( leftParameters, rightParameters, left.cols, left.rows, csv, left, right, vm.count( "input-rectified" ) );
                if( vm.count( "output-rectified" ) )
                    run< snark::imaging::rectified >( leftParameters, rightParameters, left.cols, left.rows, csv, left, right, vm.count( "input-rectified" ) );
            }
        }
        else
        {
            // read from std in
            if( !vm.count( "roi" ) )
            {
                std::cerr << argv[0] << ": please specify --roi to read from stdin or --left and --right to read from files" << std::endl;
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
            if( vm.count( "disparity" ) == 0 && vm.count( "output-rectified" ) == 0 )
            {
                run_stream< snark::imaging::stereo >( leftParameters, rightParameters, roiArray, input_csv, csv, vm.count( "input-rectified" ) );
            }
            else
            {
                if( vm.count( "disparity" ) )
                    run_stream< snark::imaging::disparity >( leftParameters, rightParameters, roiArray, input_csv, csv, vm.count( "input-rectified" ) );
                if( vm.count( "output-rectified" ) )
                    run_stream< snark::imaging::rectified >( leftParameters, rightParameters, roiArray, input_csv, csv, vm.count( "input-rectified" ) );
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
