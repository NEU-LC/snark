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


#include <fstream>
#include <iostream>
#include <boost/scoped_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/string/split.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

static void usage()
{
    std::cerr << std::endl;
    std::cerr << "compute the undistortion transformation map" << std::endl;
    std::cerr << std::endl;
    std::cerr << "the map is stored as 2 appended row-major float images," << std::endl;
    std::cerr << "one for the x coordinate, one for y" << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage: image-undistort-map <file> <options>" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<file>: output file, \"-\" for stdout" << std::endl;
    std::cerr << std::endl;
    std::cerr << "<options>" << std::endl;
    std::cerr << "    --intrinsics <fx,fy,cx,cy>: intrinsic parameters in pixel" << std::endl;
    std::cerr << "    --distortion <k1,k2,p1,p2,k3>: distortion parameters" << std::endl;
    std::cerr << "    --size <width>x<height>: image size in pixel" << std::endl;
    std::cerr << std::endl;
    std::cerr << "example: " << std::endl;
    std::cerr << "    image-undistort-map --intrinsics \"830.2,832.4,308.8,232.6\" --distortion \"-0.42539,0.14800,0.00218,0.00061\" --size 1280x960 bumblebee-undistort-map.bin " << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

static std::pair< unsigned int, unsigned int> get_size( const std::string& size )
{
    std::vector< std::string > sizeVector = comma::split( size, 'x' );
    if( sizeVector.size() != 2u ) { std::cerr << "image-undistort-map: please specify --size as <width>x<height>" << std::endl; exit( 1 ); }
    unsigned int width = boost::lexical_cast< unsigned int >( sizeVector[0] );
    unsigned int height = boost::lexical_cast< unsigned int >( sizeVector[1] );
    return std::make_pair( width, height );
}

int main(int argc, char *argv[])
{
    try
    {
        comma::command_line_options options( argc, argv );
        if( options.exists( "--help,-h" ) ) { usage(); }

        // intrisic parameters
        std::string intrinsicString = options.value< std::string >( "--intrinsics" );
        std::vector< std::string > intrinsics = comma::split( intrinsicString, ',' );
        if( intrinsics.size() != 4u )
        {
            std::cerr << "image-undistort-map: please specify intrinsic parameters as fx,fy,cx,cy" << std::endl; exit( 1 );
        }
        double fx = boost::lexical_cast< double >( intrinsics[0] );
        double fy = boost::lexical_cast< double >( intrinsics[1] );
        double cx = boost::lexical_cast< double >( intrinsics[2] );
        double cy = boost::lexical_cast< double >( intrinsics[3] );
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0,  cx,
                                                         0,  fy, cy,
                                                         0,  0,  1);
        // distortion parameters
        std::string distortionString = options.value< std::string >( "--distortion" );
        std::vector< std::string > distortion = comma::split( distortionString, ',' );
        if( distortion.size() != 5u )
        {
            std::cerr << "image-undistort-map: please specify intrinsic parameters as k1,k2,p1,p2,k3" << std::endl; exit( 1 );
        }
        double k1 = boost::lexical_cast< double >( distortion[0] );
        double k2 = boost::lexical_cast< double >( distortion[1] );
        double p1 = boost::lexical_cast< double >( distortion[2] );
        double p2 = boost::lexical_cast< double >( distortion[3] );
        double k3 = boost::lexical_cast< double >( distortion[4] );
        cv::Mat distCoeffs = ( cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3 );
        
        // image size
        std::string sizeString = options.value< std::string >( "--size" );
        std::pair< unsigned int, unsigned int> size = get_size( sizeString );

        // compute maps
        cv::Mat map1;
        cv::Mat map2;
        cv::initUndistortRectifyMap( cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size( size.first, size.second ), CV_32FC1, map1, map2 );

        std::vector< std::string > unnamed = options.unnamed( "", "--intrinsics,--distortion,--size" );
        if( unnamed.empty() ) { std::cerr << "image-undistort-map: please specify output file name" << std::endl; exit( 1 ); }
        std::ostream* os = &std::cout;
        boost::scoped_ptr< std::ofstream > ofs;
        if( unnamed[0] != "-" )
        {
            ofs.reset( new std::ofstream( unnamed[0].c_str() ) );
            os = ofs.get();
        }
        os->write( (char*)map1.data, map1.size().width * map1.size().height * 4 ); // type is CV_32FC1
        os->write( (char*)map2.data, map2.size().width * map2.size().height * 4 );
        return 0;
    }
    catch( std::exception& e )
    {
        std::cerr << "image-undistort-map: "<< e.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "image-undistort-map: unknown exception" << std::endl;
    }
    return 1;
}


