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


#include <fstream>
#include <iostream>
#include <boost/filesystem/operations.hpp>
#include <boost/scoped_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/exception.h>
#include <comma/name_value/ptree.h>
#include <comma/name_value/serialize.h>
#include <comma/string/split.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>

#include "../camera/pinhole.h"
#include "../camera/traits.h"

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
    std::cerr << "    --camera-config,--camera,--config,-c=<parameters>: camera configuration" << std::endl;
    std::cerr << "        <parameters>: filename of json configuration file or ';'-separated path-value pairs" << std::endl;
    std::cerr << "                      e.g: --config=\"focal_length/x=123;focal_length/y=123.1;...\"" << std::endl;
    std::cerr << "    --output-config,--sample-config: output sample config and exit" << std::endl;
    std::cerr << "    --intrinsics <fx,fy,cx,cy>: deprecated, use --config; intrinsic parameters in pixel" << std::endl;
    std::cerr << "    --distortion <k1,k2,p1,p2,k3>: deprecated, use --config; distortion parameters" << std::endl;
    std::cerr << "    --size <width>x<height>: deprecated, use --config; image size in pixel" << std::endl;
    std::cerr << std::endl;
    std::cerr << "example: " << std::endl;
    std::cerr << "    image-undistort-map --intrinsics \"830.2,832.4,308.8,232.6\" --distortion \"-0.42539,0.14800,0.00218,0.00061\" --size 1280x960 bumblebee-undistort-map.bin " << std::endl;
    std::cerr << std::endl;
    exit( 1 );
}

static snark::camera::pinhole make_pinhole( const std::string& config_parameters )
{
    snark::camera::pinhole pinhole;
    if( config_parameters.find_first_of( '=' ) == std::string::npos )
    {
        const std::vector< std::string >& v = comma::split( config_parameters, ":#@" );
        if( v.size() == 1 ) { comma::read_json( pinhole, config_parameters, true ); }
        else { comma::read_json( pinhole, config_parameters.substr( 0, config_parameters.size() - v.back().size() - 1 ), v.back(), true ); }
    }
    else
    {
        boost::property_tree::ptree p;
        comma::property_tree::from_path_value_string( config_parameters, '=', ';', comma::property_tree::path_value::no_check, true );
        comma::from_ptree from_ptree( p, true );
        comma::visiting::apply( from_ptree ).to( pinhole );
    }
    if( !pinhole.principal_point ) { pinhole.principal_point = pinhole.image_centre(); }
    return pinhole;
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
        std::string config_parameters = options.value< std::string >( "--camera-config,--camera,--config,-c", "" );
        double fx;
        double fy;
        double cx;
        double cy;
        double k1;
        double k2;
        double p1;
        double p2;
        double k3;
        std::pair< unsigned int, unsigned int> size;
        if( config_parameters.empty() )
        {
            // intrisic parameters
            std::string intrinsicString = options.value< std::string >( "--intrinsics" );
            std::vector< std::string > intrinsics = comma::split( intrinsicString, ',' );
            if( intrinsics.size() != 4u )
            {
                std::cerr << "image-undistort-map: please specify intrinsic parameters as fx,fy,cx,cy" << std::endl; exit( 1 );
            }
            fx = boost::lexical_cast< double >( intrinsics[0] );
            fy = boost::lexical_cast< double >( intrinsics[1] );
            cx = boost::lexical_cast< double >( intrinsics[2] );
            cy = boost::lexical_cast< double >( intrinsics[3] );
            
            // distortion parameters
            std::string distortionString = options.value< std::string >( "--distortion" );
            std::vector< std::string > distortion = comma::split( distortionString, ',' );
            if( distortion.size() != 5u )
            {
                std::cerr << "image-undistort-map: please specify intrinsic parameters as k1,k2,p1,p2,k3" << std::endl; exit( 1 );
            }
            k1 = boost::lexical_cast< double >( distortion[0] );
            k2 = boost::lexical_cast< double >( distortion[1] );
            p1 = boost::lexical_cast< double >( distortion[2] );
            p2 = boost::lexical_cast< double >( distortion[3] );
            k3 = boost::lexical_cast< double >( distortion[4] );
            
            // image size
            std::string sizeString = options.value< std::string >( "--size" );
            size = get_size( sizeString );
        }
        else
        {
            snark::camera::pinhole pinhole = make_pinhole( config_parameters );
            Eigen::Vector2d pixel_size = pinhole.pixel_size();
            fx = pinhole.focal_length / pixel_size.x();
            fy = pinhole.focal_length / pixel_size.y();
            cx = pinhole.principal_point->x();
            cy = pinhole.principal_point->y();
            k1 = pinhole.distortion.radial.k1;
            k2 = pinhole.distortion.radial.k2;
            k3 = pinhole.distortion.radial.k3;
            p1 = pinhole.distortion.tangential.p1;
            p2 = pinhole.distortion.tangential.p2;
            size.first = pinhole.image_size.x();
            size.second = pinhole.image_size.y();
        }
        
        cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << fx, 0,  cx,
                                                         0,  fy, cy,
                                                         0,  0,  1);
        cv::Mat distCoeffs = ( cv::Mat_<double>(5,1) << k1, k2, p1, p2, k3 );
        
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


