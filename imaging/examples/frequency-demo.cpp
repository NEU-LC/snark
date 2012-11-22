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

#include <snark/imaging/frequency_domain.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// run regionprops on an image
int main( int ac, char** av )
{
    if( ac != 3 )
    {
        std::cerr << "usage: " << av[0] << " <image> <mask-size> " << std::endl;
        return 1;
    }
    cv::Mat image = cv::imread( av[1], 0 );
    cv::imshow( "image", image );
    double scale = atof( av[2] );
    unsigned int background = 255;
    unsigned int foreground = 0;
    if( scale < 0 ) // low pass instead of high pass
    {
        scale = -scale;
        background = 0;
        foreground = 255;        
    }
    cv::Mat mask( image.size(), CV_8UC1, background );
    if( image.cols * scale > 1 || image.rows * scale > 1 )
    {
        cv::ellipse( mask, cv::Point( image.cols >> 1, image.rows >> 1 ), cv::Size( image.cols * scale, image.rows * scale ), 0, 0, 360, foreground, -1 );
    }
    snark::imaging::frequency_domain frequency( mask );
    cv::Mat filtered = frequency.filter( image );
    cv::imshow( "frequency", frequency.magnitude() );
    cv::imshow( "filtered", filtered );
    cv::waitKey();
    return 0;
}

