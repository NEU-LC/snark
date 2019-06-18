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

#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "frequency_domain.h"

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

