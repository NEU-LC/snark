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

#include <snark/imaging/region_properties.h>
#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// run regionprops on an image
int main( int ac, char** av )
{
    if( ac != 2 )
    {
        std::cerr << "usage: " << av[0] << " <image> " << std::endl;
        return 1;
    }
    cv::Mat image = cv::imread( av[1], 1 );
    cv::Mat binary;
    cv::threshold( image, binary, 128, 255, CV_THRESH_BINARY_INV );
    snark::imaging::region_properties properties( binary );
    properties.show( image );
    cv::imshow( "image", image );
    cv::waitKey();
    return 0;
}
