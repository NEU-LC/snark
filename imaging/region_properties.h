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

#ifndef SNARK_IMAGING_REGION_PROPERTIES_H
#define SNARK_IMAGING_REGION_PROPERTIES_H

#include <opencv2/core/core.hpp>

namespace snark{ namespace imaging {

/// do connected components analysis in a binary image
/// similar to the matlab function 'regionprops' 
class region_properties
{
public:
    struct blob
    {
        cv::Point centroid;
        double area;
        double majorAxis;
        double minorAxis;
        double orientation;
        double eccentricity;
        double solidity;
    };
    
    region_properties( const cv::Mat& image, double minArea = 1 );

    void show( cv::Mat& image, bool text = true );
    const std::vector< blob >& blobs() const { return m_blobs; }
    
private:
    double m_minArea;
    std::vector< blob > m_blobs;
};
    
    
} } 

#endif
