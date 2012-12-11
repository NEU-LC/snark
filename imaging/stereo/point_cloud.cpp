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

#include <snark/imaging/stereo/point_cloud.h>
#include <iostream>

namespace snark { namespace imaging {

static const unsigned int numberOfDisparities = 80; // TODO config ?

/// default constructor
point_cloud::point_cloud ( unsigned int channels )
{
    m_sgbm.SADWindowSize = 5; 
    m_sgbm.minDisparity = 0;
    m_sgbm.P1 = 8*channels*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
    m_sgbm.P2 = 32*channels*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
    m_sgbm.numberOfDisparities = numberOfDisparities;
    m_sgbm.uniquenessRatio = 10;
    m_sgbm.speckleWindowSize = 1000; 
    m_sgbm.speckleRange = 16;
    m_sgbm.disp12MaxDiff = 1;
    m_sgbm.preFilterCap = 63;
    m_sgbm.fullDP = true; 
}

/// constructor from pre-configured sgbm struct
point_cloud::point_cloud ( const cv::StereoSGBM& sgbm ):
    m_sgbm( sgbm )
{

}

/// compute disparity and point cloud
/// @param Q Q matrix computed with stereo recfity
/// @param left rectified left image
/// @param right rectified right image
cv::Mat point_cloud::get ( const cv::Mat& Q, const cv::Mat& left, const cv::Mat& right )
{
    m_disparity = get_disparity( left, right );
    cv::Mat points;
    cv::reprojectImageTo3D( m_disparity, points, Q, true);
    return points;
}

/// compute disparity only
/// @param left rectified left image
/// @param right rectified right image
cv::Mat point_cloud::get_disparity ( const cv::Mat& left, const cv::Mat& right )
{
    cv::Mat disparity;
    m_sgbm.P1 = 8*left.channels()*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
    m_sgbm.P2 = 32*left.channels()*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
    m_sgbm( left, right, disparity );
    return disparity;
}



} }

