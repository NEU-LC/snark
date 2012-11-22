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


#include "point_cloud.h"
#include <snark/imaging/stereo/point_cloud.h>

namespace snark { namespace imaging {

static const unsigned int numberOfDisparities = 80; // TODO config ?

    
point_cloud::point_cloud ( const cv::Mat& Q, unsigned int channels ):
    m_Q( Q )
{
    m_sgbm.SADWindowSize = 3; // victor has 5
    m_sgbm.minDisparity = 0;
    m_sgbm.P1 = 8*channels*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
    m_sgbm.P2 = 32*channels*m_sgbm.SADWindowSize*m_sgbm.SADWindowSize;
    m_sgbm.numberOfDisparities = numberOfDisparities;
    m_sgbm.uniquenessRatio = 10;
    m_sgbm.speckleWindowSize = 100; // victor has 1000
    m_sgbm.speckleRange = 32; // victor has 16
    m_sgbm.uniquenessRatio = 15; // victor has 10
    m_sgbm.disp12MaxDiff = 1;
    m_sgbm.preFilterCap = 63;
    m_sgbm.fullDP = true;
}

cv::Mat point_cloud::get ( const cv::Mat& right, const cv::Mat& left )
{
    m_sgbm( right, left, m_disparity );
    cv::Mat points;
    cv::reprojectImageTo3D( m_disparity, points, m_Q, true);
}




} }

