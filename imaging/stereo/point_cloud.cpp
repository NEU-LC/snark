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

#include "point_cloud.h"

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

