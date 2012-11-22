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

#ifndef SNARK_IMAGING_STEREO_POINT_CLOUD_H
#define SNARK_IMAGING_STEREO_POINT_CLOUD_H

#include <opencv2/calib3d/calib3d.hpp>

namespace snark { namespace imaging {

/// get point cloud from rectified stereo images
class point_cloud
{
public:
    point_cloud( const cv::Mat& Q, unsigned int channels = 3 );

    cv::Mat get( const cv::Mat& right, const cv::Mat& left );
    const cv::Mat& disparity() const { return m_disparity; }
    
private:
    cv::Mat m_Q;
    cv::StereoSGBM m_sgbm;
    cv::Mat m_disparity;
};

} }

#endif // SNARK_IMAGING_STEREO_POINT_CLOUD_H
