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

#ifndef SNARK_IMAGING_APPLICATIONS_STEREO_RECTIFIED_H
#define SNARK_IMAGING_APPLICATIONS_STEREO_RECTIFIED_H

#include "rectify_map.h"
#include "point_cloud.h"
#include "../../../imaging/cv_mat/serialization.h"
#include "parameters.h"

namespace snark { namespace imaging {

/// output disparity image to stdout from stereo pair
class rectified
{
public:
    rectified( const camera_parser& left, const camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv, bool input_rectified );
    rectified( const camera_parser& left, const camera_parser& right,
            const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
            const comma::csv::options& csv, bool input_rectified );

    void process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time = boost::posix_time::ptime() );
    cv::Mat concatenate( const cv::Mat& left, const cv::Mat& right );
private:
    Eigen::Matrix3d m_rotation;
    Eigen::Vector3d m_translation;
    rectify_map m_rectify;
    bool m_input_rectified;
    cv_mat::serialization m_serialization;
};

} }


#endif // SNARK_IMAGING_APPLICATIONS_STEREO_RECTIFIED_H
