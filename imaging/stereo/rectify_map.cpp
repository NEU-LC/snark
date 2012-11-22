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

#include <snark/imaging/stereo/rectify_map.h>
#include "rectify_map.h"
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace snark { namespace imaging {

/// constructor    
rectify_map::rectify_map ( const Eigen::Matrix3d& leftCamera, const Eigen::Vector4d& leftDistortion, const Eigen::Matrix3d& rightCamera, const Eigen::Vector4d& rightDistortion,
                           const Eigen::Vector2i imageSize, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation )
{
    cv::eigen2cv( leftCamera, m_leftCamera );
    cv::eigen2cv( leftDistortion, m_leftDistortion );
    cv::eigen2cv( rightCamera, m_rightCamera );
    cv::eigen2cv( rightDistortion, m_rightDistortion );
    m_imageSize.width = imageSize.x();
    m_imageSize.height = imageSize.y();
    cv::eigen2cv( rotation, m_rotation );
    cv::eigen2cv( translation, m_translation );
    
    cv::stereoRectify( m_leftCamera, m_leftDistortion, m_rightCamera, m_rightDistortion, m_imageSize, m_rotation, m_translation,
                       m_R1, m_R2, m_P1, m_P2, m_Q );

    cv::initUndistortRectifyMap( m_leftCamera, m_leftDistortion, m_R1, m_P1, m_imageSize, CV_16SC2, m_map11, m_map12 );
    cv::initUndistortRectifyMap( m_rightCamera, m_rightDistortion, m_R2, m_P2, m_imageSize, CV_16SC2, m_map21, m_map22);
}

/// remap left image
cv::Mat rectify_map::remap_left ( const cv::Mat left ) const
{
    cv::Mat result;
    cv::remap( left, result, m_map11, m_map12, cv::INTER_LINEAR );
    return result;
}

/// remap right image
cv::Mat rectify_map::remap_right ( const cv::Mat right ) const
{
    cv::Mat result;
    cv::remap( right, result, m_map21, m_map22, cv::INTER_LINEAR );
    return result;
}

    
} }
