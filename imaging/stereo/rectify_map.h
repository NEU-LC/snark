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
// 3. All advertising materials mentioning features or use of this software
//    must display the following acknowledgement:
//    This product includes software developed by the The University of Sydney.
// 4. Neither the name of the The University of Sydney nor the
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


#ifndef SNARK_IMAGING_STEREO_RECTIFY_MAP_H
#define SNARK_IMAGING_STEREO_RECTIFY_MAP_H

#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>

namespace snark { namespace imaging {

/// compute rectify maps for a camera pair    
class rectify_map
{
public:

    typedef Eigen::Matrix< double, 5, 1 > Vector5d;
    
    rectify_map( const Eigen::Matrix3d& leftCamera, const Vector5d& leftDistortion, const Eigen::Matrix3d& rightCamera, const Vector5d& rightDistortion,
                 unsigned int imageWidth, unsigned int imageHeight, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation );

    rectify_map( const Eigen::Matrix3d& leftCamera, const Eigen::Matrix3d& rightCamera, const Eigen::Vector3d& translation,
                 const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y );

    /// return the Q matrix
    const cv::Mat& Q() const { return m_Q; }
    
    const cv::Mat& map11() const { return m_map11; };
    const cv::Mat& map12() const { return m_map12; };
    const cv::Mat& map21() const { return m_map21; };
    const cv::Mat& map22() const { return m_map22; };

    cv::Mat remap_left( const cv::Mat& left ) const;
    cv::Mat remap_right( const cv::Mat& right ) const;

private:
    cv::Mat m_leftCamera;
    cv::Mat m_leftDistortion;
    cv::Mat m_rightCamera;
    cv::Mat m_rightDistortion;
    cv::Size m_imageSize;
    cv::Mat m_rotation;
    cv::Mat m_translation;

    cv::Mat m_R1;
    cv::Mat m_R2;
    cv::Mat m_P1;
    cv::Mat m_P2;
    cv::Mat m_Q;

    cv::Mat m_map11;
    cv::Mat m_map12;
    cv::Mat m_map21;
    cv::Mat m_map22;
};
    
} }

#endif // SNARK_IMAGING_STEREO_RECTIFY_MAP_H
