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


#include "disparity.h"
#include <boost/thread/thread_time.hpp>

namespace snark { namespace imaging {

disparity::disparity ( const snark::imaging::camera_parser& left, const snark::imaging::camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv, bool input_rectified ):
    m_rotation( right.rotation() * left.rotation().transpose() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify( left.camera(), left.distortion(), right.camera(), right.distortion(), width, height, m_rotation, m_translation, input_rectified ),
    m_input_rectified( input_rectified ),
    m_serialization( csv.fields, csv.format() )
{

}

disparity::disparity ( const camera_parser& left, const camera_parser& right,
                 const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
                 const comma::csv::options& csv, bool input_rectified ):
    m_rotation( Eigen::Matrix3d::Identity() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify( left.camera(), right.camera(), m_translation, left_x, left_y, right_x, right_y, input_rectified ),
    m_input_rectified( input_rectified ),
    m_serialization( csv.fields, csv.format() )
{
    
}

void disparity::process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time )
{
    snark::imaging::point_cloud cloud( sgbm );

    cv::Mat disparity;
    if (!m_input_rectified)
    {
        cv::Mat leftRectified = m_rectify.remap_left( left );
        cv::Mat rightRectified = m_rectify.remap_right( right );
        disparity = cloud.get_disparity( leftRectified, rightRectified );
    }
    else
    {
        disparity = cloud.get_disparity( left, right );
    }

    m_serialization.write( std::cout, std::make_pair( time, disparity ) );
}

} }


