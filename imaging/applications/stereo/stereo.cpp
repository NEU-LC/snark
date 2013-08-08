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


#include "stereo.h"
#include <opencv2/highgui/highgui.hpp>

namespace snark { namespace imaging {

stereo::stereo ( const snark::imaging::camera_parser& left, const snark::imaging::camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv, bool input_rectified ):
    m_rotation( right.rotation() * left.rotation().transpose() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify( left.camera(), left.distortion(), right.camera(), right.distortion(), width, height, m_rotation, m_translation, input_rectified ),
    m_input_rectified( input_rectified ),
    m_frame_counter( 0 )
{
    if( csv.binary() )
    {
        m_binary.reset( new comma::csv::binary< colored_point >( csv ) );
        m_output.resize( csv.format().size() );
    }
    else
    {
        m_ascii.reset( new comma::csv::ascii< colored_point >( csv ) );
    }
}

stereo::stereo ( const camera_parser& left, const camera_parser& right,
                 const cv::Mat& left_x, const cv::Mat& left_y, const cv::Mat& right_x, const cv::Mat& right_y,
                 const comma::csv::options& csv, bool input_rectified ):
    m_rotation( Eigen::Matrix3d::Identity() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify( left.camera(), right.camera(), m_translation, left_x, left_y, right_x, right_y, input_rectified ),
    m_input_rectified( input_rectified ),
    m_frame_counter( 0 )
{
    if( csv.binary() )
    {
        m_binary.reset( new comma::csv::binary< colored_point >( csv ) );
        m_output.resize( csv.format().size() );
    }
    else
    {
        m_ascii.reset( new comma::csv::ascii< colored_point >( csv ) );
    }
}


void stereo::process( const cv::Mat& left, const cv::Mat& right, const cv::StereoSGBM& sgbm, boost::posix_time::ptime time )
{
    snark::imaging::point_cloud cloud( sgbm );

    cv::Mat points;
    cv::Mat leftRectified, rightRectified;
    if (!m_input_rectified)
    {
        leftRectified = m_rectify.remap_left( left );
        rightRectified = m_rectify.remap_right( right );
        points = cloud.get( m_rectify.Q(), leftRectified, rightRectified );
    }
    else
    {
        leftRectified = left;
        rightRectified = right;
        points = cloud.get( m_rectify.Q(), left, right );
    }

    for( int i = 0; i < points.rows; i++ )
    {
       for( int j = 0; j < points.cols; j++ )
       {
            cv::Point3f point = points.at< cv::Point3f >( i, j );

            if( std::fabs( point.z ) < 10000 ) // CV uses 10,000 as invalid. TODO config max distance ?
            {
                point *= 16.0; // disparity has a factor 16
                cv::Vec3b color = leftRectified.at< cv::Vec3b >( i, j );
                colored_point point_color( point.x, point.y, point.z, color[2], color[1], color[0] );
                point_color.time = time;
                point_color.block = m_frame_counter;
                if( m_binary )
                {
                    m_binary->put( point_color, &m_output[0] );
                    std::cout.write( &m_output[0], m_output.size() );
                    std::cout.flush();
                }
                else
                {
                    std::string line;
                    m_ascii->put( point_color, line );
                    std::cout << line << std::endl;
                }
            }
        }
    }
    m_frame_counter++;
}


} }


