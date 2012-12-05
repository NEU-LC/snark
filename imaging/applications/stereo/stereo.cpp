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

#include "stereo.h"
#include <opencv2/highgui/highgui.hpp>

namespace snark { namespace imaging {

stereo::stereo ( const snark::imaging::camera_parser& left, const snark::imaging::camera_parser& right, unsigned int width, unsigned int height, const comma::csv::options& csv ):
    m_rotation( right.rotation() * left.rotation().transpose() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify ( left.camera(), left.distortion(), right.camera(), right.distortion(), width, height, m_rotation, m_translation ),
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
                 const comma::csv::options& csv ):
    m_rotation( Eigen::Matrix3d::Identity() ),
    m_translation( right.translation() - left.translation() ),
    m_rectify ( left.camera(), right.camera(), m_translation, left_x, left_y, right_x, right_y ),
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


void stereo::process( const cv::Mat& left, const cv::Mat& right, boost::posix_time::ptime time )
{
    cv::Mat leftRectified = m_rectify.remap_left( left );
    cv::Mat rightRectified = m_rectify.remap_right( right );

//     cv::imshow( "left", leftRectified );
//     cv::imshow( "right", rightRectified );
//     cv::imwrite( "left-rectified.png", leftRectified );
//     cv::imwrite( "right-rectified.png", rightRectified );
    
    snark::imaging::point_cloud cloud( m_rectify.Q(), left.channels() );
    cv::Mat points = cloud.get( leftRectified, rightRectified );

    for( int i = 0; i < points.rows; i++ )
    {
       for( int j = 0; j < points.cols; j++ )
       {
            cv::Point3f point = points.at< cv::Point3f >( i, j );
            if( point.z < 100 )
            {
                cv::Vec3b color = leftRectified.at< cv::Vec3b >( i, j );
                colored_point point_color( point.x, point.y, point.z, color[2], color[1], color[1] );
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
    cv::Mat disparity = cloud.disparity();

    cv::Mat disparity8;
    unsigned int numberOfDisparities = 80;
    numberOfDisparities = ((left.cols/8) + 15) & -16;
    disparity.convertTo( disparity8, CV_8U, 255 / ( numberOfDisparities *16.0 ) );
//     cv::imshow( "disparity", disparity8 );
//     cv::imwrite( "disparity.png", disparity8 );
//     cv::waitKey();
}

    
} }


